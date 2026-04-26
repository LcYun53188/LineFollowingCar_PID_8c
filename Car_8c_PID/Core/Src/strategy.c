#include "strategy.h"
#include <stdio.h>    // 用于 printf 调试
#include <stdbool.h>  // 用于 bool 类型

//================================================================
// 内部私有变量
//================================================================
static RobotState_t g_current_state = STATE_LINE_FOLLOW;
static RobotState_t g_last_state = STATE_STOP; // 用于检测状态变化
static uint32_t g_state_enter_time = 0;         // 用于状态计时
static uint32_t g_left_trigger_time = 0;  // 用于时间法判断十字
static uint32_t g_right_trigger_time = 0; // 用于时间法判断十字
static uint8_t g_sensor_binary[8] = {0}; // 8个传感器的二值化状态 (0=白, 1=黑)
static uint8_t g_black_count = 0;        // 看到黑线的传感器总数

// 局部防抖与锁定变量 [修复用]
static uint32_t g_crossroad_exit_time = 0; // [修复2] 记录退出十字态的时刻
static uint8_t  g_gap_debounce_count = 0;  // [修复3] 连续全白帧计数器

// PID 运算变量
static float g_pid_error = 0;
static float g_pid_last_error = 0;
static float g_pid_integral = 0;
static float g_pid_derivative = 0;


//================================================================
// 内部私有函数声明
//================================================================

static void update_sensor_data(void);
static void update_fsm_transitions(void);
static void run_state_machine(void);
static float calculate_line_error(void);
static int32_t compute_pid(float error);
static void car_line_follow_pid(int32_t correction);

// 条件检查 Helper
static bool is_crossroad_time_triggered(void);
static bool is_crossroad_spatial(void);
static bool is_sharp_right_turn(void);
static bool is_sharp_left_turn(void);
static bool is_kinked_right(void);
static bool is_kinked_left(void);

/**
 * @brief 策略层初始化
 */
void Strategy_Init(void)
{
    g_current_state = STATE_LINE_FOLLOW;
    g_last_state = STATE_STOP;
    g_state_enter_time = HAL_GetTick(); // 防止启动时 elapsed_time 虚高
    g_left_trigger_time = 0;
    g_right_trigger_time = 0;
    g_crossroad_exit_time = 0;
    g_gap_debounce_count = 0;
    printf("Strategy FSM Initialized. State: LINE_FOLLOW\r\n");
}

/**
 * @brief 策略层主循环 (在 main.c 的 while(1) 中调用)
 */
void Strategy_Run_Main_Loop(void)
{
    // 1. 获取并预处理感知数据
    update_sensor_data();
    
    // 2. FSM 核心大脑，负责状态切换决策
    update_fsm_transitions();
    
    // 3. 根据当前状态执行动作 (非阵带延时)
    run_state_machine();

    // 4. 主循环节拍延时 (10ms -> 约100Hz)
    HAL_Delay(10);
}

/**
 * @brief 更新传感器数据，进行二值化处理及边沿时间窗计算
 */
static void update_sensor_data(void)
{
    uint32_t current_time = HAL_GetTick();

    // 1. 传感器数据二值化
    g_black_count = 0;
    for (int i = 0; i < 8; i++) {
        if (g_adc_values[i] > SENSOR_BLACK_THRESHOLD) {
            g_sensor_binary[i] = 1;
            g_black_count++;
        } else {
            g_sensor_binary[i] = 0;
        }
    }

    // 2. 边沿触发时间记录 (用于十字路口时间窗过滤)
    bool left_edge_triggered = (g_sensor_binary[0] == 1 || g_sensor_binary[1] == 1);
    bool right_edge_triggered = (g_sensor_binary[6] == 1 || g_sensor_binary[7] == 1);

    if (left_edge_triggered) g_left_trigger_time = current_time;
    if (right_edge_triggered) g_right_trigger_time = current_time;

    // 清理过期的时间戳
    if (g_left_trigger_time > 0 && (current_time - g_left_trigger_time > CROSS_DETECT_WINDOW_MS)) {
        g_left_trigger_time = 0;
    }
    if (g_right_trigger_time > 0 && (current_time - g_right_trigger_time > CROSS_DETECT_WINDOW_MS)) {
        g_right_trigger_time = 0;
    }
}

/**
 * @brief 检查是否满足十字路口的时间窗触发条件
 */
static bool is_crossroad_time_triggered(void)
{
    if (g_left_trigger_time > 0 && g_right_trigger_time > 0) {
        uint32_t diff = (g_left_trigger_time > g_right_trigger_time) ? 
                        (g_left_trigger_time - g_right_trigger_time) : 
                        (g_right_trigger_time - g_left_trigger_time);
        
        // diff 必须在 (20ms, CROSS_DETECT_WINDOW_MS) 区间内：
        // - 下限 20ms: 防止同一帧内左右同时触发导致 diff=0 的虚报
        // - 上限 CROSS_DETECT_WINDOW_MS: 超时的记录已被清零，不会达到加这里
        if (diff > 20 && diff < CROSS_DETECT_WINDOW_MS) {
            g_left_trigger_time = 0;
            g_right_trigger_time = 0;
            return true;
        }
    }
    return false;
}

/**
 * @brief 空间上判定十字路口：必须同时满足左侧和右侧边缘都有覆盖，且总数达标
 */
static bool is_crossroad_spatial(void)
{
    bool has_left = (g_sensor_binary[0] || g_sensor_binary[1]);
    bool has_right= (g_sensor_binary[6] || g_sensor_binary[7]);
    
    // 如果黑点很多(>=5)，并且横跨了左右两端，说明这是真正的十字或丁字横线
    if (g_black_count >= 5 && has_left && has_right) {
        return true;
    }
    return false;
}

/**
 * @brief 检查是否满足锐角/直角右转条件 (严格空间互斥)
 */
static bool is_sharp_right_turn(void)
{
    // 1. 左侧必须绝对干净 (排除十字路口及车身极度偏左的情况)
    bool left_clean = (g_sensor_binary[0] == 0 && g_sensor_binary[1] == 0 && g_sensor_binary[2] == 0);
    // 2. 右侧必须有重墨覆盖面积 (排除普通的微小摇摆)
    bool right_heavy = (g_sensor_binary[5] + g_sensor_binary[6] + g_sensor_binary[7]) >= 2;
    // 3. 点数达标
    return (left_clean && right_heavy && g_black_count >= 3);
}

/**
 * @brief 检查是否满足锐角/直角左转条件 (严格空间互斥)
 */
static bool is_sharp_left_turn(void)
{
    // 1. 右侧必须绝对干净
    bool right_clean = (g_sensor_binary[5] == 0 && g_sensor_binary[6] == 0 && g_sensor_binary[7] == 0);
    // 2. 左侧必须有重墨覆盖面积
    bool left_heavy = (g_sensor_binary[0] + g_sensor_binary[1] + g_sensor_binary[2]) >= 2;
    // 3. 点数达标
    return (right_clean && left_heavy && g_black_count >= 3);
}

/**
 * @brief 特征判定：最外侧黑 + 中心黑 + 该侧中间白 (特定的锐角/大角度折转)
 */
static bool is_kinked_right(void)
{
    // 最外侧[7]黑，中心[3,4]黑，中间[5,6]白
    return (g_sensor_binary[7] == 1 && 
           (g_sensor_binary[3] == 1 || g_sensor_binary[4] == 1) && 
            g_sensor_binary[5] == 0 && g_sensor_binary[6] == 0);
}

static bool is_kinked_left(void)
{
    // 最外侧[0]黑，中心[3,4]黑，中间[1,2]白
    return (g_sensor_binary[0] == 1 && 
           (g_sensor_binary[3] == 1 || g_sensor_binary[4] == 1) && 
            g_sensor_binary[1] == 0 && g_sensor_binary[2] == 0);
}

/**
 * @brief [FSM 大脑] 处理所有状态的转移逻辑
 */
static void update_fsm_transitions(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - g_state_enter_time;

    // --- [全局最高优先级] 十字路口强行介入 ---
    // 无论当前在什么状态(循迹、转弯、盲跑)，一旦识别到横线特征，立即以十字路口对待
    // 排除 STOP 态和 CROSSROAD 本身，并遵守退出锁定期防止边缘振荡
    if (g_current_state != STATE_STOP && g_current_state != STATE_CROSSROAD) {
        
        bool time_hit = is_crossroad_time_triggered();
        bool spatial_hit = is_crossroad_spatial() && 
                           (current_time - g_crossroad_exit_time) > CROSS_LOCKOUT_MS;

        if (time_hit || spatial_hit) {
            g_current_state = STATE_CROSSROAD;
            return; // 立即跳出处理，执行新状态
        }
    }

    switch (g_current_state)
    {
        case STATE_LINE_FOLLOW:
            // 剩下的优先级1: 特定大角度/锐角折转判定 (特定特征)
            if (is_kinked_right()) {
                g_current_state = STATE_RIGHT_TURN;
            }
            else if (is_kinked_left()) {
                g_current_state = STATE_LEFT_TURN;
            }
            // 剩下的优先级2: 间隙/丢线盲跑
            else if (g_black_count == 0) {
                g_gap_debounce_count++;
                if (g_gap_debounce_count >= GAP_DEBOUNCE_FRAMES) {
                    g_gap_debounce_count = 0;
                    g_current_state = STATE_GAP;
                }
            }
            // 剩下的优先级3: 普通锐角/直角转向
            else if (is_sharp_right_turn()) {
                g_gap_debounce_count = 0;
                g_current_state = STATE_RIGHT_TURN;
            }
            else if (is_sharp_left_turn()) {
                g_gap_debounce_count = 0;
                g_current_state = STATE_LEFT_TURN;
            }
            else {
                g_gap_debounce_count = 0;
            }
            break;

        case STATE_CROSSROAD:
            // 退出条件1: 已驶出十字路口宽线区域
            if (g_black_count < 5) {
                g_crossroad_exit_time = HAL_GetTick(); // [修复2] 记录退出时刻
                g_current_state = STATE_LINE_FOLLOW;
            }
            // 退出条件2: 防卡死安全超时
            else if (elapsed_time > CROSS_DELAY_MS) {
                g_crossroad_exit_time = HAL_GetTick(); // [修复2] 记录退出时刻
                g_current_state = STATE_LINE_FOLLOW;
            }
            break;

        case STATE_RIGHT_TURN:
        case STATE_LEFT_TURN:
        case STATE_T_JUNCTION:
            // 退出条件1: 传感器检测到线已经居中 (中间两个传感器检测到黑线)
            // 这里的 100ms 是为了防止刚进入转弯状态时，旧的信号导致立即退出
            if (elapsed_time > 100 && (g_sensor_binary[3] == 1 || g_sensor_binary[4] == 1)) {
                g_current_state = STATE_LINE_FOLLOW;
            }
            // 退出条件2: 安全超时保护 (维持原有 TURN_DELAY_MS 作为最大盲转时间)
            else if (elapsed_time > TURN_DELAY_MS) {
                g_current_state = STATE_LINE_FOLLOW;
            }
            break;

        case STATE_GAP:
            // 退出条件1: 重新捕捉到普通黑线，提前结束盲跑
            if (g_black_count > 0) {
                g_current_state = STATE_LINE_FOLLOW;
            }
            // 退出条件2: 盲跑保护超时
            else if (elapsed_time > GAP_DELAY_MS) {
                g_current_state = STATE_LINE_FOLLOW;
            }
            break;

        case STATE_STOP:
        default:
            break;
    }
}

/**
 * @brief [FSM 执行器] 运行状态对应的动作 (非阻塞)
 */
static void run_state_machine(void)
{
    // 当状态发生改变时，记录进入该状态的时刻
    if (g_current_state != g_last_state) {
        g_state_enter_time = HAL_GetTick();
        printf("FSM: %d -> %d\r\n", g_last_state, g_current_state);

        // [修复1] 任何状态切换时均清零时间戳，防止非循迹期间累积的边沿时间戳
        //         在返回 LINE_FOLLOW 时导致"鬼影十字路口"误触发
        g_left_trigger_time = 0;
        g_right_trigger_time = 0;

        if (g_current_state == STATE_LINE_FOLLOW) {
            // 切回循迹: 清空 PID 历史偏差，防止车身打弯暴走
            g_pid_last_error = 0;
            g_pid_integral = 0;
        }

        g_last_state = g_current_state;
    }

    // 此函数现在是完全非阻塞的，它只根据当前状态设定电机速度
    switch (g_current_state)
    {
        case STATE_LINE_FOLLOW:
        {
            float error = calculate_line_error();
            int32_t correction = compute_pid(error);
            car_line_follow_pid(correction);
            break;
        }
        
        case STATE_CROSSROAD:
        {
            // 动作: 高速直行冲过十字路口。状态切换由 update_fsm_transitions 处理。
            set_left_speed(CROSS_SPEED);
            set_right_speed(CROSS_SPEED);
            break;
        }

        case STATE_RIGHT_TURN:
        {
            // 动作: 原地右转。状态切换由 update_fsm_transitions 处理。
            set_left_speed(TURN_SPEED);
            set_right_speed(-TURN_SPEED);
            break;
        }
        
        case STATE_LEFT_TURN:
        {
            // 动作: 原地左转。状态切换由 update_fsm_transitions 处理。
            set_left_speed(-TURN_SPEED); 
            set_right_speed(TURN_SPEED);
            break;
        }
        
        case STATE_GAP:
        {
            // 动作: 直行过间隙/断线。状态切换由 update_fsm_transitions 处理。
            set_left_speed(GAP_SPEED);
            set_right_speed(GAP_SPEED);
            break;
        }

        case STATE_T_JUNCTION:
        {
            // 动作: 断线尽头的丁字路口，执行默认转向
            if (T_TURN_IS_LEFT) {
                set_left_speed(-TURN_SPEED); 
                set_right_speed(TURN_SPEED);
            } else {
                set_left_speed(TURN_SPEED);
                set_right_speed(-TURN_SPEED);
            }
            break;
        }
        
        case STATE_STOP:
        default:
        {
            set_left_speed(0);
            set_right_speed(0);
            break;
        }
    }

    // 循环延时已移至 Strategy_Run_Main_Loop() 末尾集中管理
}


//================================================================
// PID 循迹函数 (内部使用)
//================================================================

/**
 * @brief  [PID核心] 计算循迹误差 (8通道加权平均法)
 */
static float calculate_line_error(void)
{
    int32_t s[8];
    for (int i = 0; i < 8; i++) {
        // [!!] 校准点: SENSOR_WHITE_BASELINE (来自 strategy.h)
        s[i] = (g_adc_values[i] > SENSOR_WHITE_BASELINE) ? (g_adc_values[i] - SENSOR_WHITE_BASELINE) : 0; 
    }

    // 权重: -700, -500, -300, -100, +100, +300, +500, +700
    int32_t weighted_sum = (s[0] * -700) + (s[1] * -500) + (s[2] * -300) + (s[3] * -100) +
                           (s[4] * 100) + (s[5] * 300) + (s[6] * 500) + (s[7] * 700);
    
    int32_t total_sum = s[0] + s[1] + s[2] + s[3] + s[4] + s[5] + s[6] + s[7];

    if (total_sum == 0) {
        // 全白，但不在 GAP 状态 (例如PID循迹中途的轻微丢线)
        return g_pid_last_error; 
    }
    
    // [!!] 如果传感器反相 (黑线低, 白线高), 
    // 请将 SENSOR_WHITE_BASELINE 改为黑线基准值，并反转所有权重
    
    return (float)weighted_sum / total_sum;
}

/**
 * @brief  [PID核心] 计算PID矫正量
 */
static int32_t compute_pid(float error)
{
    g_pid_error = error;
    
    float p_term = PID_KP * g_pid_error;
    
    g_pid_integral += g_pid_error;
    if (g_pid_integral > PID_INTEGRAL_LIMIT) g_pid_integral = PID_INTEGRAL_LIMIT;
    if (g_pid_integral < -PID_INTEGRAL_LIMIT) g_pid_integral = -PID_INTEGRAL_LIMIT;
    float i_term = PID_KI * g_pid_integral;
    
    g_pid_derivative = error - g_pid_last_error;
    g_pid_last_error = error;
    float d_term = PID_KD * g_pid_derivative;

    int32_t correction = (int32_t)(p_term + i_term + d_term);
    
    return correction;
}

/**
 * @brief  [PID核心] 应用PID结果到电机
 */
static void car_line_follow_pid(int32_t correction)
{
    // [PID循迹] correction 为负 -> 车头偏左 -> 左轮减速、右轮加速，差速追回原地
    // [PID循迹] correction 为正 -> 车头偏右 -> 右轮减速、左轮加速
    int32_t left_speed  = BASE_SPEED + correction;
    int32_t right_speed = BASE_SPEED - correction;

    // [!! 设计决策 !!] 循迹 PID 此处不允许轮子倒转（限制为正向）。
    // 大弯道防御策略：如果某侧轮子要求倒转，说明已严重超出 PID 修正范围，
    // 此时应由 FSM 切入转弯态处理，而非由 PID 强行将轮子倒转拉回。
    if (left_speed  < 0)   left_speed  = 0;
    if (left_speed  > 999) left_speed  = 999;
    if (right_speed < 0)   right_speed = 0;
    if (right_speed > 999) right_speed = 999;

    set_left_speed(left_speed);
    set_right_speed(right_speed);
}

