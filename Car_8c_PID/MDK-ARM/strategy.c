#include "strategy.h"
#include <stdio.h> // 用于 printf 调试

//================================================================
// 内部私有变量
//================================================================
static RobotState_t g_current_state = STATE_LINE_FOLLOW;
static uint8_t g_sensor_binary[8] = {0}; // 8个传感器的二值化状态 (0=白, 1=黑)
static uint8_t g_black_count = 0;        // 看到黑线的传感器总数

// PID 运算变量
static float g_pid_error = 0;
static float g_pid_last_error = 0;
static float g_pid_integral = 0;
static float g_pid_derivative = 0;


//================================================================
// 内部私有函数声明
//================================================================
static void detect_track_features(void);
static void run_state_machine(void);
static float calculate_line_error(void);
static int32_t compute_pid(float error);
static void car_line_follow_pid(int32_t correction);


/**
 * @brief 策略层初始化
 */
void Strategy_Init(void)
{
    g_current_state = STATE_LINE_FOLLOW;
    printf("Strategy FSM Initialized. State: LINE_FOLLOW\r\n");
}

/**
 * @brief 策略层主循环 (在 main.c 的 while(1) 中调用)
 */
void Strategy_Run_Main_Loop(void)
{
    // 1. 识别赛道特征 (更新 g_current_state)
    detect_track_features();
    
    // 2. 根据当前状态执行动作
    run_state_machine();
}


/**
 * @brief [FSM 大脑] 识别赛道特征，并切换状态
 */
static void detect_track_features(void)
{
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

    // 2. FSM 状态决策
    // [注意] 仅在当前是 STATE_LINE_FOLLOW 时才进行决策
    // 这样可以防止在执行一个动作(如转弯)的中途被再次打断
    if (g_current_state != STATE_LINE_FOLLOW) {
        return;
    }

    // 特征1：十字路口 (包括倾斜的)
    // (6个或更多传感器同时看到黑线)
    if (g_black_count >= 6) {
        g_current_state = STATE_CROSSROAD;
        printf("FSM: -> CROSSROAD\r\n");
        return;
    }
    
    // 特征2：丁字断线/跨越路段 (8个全白)
    if (g_black_count == 0) {
        g_current_state = STATE_GAP;
        printf("FSM: -> GAP\r\n");
        return;
    }

    // 特征3：直角/锐角右转
    // (左侧全白，右侧至少3个黑)
    if (g_sensor_binary[0] == 0 && g_sensor_binary[1] == 0 && g_sensor_binary[2] == 0 &&
        (g_sensor_binary[5] == 1 || g_sensor_binary[6] == 1 || g_sensor_binary[7] == 1) &&
         g_black_count >= 3) // 确保是黑线，而不是噪点
    {
        g_current_state = STATE_RIGHT_TURN;
        printf("FSM: -> RIGHT TURN\r\n");
        return;
    }

    // 特征4：直角/锐角左转
    // (右侧全白，左侧至少3个黑)
    if ((g_sensor_binary[0] == 1 || g_sensor_binary[1] == 1 || g_sensor_binary[2] == 1) &&
         g_sensor_binary[5] == 0 && g_sensor_binary[6] == 0 && g_sensor_binary[7] == 0 &&
         g_black_count >= 3)
    {
        g_current_state = STATE_LEFT_TURN;
        printf("FSM: -> LEFT TURN\r\n");
        return;
    }
    
    // 默认：保持循迹
    g_current_state = STATE_LINE_FOLLOW;
}

/**
 * @brief [FSM 执行器] 运行状态对应的动作
 */
static void run_state_machine(void)
{
    switch (g_current_state)
    {
        case STATE_LINE_FOLLOW: // 状态0：正常PID循迹
        {
            float error = calculate_line_error();
            int32_t correction = compute_pid(error);
            car_line_follow_pid(correction);
            
            // [调试窗口]
            //printf("STATE: FOLLOW | Err: %.1f | Corr: %ld\r\n", error, correction);
            
            // PID循迹需要高频率，所以在这里加短暂延时
            HAL_Delay(10); 
            break;
        }
        
        case STATE_CROSSROAD: // 状态1：十字路口 (策略：盲跑直行)
        {
            printf("ACTION: Crossing... (Speed: %d, Delay: %dms)\r\n", CROSS_SPEED, CROSS_DELAY_MS);
            
            // 策略：高速直行冲过路口
            set_left_speed(CROSS_SPEED);
            set_right_speed(CROSS_SPEED);
            HAL_Delay(CROSS_DELAY_MS); // [!!] 调试此值
            
            g_current_state = STATE_LINE_FOLLOW; // 执行完毕，切回循迹
            break;
        }

        case STATE_RIGHT_TURN: // 状态2：直角/锐角右转 (策略：盲跑转弯)
        {
            printf("ACTION: Turning Right... (Speed: %d, Delay: %dms)\r\n", TURN_SPEED, TURN_DELAY_MS);
            
            // 策略：(可选) 先直行一小步，让车身过线
            set_left_speed(BASE_SPEED);
            set_right_speed(BASE_SPEED);
            HAL_Delay(50); // 冲过转角 (50ms 示例值)
            
            // 策略：原地右转
            set_left_speed(TURN_SPEED);
            set_right_speed(-TURN_SPEED);
            HAL_Delay(TURN_DELAY_MS); // [!!] 调试此值以转 90 度

            g_current_state = STATE_LINE_FOLLOW; // 执行完毕，切回循迹
            break;
        }
        
        case STATE_LEFT_TURN: // 状态3：直角/锐角左转 (策略：盲跑转弯)
        {
            printf("ACTION: Turning Left... (Speed: %d, Delay: %dms)\r\n", TURN_SPEED, TURN_DELAY_MS);
            
            set_left_speed(BASE_SPEED);
            set_right_speed(BASE_SPEED);
            HAL_Delay(50);
            
            set_left_speed(-TURN_SPEED); 
            set_right_speed(TURN_SPEED);
            HAL_Delay(TURN_DELAY_MS);

            g_current_state = STATE_LINE_FOLLOW;
            break;
        }
        
        case STATE_GAP: // 状态4：丁字断线/跨越 (策略：盲跑直行)
        {
            printf("ACTION: Crossing Gap... (Speed: %d, Delay: %dms)\r\n", GAP_SPEED, GAP_DELAY_MS);
            
            // 策略：保持原速直行冲过断线区
            set_left_speed(GAP_SPEED);
            set_right_speed(GAP_SPEED);
            HAL_Delay(GAP_DELAY_MS); // [!!] 调试此值
            
            g_current_state = STATE_LINE_FOLLOW; // 执行完毕，切回循迹
            break;
        }
        
        case STATE_STOP: // 状态5：停止
        default:
        {
            printf("ACTION: STOP!\r\n");
            set_left_speed(0);
            set_right_speed(0);
            break;
        }
    } // end switch(g_current_state)
}


//================================================================
// PID 循迹函数 (内部使用)
//================================================================

/**
 * @brief  [PID核心] 计算循迹误差 (8通道加权平均法)
 */
static float calculate_line_error()
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
    int32_t left_speed = BASE_SPEED - correction;
    int32_t right_speed = BASE_SPEED + correction;

    if (left_speed < 0) left_speed = 0;
    if (left_speed > 999) left_speed = 999;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 999) right_speed = 999;

    set_left_speed(left_speed);
    set_right_speed(right_speed);
}
