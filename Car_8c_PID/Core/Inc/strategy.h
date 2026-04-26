#ifndef __STRATEGY_H
#define __STRATEGY_H

#include "main.h" // 包含 HAL 库和外设句柄

//================================================================
// 1. 传感器校准阈值 (!! 必须调试 !!)
//================================================================
// 您需要用 Keil 的 Live Watch 观察 g_adc_values[] 数组来确定这两个值
#define SENSOR_WHITE_BASELINE   1000  // [!!] 白线基准值: 传感器在白纸上的读数 (例如 800-1000)
#define SENSOR_BLACK_THRESHOLD  3000  // [!!] 黑线判定值: 高于此值，就坚决认为是黑线 (例如 3000)


//================================================================
// 2. PID 循迹参数 (!! 必须调试 !!)
//================================================================
#define PID_KP (0.3f)   // P比例: 8路更敏感，P值可能需要调低
#define PID_KI (0.001f) // I积分
#define PID_KD (0.2f)   // D微分
#define PID_INTEGRAL_LIMIT 500.0f // I积分限幅


//================================================================
// 3. 基础速度与盲跑策略 (!! 必须调试 !!)
//================================================================
#define BASE_SPEED 350       // 基础循迹速度 (0-999)

// [策略] 直角/锐角转弯
#define TURN_SPEED 400       // 原地转弯速度
#define TURN_DELAY_MS 300    // [!!] 原地转弯持续时间(ms)。调试此值以获得90度转弯

// [策略] 十字路口
#define CROSS_SPEED 450      // 冲过十字路口的速度
#define CROSS_DELAY_MS 200   // [!!] 直行冲过路口的时间(ms)
#define CROSS_DETECT_WINDOW_MS 250 // [!!] 在此时间窗(ms)内左右均看到黑线,则认为是十字
#define CROSS_LOCKOUT_MS 150        // [修复2] 退出十字路口后的重入锁定时间(ms), 防止边缘振荡

// [策略] 丁字断线
#define GAP_SPEED 450        // 冲过断线的速度
#define GAP_DELAY_MS 150     // [!!] 直行冲过断线的时间(ms)
#define GAP_DEBOUNCE_FRAMES 3  // [修复3] 连续N帧丢线才确认进入盲跑态, 防ADC噪声误触发

// [策略] 断线端点丁字路口 (当前已改为直接返回巡线)
#define T_TURN_IS_LEFT 0     // 1: 断线后遇到丁字路口默认左转, 0: 默认右转


//================================================================
// FSM 状态机 (内部使用)
//================================================================
typedef enum {
    STATE_LINE_FOLLOW,    // 0: 正常PID循迹
    STATE_CROSSROAD,      // 1: 处理十字路口
    STATE_RIGHT_TURN,     // 2: 处理直角/锐角右转
    STATE_LEFT_TURN,      // 3: 处理直角/锐角左转
    STATE_GAP,            // 4: 处理断线盲跑
    STATE_T_JUNCTION,     // 5: [已弃用] 处理断线后遇到的丁字路口 (现改为返回巡线)
    STATE_STOP            // 6: 停止
} RobotState_t;


//================================================================
// 全局函数声明
//================================================================

/**
 * @brief 策略层初始化 (如果需要)
 */
void Strategy_Init(void);

/**
 * @brief 策略层主循环 (在 main.c 的 while(1) 中调用)
 */
void Strategy_Run_Main_Loop(void);


//================================================================
// 外部依赖声明 (这些函数必须存在于 main.c 中)
//================================================================
extern uint32_t g_adc_values[8]; // DMA 缓冲区
extern UART_HandleTypeDef huart1; // 调试串口

// 电机控制 API (由 main.c 提供)
void set_left_speed(int32_t speed);
void set_right_speed(int32_t speed);


#endif /* __STRATEGY_H */
