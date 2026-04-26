/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 8路红外灰度循迹小车 - FSM + PID 主程序
  * @note           : 接线校准程序已移至 main_test.c，此文件为正式运行文件。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h" // 确保 CubeMX 生成了 usart.h
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "strategy.h" // [!!] 必须包含，因为 set_motor_speed 在此文件中声明了
#include <stdio.h>    
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;      // <--- 已修正
extern DMA_HandleTypeDef hdma_adc1; // <--- 已修正
extern TIM_HandleTypeDef htim4;   // <--- 已修正
extern UART_HandleTypeDef huart1; // <--- 已修正

/* USER CODE BEGIN PV */
// --- [!!] 全局变量 [!!] ---
// ADC DMA 缓冲区 (必须在此处定义，供 strategy.c 外部访问)
uint32_t g_adc_values[8]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void set_motor_speed(int motorID, int32_t speed);
void set_left_speed(int32_t speed);
void set_right_speed(int32_t speed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// [!!] 重定向 printf (虽然本程序没用，但保留)
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init(); 
  /* USER CODE BEGIN 2 */
  // --- [!!] 您的初始化代码从这里开始 [!!] ---
  
  // 启动 4 个 PWM 通道
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // 启动 ADC DMA (连续采集 8 路传感器)
  HAL_ADC_Start_DMA(&hadc1, g_adc_values, 8);
  
  // 初始化 FSM 策略层
  Strategy_Init();
  
  // --- [!!] 您的初始化代码在这里结束 [!!] ---
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 执行 FSM 策略层主循环（感知 -> 状态判断 -> 电机输出）
    Strategy_Run_Main_Loop();
    // --- [!!] 循环代码结束 [!!] ---
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * (此函数由 CubeMX 自动生成，保持不变)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// --- [!!] 您的所有自定义函数从这里开始 [!!] ---
// [!!] 我们必须保留这些函数，因为 while(1) 正在调用它们 [!!]

/**
 * @brief  单独控制一个电机 (核心函数) [已升级]
 * @note   此函数供 strategy.c 调用 (必须放在 main.c)
 */
void set_motor_speed(int motorID, int32_t speed)
{
    uint32_t channel;
    GPIO_TypeDef* port1;
    uint16_t pin1;
    GPIO_TypeDef* port2;
    uint16_t pin2;
    TIM_HandleTypeDef* htim = &htim4; 

    switch (motorID) {
        case 1: // M1 - 左前
            channel = TIM_CHANNEL_1; // PB6
            port1 = GPIOB; pin1 = GPIO_PIN_12; // IN1
            port2 = GPIOB; pin2 = GPIO_PIN_13; // IN2
            break;
        case 2: // M2 - 右前
            channel = TIM_CHANNEL_2; // PB7
            port1 = GPIOB; pin1 = GPIO_PIN_14; // IN3
            port2 = GPIOB; pin2 = GPIO_PIN_15; // IN4
            break;
        case 3: // M3 - 左后
            channel = TIM_CHANNEL_3; // PB8
            port1 = GPIOB; pin1 = GPIO_PIN_0;  // IN1 (L298N-2)
            port2 = GPIOB; pin2 = GPIO_PIN_1;  // IN2 (L298N-2)
            break;
        case 4: // M4 - 右后
            channel = TIM_CHANNEL_4; // PB9
            port1 = GPIOA; pin1 = GPIO_PIN_8;  // [修改] IN3 (L298N-2)
            port2 = GPIOA; pin2 = GPIO_PIN_11; // [修改] IN4 (L298N-2)
            break;
        default:
            return; 
    }

    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;

    if (speed >= 0) { // 前进
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(htim, channel, speed);
    } else { // 后退
        HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(htim, channel, -speed); 
    }
}

/**
 * @brief  设置小车左侧整体速度 (供 strategy.c 调用)
 */
void set_left_speed(int32_t speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;
    set_motor_speed(1, speed); // 左前
    set_motor_speed(3, speed); // 左后
}

/**
 * @brief  设置小车右侧整体速度 (供 strategy.c 调用)
 */
void set_right_speed(int32_t speed)
{
    if (speed > 999) speed = 999;
    if (speed < -999) speed = -999;
    set_motor_speed(2, speed); // 右前
    set_motor_speed(4, speed); // 右后
}

// --- [!!] 您的所有自定义函数在这里结束 [!!] ---
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint8_t line)
{
  /* USER CODE BEGIN 6 */
  /* ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */