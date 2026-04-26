/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "strategy.h" // [!!] 包含策略头文件
#include <stdio.h>    // [!!] 包含 printf

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

/* USER CODE BEGIN PV */
// --- [!!] 全局变量 [!!] ---
// ADC DMA 缓冲区 (必须在此处定义，供 strategy.c 外部访问)
uint32_t g_adc_values[8]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// [!!] 重定向 printf 到 UART1 (调试窗口)
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
	
	// 启动 4 个 PWM 通道
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // 启动 ADC DMA (8通道)
  HAL_ADC_Start_DMA(&hadc1, g_adc_values, 8);
  
  // 初始化策略状态机
  Strategy_Init();
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		// 主循环只调用策略函数，所有逻辑都在 strategy.c 中
    Strategy_Run_Main_Loop();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
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
    if (speed < 0) speed = 0;
    set_motor_speed(1, speed); // 左前
    set_motor_speed(3, speed); // 左后
}

/**
 * @brief  设置小车右侧整体速度 (供 strategy.c 调用)
 */
void set_right_speed(int32_t speed)
{
    if (speed > 999) speed = 999;
    if (speed < 0) speed = 0;
    set_motor_speed(2, speed); // 右前
    set_motor_speed(4, speed); // 右后
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
