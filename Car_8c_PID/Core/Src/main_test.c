/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_test.c
  * @brief          : Main program body for Motor Wiring Test
  ******************************************************************************
  * @attention
  *
  * This file is for testing motor wiring.
  * It will drive both motors forward at a slow speed for 5 seconds, then stop.
  * Observe wheel rotation and adjust wiring if necessary.
  *
  * IMPORTANT: This is a temporary test file. To use it, exclude main.c
  *            from the build in your IDE and include this file. After testing,
  *            reverse the process.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ----------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h" // Assuming usart is used for printf debugging

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // For printf
/* USER CODE END Includes */

/* Private typedef ----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1; // Assuming TIM1 is for Left Motor PWM
TIM_HandleTypeDef htim2; // Assuming TIM2 is for Right Motor PWM
UART_HandleTypeDef huart1; // Assuming USART1 is for printf

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void); // Declare Error_Handler
void set_left_speed(int32_t speed);
void set_right_speed(int32_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* HAL之前的所有初始化 */
  HAL_Init();

  /* 配置系统时钟 */
  SystemClock_Config();

  /* 初始化所有配置的外设 */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init(); // Include if printf is used

  /* USER CODE BEGIN 2 */
  // 启动PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 左电机PWM (假设通道1)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 右电机PWM (假设通道1)

  printf("Motor Wiring Test: Running forward for 5 seconds...\r\n");
  
  // 让左右轮同时正转 (速度值200)
  set_left_speed(200);
  set_right_speed(200);
  
  // 持续5秒
  HAL_Delay(5000);
  
  // 停止电机
  set_left_speed(0);
  set_right_speed(0);
  
  printf("Motor Wiring Test: Finished. Please restore your original main.c in the build.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    // Test finished, do nothing.
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
}

/* USER CODE BEGIN 4 */
// 电机控制函数 (请确保GPIO引脚与您的硬件一致)
// 假设左电机控制引脚为 PB12 (IN1), PB13 (IN2), PWM由TIM1_CH1控制
// 假设右电机控制引脚为 PB14 (IN3), PB15 (IN4), PWM由TIM2_CH1控制
void set_left_speed(int32_t speed)
{
    if (speed > 0) { // Forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); // PWM
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN2
    } else if (speed < 0) { // Backward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed); // PWM
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // IN2
    } else { // Stop
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

void set_right_speed(int32_t speed)
{
    if (speed > 0) { // Forward
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed); // PWM
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // IN3
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN4
    } else if (speed < 0) { // Backward
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed); // PWM
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN3
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // IN4
    } else { // Stop
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
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

#ifdef  USE_FULL_ASSERT
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