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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
#define SBUS_FRAME_SIZE    25
#define SBUS_START_BYTE    0x0F
#define SBUS_END_BYTE      0x00

uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE];
uint16_t rc_channels[16];
volatile uint8_t sbus_data_ready_flag = 0;
uint16_t pulse_width;
uint16_t throttle_channel_value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sbus_decode(volatile uint8_t* sbus_frame, uint16_t* channels) {
    // Kiểm tra Start Byte và End Byte
    if (sbus_frame[0] != SBUS_START_BYTE || sbus_frame[24] != SBUS_END_BYTE) {
        return; // Gói tin không hợp lệ
    }

    // Giải mã 16 kênh (mỗi kênh 11 bit)
    channels[0]  = (uint16_t)((sbus_frame[1]    | sbus_frame[2] << 8) & 0x07FF);
    channels[1]  = (uint16_t)((sbus_frame[2] >> 3 | sbus_frame[3] << 5) & 0x07FF);
    channels[2]  = (uint16_t)((sbus_frame[3] >> 6 | sbus_frame[4] << 2 | sbus_frame[5] << 10) & 0x07FF);
    channels[3]  = (uint16_t)((sbus_frame[5] >> 1 | sbus_frame[6] << 7) & 0x07FF);
    channels[4]  = (uint16_t)((sbus_frame[6] >> 4 | sbus_frame[7] << 4) & 0x07FF);
    channels[5]  = (uint16_t)((sbus_frame[7] >> 7 | sbus_frame[8] << 1 | sbus_frame[9] << 9) & 0x07FF);
    channels[6]  = (uint16_t)((sbus_frame[9] >> 2 | sbus_frame[10] << 6) & 0x07FF);
    channels[7]  = (uint16_t)((sbus_frame[10] >> 5 | sbus_frame[11] << 3) & 0x07FF);
    channels[8]  = (uint16_t)((sbus_frame[12]   | sbus_frame[13] << 8) & 0x07FF);
    channels[9]  = (uint16_t)((sbus_frame[13] >> 3 | sbus_frame[14] << 5) & 0x07FF);
    channels[10] = (uint16_t)((sbus_frame[14] >> 6 | sbus_frame[15] << 2 | sbus_frame[16] << 10) & 0x07FF);
    channels[11] = (uint16_t)((sbus_frame[16] >> 1 | sbus_frame[17] << 7) & 0x07FF);
    channels[12] = (uint16_t)((sbus_frame[17] >> 4 | sbus_frame[18] << 4) & 0x07FF);
    channels[13] = (uint16_t)((sbus_frame[18] >> 7 | sbus_frame[19] << 1 | sbus_frame[20] << 9) & 0x07FF);
    channels[14] = (uint16_t)((sbus_frame[20] >> 2 | sbus_frame[21] << 6) & 0x07FF);
    channels[15] = (uint16_t)((sbus_frame[21] >> 5 | sbus_frame[22] << 3) & 0x07FF);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		if (Size == SBUS_FRAME_SIZE)
		{
			sbus_decode(sbus_rx_buffer, rc_channels);
			sbus_data_ready_flag = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);
	}
}

long map(long value, long in_min, long in_max, long out_min, long out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (sbus_data_ready_flag) {
	        sbus_data_ready_flag = 0;
	        throttle_channel_value = rc_channels[2];

	        // pulse_width = map(throttle_channel_value, 398, 1993, 60, 99);
	        // if (pulse_width < 60) pulse_width = 60;
	        // if (pulse_width > 99) pulse_width = 99;

	        // // Cập nhật độ rộng xung PWM
	        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
	        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_width);
	        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_width);
	        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_width);
	    }
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
