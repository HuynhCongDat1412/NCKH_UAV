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
#include <string.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD2DEG 57.2957795f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t LoopTimer = 0;
float dt = 0.004f;
//gyro
volatile float RateRoll = 0.0f, RatePitch = 0.0f, RateYaw = 0.0f;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;


//accelerometer
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float AngleCalibrationRoll = 0; // Thêm dòng này
float AngleCalibrationPitch = 0; // Thêm dòng này

//kalman filter
float KalmanAngleRoll=0, 
KalmanUncertaintyAngleRoll=2*2; //gia su sai so 2 do thi phuong sai la 2^2
float KalmanAnglePitch=0, 
KalmanUncertaintyAnglePitch=2*2;

//PID variable
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch,PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch,PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
// chua tune

// PID gains
float PRateRoll = 0.6f;  float PRatePitch = 0.6f;
float PRateYaw  = 2.0f;
float IRateRoll = 0.0f;  float IRatePitch = 0.0f;
float IRateYaw  = 0.0f;
float DRateRoll = 0.0f; float DRatePitch = 0.0f;
float DRateYaw  = 0.0f;

//error = desiredAngle - KalmanAngle
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll,   ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2.0f; float PAnglePitch = 2.0f;
float IAngleRoll = 0.0f; float IAnglePitch = 0.0f;
float DAngleRoll = 0.0f; float DAnglePitch = 0.0f;
//Motor Input
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

#define I2C_TIMEOUT_MS 3 // vi vong lap 4ms nen de timeout 3ms

// vi 0x68 co 7 bit ma ta dia chi can 8 bit nen ta dich 1 bit
static inline HAL_StatusTypeDef wr8(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(hi2c, 0x68<<1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}
static inline HAL_StatusTypeDef rd(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t* buf, uint16_t len) {
  return HAL_I2C_Mem_Read(hi2c, 0x68<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS);
}

//kalman filter lay input la: gia tri truoc do, sai so truoc do, input(rate_gyro), gia tri measure (angle_accelerometer) 
void kalman_1d(float* pKalmanState, 
  float* pKalmanUncertainty, float KalmanInput, 
  float KalmanMeasurement, float dt_s) 
  {
    float KalmanState = *pKalmanState;
    float KalmanUncertainty = *pKalmanUncertainty;
    KalmanState=KalmanState+dt_s*KalmanInput; //B1: du doan tho (raw) goc hien tai (angle(k) = angle(k-1) + dt*toc_do_goc )
    KalmanUncertainty=KalmanUncertainty + dt_s*dt_s * 4 * 4; //B2 du doan tho (raw) sai so hien tai
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //B3: tinh he so kalman gain
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState); //B4: du doan toi uu goc hien tai (angle_kalman)
    KalmanUncertainty=(1-KalmanGain) *  KalmanUncertainty; //B5: du doan toi uu sai so hien tai (saiSo_kalman)
    *pKalmanState=KalmanState; //luu goc du doan_kalman hien tai
    *pKalmanUncertainty=KalmanUncertainty; //luu sai so du doan_kalman hien tai
 }
void gyro_signal(void){
  
  uint8_t b[6];
  uint8_t a[6]; //xem xet bo thang a, dung b cho ca gyro voi acce
  // read register 0x43 -> 0x48 => 6 byte
  // save to b[6], vi data cua gyro 16bit
  //, ma chia ra 2 mang nen b[0] va b[1] la data day du cua rate_x, tuong tu y voi z
  // tuy nhien voi sensing scale factor 65.5/ do/s thi ta chia gia tri doc duoc cho 65.5 => toc do  
  if (rd(&hi2c1, 0x43, b, 6) == HAL_OK) {
    int16_t gx_raw = (int16_t)((b[0] << 8) | b[1]);
    int16_t gy_raw = (int16_t)((b[2] << 8) | b[3]);
    int16_t gz_raw = (int16_t)((b[4] << 8) | b[5]);
    const float sens = 65.5f; // LSB per (deg/s) for FS_SEL=1
    RateRoll  = gx_raw / sens;
    RatePitch = gy_raw / sens;
    RateYaw   = gz_raw / sens;
  }

  //doc accelerometer
  if (rd(&hi2c1, 0x3B, a, 6) == HAL_OK) {
    int16_t AccXLSB = (int16_t)((a[0] << 8) | a[1]);
    int16_t AccYLSB = (int16_t)((a[2] << 8) | a[3]);
    int16_t AccZLSB = (int16_t)((a[4] << 8) | a[5]);


	AccX=(float)AccXLSB/4096;
	AccY=(float)AccYLSB/4096;
	AccZ=(float)AccZLSB/4096;
  float denomR = sqrtf(AccX*AccX + AccZ*AccZ) + 1e-6f; // tranh chia cho 0
  float denomP = sqrtf(AccY*AccY + AccZ*AccZ) + 1e-6f;
  AngleRoll  = atanf(AccY / denomR) * RAD2DEG;
  AnglePitch = -atanf(AccX / denomP) * RAD2DEG;
  }
} 

void pid_equation(float Error, float P , float I, float D, 
                      float PrevError, float PrevIterm, float dt_s) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*dt_s*0.5f;
  if (Iterm > 400) Iterm=400; //anti windup
  else if (Iterm <-400) Iterm=-400;
  float deriv_dt = (dt_s > 0.0005f) ? dt_s : 0.0005f;
  float Dterm = D * (Error - PrevError) / deriv_dt;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
PrevErrorRateRoll=0; PrevErrorRatePitch=0; 
PrevErrorRateYaw=0;
PrevItermRateRoll=0; PrevItermRatePitch=0; 
PrevItermRateYaw=0;
PrevErrorAngleRoll=0; PrevErrorAnglePitch=0; 
PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

static inline uint16_t us_saturate(int v) {
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return (uint16_t)v;
}
static inline void set_motor_us(TIM_HandleTypeDef* htim, uint32_t ch, uint16_t us) {
  __HAL_TIM_SET_COMPARE(htim, ch, us); // 1 tick = 1 us
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  set_motor_us(&htim3,TIM_CHANNEL_1,1000);
  set_motor_us(&htim3,TIM_CHANNEL_2,1000);
  set_motor_us(&htim3,TIM_CHANNEL_3,1000);
  set_motor_us(&htim3,TIM_CHANNEL_4,1000);
  HAL_Delay(250);
  (void)wr8(&hi2c1, 0x6B, 0x00);  // wake mpu 
  (void)wr8(&hi2c1, 0x1A,0x03); // cho thanh ghi 0x1A = 0x03 => low pass filter ~44hz
  (void)wr8(&hi2c1, 0x1B,0x08); // config sensing scale factor la *65.5*/ do/s
  //sample rate cua cam bien binh thuong la 8khz, neu bat lowpassfilter thi doc la 1000hz
  (void)wr8(&hi2c1, 0x19, 0x03); // Set Sample Rate cua cam bien 250hz
  (void)wr8(&hi2c1, 0x1C, 0x10); // set full scale range cua accelerometer la +-8g
  
  //calibrate gyro && accelerometer
  for (RateCalibrationNumber = 0;
	  RateCalibrationNumber < 2000;
	  RateCalibrationNumber++)
  {
	  gyro_signal();
	  RateCalibrationPitch += RatePitch;
	  RateCalibrationRoll += RateRoll;
	  RateCalibrationYaw += RateYaw;

	  AngleCalibrationPitch += AnglePitch;
	  AngleCalibrationRoll += AngleRoll;
	  HAL_Delay(1);
  }
	RateCalibrationPitch/=2000;
	RateCalibrationRoll/=2000;
	RateCalibrationYaw/=2000;

	AngleCalibrationPitch /= 2000;
	AngleCalibrationRoll /= 2000;

  
	LoopTimer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Timing gate at ~250 Hz
    while ((HAL_GetTick() - LoopTimer) < 4) {}
    uint32_t now = HAL_GetTick();
    dt = (now - LoopTimer) * 0.001f; // seconds
    if (dt < 0.001f) dt = 0.001f;    // clamp to avoid div-by-zero/noise
    if (dt > 0.02f)  dt = 0.02f;
    LoopTimer = now;

    gyro_signal();
    //calib gyro && accelerometer
    RatePitch -= RateCalibrationPitch;
    RateRoll -= RateCalibrationRoll;
    RateYaw -= RateCalibrationYaw;
    AnglePitch -= AngleCalibrationPitch;
	  AngleRoll -= AngleCalibrationRoll;

    kalman_1d(&KalmanAngleRoll,  &KalmanUncertaintyAngleRoll,  RateRoll,  AngleRoll,  dt);
    kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, dt);


    // read_receiver();
    // DesiredAngleRoll=0.10*(ReceiverValue[0]-1500);
    // DesiredAnglePitch=0.10*(ReceiverValue[1]-1500);
    // InputThrottle=ReceiverValue[2];
    // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
    // InputThrottle=ReceiverValue[2];
    // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);

    //hardcode test 
    DesiredAngleRoll  = 0.0f;
    DesiredAnglePitch = 0.0f;
    DesiredRateYaw    = 0.0f;

    //PID outer Loop
    ErrorAngleRoll=DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch=DesiredAnglePitch -KalmanAnglePitch;
    pid_equation(ErrorAngleRoll, PAngleRoll, 
    IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll, dt); 		
    DesiredRateRoll=PIDReturn[0]; 
    PrevErrorAngleRoll=PIDReturn[1];
    PrevItermAngleRoll=PIDReturn[2];
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch,
    PrevItermAnglePitch, dt);
    DesiredRatePitch=PIDReturn[0]; 
    PrevErrorAnglePitch=PIDReturn[1];
    PrevItermAnglePitch=PIDReturn[2];

    //PID inner Loop
    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw;
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll,PrevItermRateRoll,dt);
    InputRoll=PIDReturn[0];
    PrevErrorRateRoll=PIDReturn[1]; 
    PrevItermRateRoll=PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch,PrevItermRatePitch,dt);
    InputPitch=PIDReturn[0]; 
    PrevErrorRatePitch=PIDReturn[1]; 
    PrevItermRatePitch=PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw,dt);
    InputYaw=PIDReturn[0]; 
    PrevErrorRateYaw=PIDReturn[1]; 
    PrevItermRateYaw=PIDReturn[2];
    
    int throttle = 1200;
    uint16_t m1 = us_saturate((int)(throttle - InputRoll - InputPitch - InputYaw));
    uint16_t m2 = us_saturate((int)(throttle - InputRoll + InputPitch + InputYaw));
    uint16_t m3 = us_saturate((int)(throttle + InputRoll + InputPitch - InputYaw));
    uint16_t m4 = us_saturate((int)(throttle + InputRoll - InputPitch + InputYaw));

    set_motor_us(&htim3, TIM_CHANNEL_1, m1);
    set_motor_us(&htim3, TIM_CHANNEL_2, m2);
    set_motor_us(&htim3, TIM_CHANNEL_3, m3);
    set_motor_us(&htim3, TIM_CHANNEL_4, m4);
  
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
