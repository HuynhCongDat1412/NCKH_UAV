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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD2DEG 57.2957795f
//#define Calibrate 0
#define ESC_MIN_CCR 50
#define ESC_MAX_CCR 100
// Giới hạn cho vòng Angle (Outer)
#define ANGLE_RATE_LIMIT 250.0f  // deg/s (Dau ra toi da cua outer loop)
#define ANGLE_I_LIMIT    100.0f  //  (Giới hạn I-term của vòng Angle)

// Giới hạn cho vòng Rate (Inner)
// NOTE: giá trị này ở đơn vị "timer counts" (TIM3 CCR), không phải µs.
// TIM3 config: 1 tick = 20µs => 20 ticks = 400µs. RATE_OUT_LIMIT = 20 -> ±20 counts.
#define RATE_OUT_LIMIT   20.0f  // power (Đầu ra tối đa, +/-20 PWM)
#define RATE_I_LIMIT     20.0f  // (Giới hạn I-term +/-20 PWM)

//Thong so giao tiep SBUS
#define SBUS_FRAME_SIZE    25 //Data chuan cua SBUS
#define SBUS_START_BYTE    0x0F //Start byte cua sbus
#define SBUS_END_BYTE      0x00 //end byte cua sbus
#define SBUS_END_BYTE2      0x04 //end byte cua sbus

#define SBUS_MIN 9 //do
#define SBUS_MAX 1641 //do
#define IDLE_US 1050

#define SBUS_ROLL_MIN 712 //do
#define SBUS_ROLL_MAX 1494 //do
#define SBUS_PITCH_MIN 662 //do
#define SBUS_PITCH_MAX 1494 //do
#define SBUS_YAWRATE_MIN 505 //do
#define SBUS_YAWRATE_MAX 1310 //do

#define Calibrate 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint32_t LoopTimer = 0; 
float dt = 0.004f; //timer duy nhat kiem soat sample rate sensor, kalman, pid 
//gyro
uint8_t mpu6050_gyro_flag = 0;
float RateRoll = 0.0f, RatePitch = 0.0f, RateYaw = 0.0f;
float RateCalibrationRoll = 0.0f, RateCalibrationPitch = 0.0f, RateCalibrationYaw = 0.0f;
int RateCalibrationNumber = 0;

//accelerometer
uint8_t value[6];
uint8_t mpu6050_acc_flag = 0;
float AccX = 0.0f, AccY = 0.0f, AccZ = 0.0f;
float AngleRoll = 0.0f, AnglePitch = 0.0f;
float AngleCalibrationRoll = 0.0f; //gia tri calib
float AngleCalibrationPitch = 0.0f; //gia tri calib

//GY-951
uint8_t gy951_rx_buffer[64]; // Bo dem de nhan data (64 byte la an toan)
uint8_t gy951_data_to_process[64];
volatile uint8_t gy951_data_ready_flag = 0;
volatile uint16_t gy951_data_size = 0;

float Yaw, Roll, Pitch;
//kalman filter
float KalmanAngleRoll=0.0f, KalmanUncertaintyAngleRoll=2*2; //gia su sai so 2 do thi phuong sai la 2^2
float KalmanAnglePitch=0.0f, KalmanUncertaintyAnglePitch=2*2;

//PID variable
float PIDReturn[]={0, 0, 0};
//-------Rate/innerLoop
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw; // power (us) dem vao motor
float PrevErrorRateRoll, PrevErrorRatePitch,PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch,PrevItermRateYaw;
// chua tune
// PID gains
// inner loop pid
float PRateRoll = 0.25f;  float PRatePitch = 0.25f;
float PRateYaw  = 0.9f;
float IRateRoll = 0.0f;  float IRatePitch = 0.0f;   // khi bench test: tạm để 0.0f
float IRateYaw  = 0.0f;
float DRateRoll = 0.0f; float DRatePitch = 0.0f;
float DRateYaw  = 0.0f;


//-------Angle/OuterLoop
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll,   ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2.0f;  float PAnglePitch = 2.0f;
float IAngleRoll = 0.0f;  float IAnglePitch = 0.0f;   // sau khi bay ổn định có thể thử 0.02f
float DAngleRoll = 0.0f;  float DAnglePitch = 0.0f;


//Motor Input
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
uint8_t m1 = 0;
uint8_t m2 = 0;
uint8_t m3 = 0;
uint8_t m4 = 0;
//Bien danh cho STM32CubeMonitor theo doi
volatile uint16_t M1_us=1000, M2_us=1000, M3_us=1000, M4_us=1000;
volatile int      Throttle_us = 1200;
volatile float    M1_pct=0, M2_pct=0, M3_pct=0, M4_pct=0, Throttle_pct=0;

//Bien danh cho SBUS
uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE]; //Luu data tho
volatile uint16_t rc_channels[16]; //sau khi decode sbus thi dem vao buffer nay
volatile uint8_t sbus_data_ready_flag = 0; //flag de check trong while
uint16_t pulse_width;
volatile uint16_t throttle_channel_value;
volatile uint32_t last_sbus_ms = 0;
static float throttle_pulse_filt = 50.0f;   // EMA filter

#define I2C_TIMEOUT_MS 3 // vi vong lap 4ms nen de timeout 3ms

//2 ham user de ghi va doc vao i2c
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
    float KalmanState = *pKalmanState; //lay gia tri k-1
    float KalmanUncertainty = *pKalmanUncertainty; ////lay sai so k-1
    KalmanState=KalmanState+dt_s*KalmanInput; //B1: du doan tho (raw) goc hien tai (angle(k) = angle(k-1) + dt*toc_do_goc )
    KalmanUncertainty=KalmanUncertainty + dt_s*dt_s * 4 * 4; //B2 du doan tho (raw) sai so hien tai
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //B3: tinh he so kalman gain
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState); //B4: du doan toi uu goc hien tai (angle_kalman)
    KalmanUncertainty=(1-KalmanGain) *  KalmanUncertainty; //B5: du doan toi uu sai so hien tai (saiSo_kalman)
    *pKalmanState=KalmanState; //luu goc du doan_kalman hien tai
    *pKalmanUncertainty=KalmanUncertainty; //luu sai so du doan_kalman hien tai
 }
void gyro_signal(void){
  


  // read register 0x43 -> 0x48 => 6 byte
  // save to b[6], vi data cua gyro 16bit
  //, ma chia ra 2 mang nen b[0] va b[1] la data day du cua rate_x, tuong tu y voi z
  // tuy nhien voi sensing scale factor 65.5/ do/s thi ta chia gia tri doc duoc cho 65.5 => toc do  
  if (rd(&hi2c1, 0x43, value, 6) == HAL_OK) {
	mpu6050_gyro_flag = 1;
    int16_t gx_raw = (int16_t)((value[0] << 8) | value[1]);
    int16_t gy_raw = (int16_t)((value[2] << 8) | value[3]);
    int16_t gz_raw = (int16_t)((value[4] << 8) | value[5]);
    const float sens = 65.5f; // LSB per (deg/s) for FS_SEL=1
    RateRoll  = gx_raw / sens;
    RatePitch = gy_raw / sens;
    RateYaw   = gz_raw / sens;
  }

  //doc accelerometer
  if (rd(&hi2c1, 0x3B, value, 6) == HAL_OK) {
	mpu6050_acc_flag = 1;
	int16_t AccXLSB = (int16_t)((value[0] << 8) | value[1]);
	int16_t AccYLSB = (int16_t)((value[2] << 8) | value[3]);
	int16_t AccZLSB = (int16_t)((value[4] << 8) | value[5]);


	AccX=(float)AccXLSB/4096;
	AccY=(float)AccYLSB/4096;
	AccZ=(float)AccZLSB/4096;
	float denomR = sqrtf(AccX*AccX + AccZ*AccZ) + 1e-6f; // tranh chia cho 0
	float denomP = sqrtf(AccY*AccY + AccZ*AccZ) + 1e-6f;
	AngleRoll  = atanf(AccY / denomR) * RAD2DEG;
	AnglePitch = -atanf(AccX / denomP) * RAD2DEG;
  }
} 

//pid tranh I bao hoa, tich luy qua nhieu, vuot gioi han i_limit
// tranh gia tri pid out vuot gioi han out_limit
void pid_equation(float Error, float P , float I, float D, 
                  float PrevError, float PrevIterm, float dt_s,
                  float i_limit, float out_limit) {
  float Pterm = P * Error;

  // D dùng dt có chặn đáy
  float deriv_dt = (dt_s > 0.0005f) ? dt_s : 0.0005f; //tranh dt qua nho ~ 0 thi chia bi loi
  float Dterm = D * (Error - PrevError) / deriv_dt;

  // Tính output tạm thời (chưa tích I) để xét bão hòa
  float pre = Pterm + PrevIterm + Dterm;

  float Iterm = PrevIterm;
  // Nếu đang bão hòa và lỗi cùng chiều → không tích I (anti-windup)
  if (!((pre >=  out_limit && Error >  0) ||
        (pre <= -out_limit && Error <  0))) {
    Iterm = PrevIterm + I * (Error + PrevError) * (dt_s * 0.5f); // dien tich hinh thang = (f(k-1) + f(k))*dt/2
    if (Iterm >  i_limit) Iterm =  i_limit;
    if (Iterm < -i_limit) Iterm = -i_limit;
  }

  float out = Pterm + Iterm + Dterm; //gia tri pid
  if (out >  out_limit) out =  out_limit;
  if (out < -out_limit) out = -out_limit;

  PIDReturn[0] = out;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
	PrevErrorRateRoll=0; PrevErrorRatePitch=0;	PrevErrorRateYaw=0;
	PrevItermRateRoll=0; PrevItermRatePitch=0;	PrevItermRateYaw=0;
	PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
	PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

//gioi han gia tri power (us) gui ve esc
static inline uint16_t pulse_saturate(int v) {
  if (v < ESC_MIN_CCR) v = ESC_MIN_CCR;
  if (v > ESC_MAX_CCR) v = ESC_MAX_CCR;
  return (uint16_t)v;
}





//giai ma sbus data raw vao rc_channel[16]
void sbus_decode(volatile uint8_t* sbus_frame, uint16_t* channels) {
    // Kiểm tra Start Byte và End Byte, end byte co loa 0x00 co loai 0x04
    if (sbus_frame[0] != SBUS_START_BYTE || (sbus_frame[24] != SBUS_END_BYTE && sbus_frame[24] != SBUS_END_BYTE2)) {
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



//lay gia tri SBUS -> us, 
static inline uint16_t sbus_to_us(uint16_t v){
  if (v < SBUS_MIN)  v = SBUS_MIN;
  if (v > SBUS_MAX)  v = SBUS_MAX;
  return (uint16_t)(1000 + (v - SBUS_MIN) * (1000.0f / (SBUS_MAX - SBUS_MIN)));
}

//ham call back RX khi nhan data 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		if (Size >= SBUS_FRAME_SIZE)
		{
			sbus_decode(sbus_rx_buffer, rc_channels);
			sbus_data_ready_flag = 1; //dat co trong while
      last_sbus_ms = HAL_GetTick(); //dat moc decode xong SBUS
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // tránh callback nửa khung
	}
//  else if (huart->Instance == USART1) // == Day la GY-951
//    {
//        // Copy data da nhan vao bo dem xu ly
//        memcpy(gy951_data_to_process, gy951_rx_buffer, Size);
//        gy951_data_to_process[Size] = '\0'; // Bien no thanh 1 chuoi string
//
//        gy951_data_size = Size; // Luu lai kich thuoc goi tin
//        gy951_data_ready_flag = 1; // Bat co bao hieu cho while(1)
//
//        // Khoi dong lai DMA de nhan goi tiep theo
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gy951_rx_buffer, 64);
//    }
}

//map gia tri trong khoang [in_min,in_max] -> [out_min, out_max]
long map(long value, long in_min, long in_max, long out_min, long out_max) {
	if (value < in_min) value = in_min;
  if (value > in_max) value = in_max;
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float fmap(long value, long in_min, long in_max, float out_min, float out_max) {
	if (value < in_min) value = in_min;
	if (value > in_max) value = in_max;
  return out_min + (float)(value - in_min) * (out_max - out_min) / (float)(in_max - in_min);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t items_matched;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  #if Calibrate
		TIM3->CCR1=ESC_MAX_CCR; //set the maximum (2ms)
		TIM3->CCR2=ESC_MAX_CCR;
		TIM3->CCR3=ESC_MAX_CCR;
		TIM3->CCR4=ESC_MAX_CCR;

		HAL_Delay(3000); //wait for 1 beep
		TIM3->CCR1=ESC_MIN_CCR; //set the minimum (2ms)
		TIM3->CCR2=ESC_MIN_CCR;
		TIM3->CCR3=ESC_MIN_CCR;
		TIM3->CCR4=ESC_MIN_CCR;
		HAL_Delay(3000); //wait for 1 beep
	#endif

  HAL_Delay(250);
  (void)wr8(&hi2c1, 0x6B, 0x00);  // wake mpu 
  (void)wr8(&hi2c1, 0x1A,0x03); // cho thanh ghi 0x1A = 0x03 => low pass filter ~44hz
  (void)wr8(&hi2c1, 0x1B,0x08); // config sensing scale factor la *65.5*/ do/s
  //sample rate cua cam bien binh thuong la 8khz, neu bat lowpassfilter thi doc la 1000hz
  (void)wr8(&hi2c1, 0x19, 0x03); // Set Sample Rate cua cam bien 250hz
  (void)wr8(&hi2c1, 0x1C, 0x10); // set full scale range cua accelerometer la +-8g

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buffer, SBUS_FRAME_SIZE); //bat callback
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gy951_rx_buffer, 64);

	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);   // tắt ngat giua chung khi dang doc tin nhan uart


  //calibrate gyro && accelerometer
  // doc 2000 lan, tinh trung binh roi tru di gia tri trung binh do
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
  //gia tri calib
	RateCalibrationPitch/=2000;
	RateCalibrationRoll/=2000;
	RateCalibrationYaw/=2000;
	AngleCalibrationPitch /= 2000;
	AngleCalibrationRoll /= 2000;
  
  //chan an toan khi chua khoi dong, can phai gat can throttle ve min roi moi bay duoc
  // Chờ tín hiệu SBUS đầu tiên
// Chờ tín hiệu SBUS đầu tiên
   while (sbus_data_ready_flag == 0) { HAL_Delay(1); }
   // Chờ ga về min
   while ( (uint16_t)map(rc_channels[2], SBUS_MIN, SBUS_MAX, ESC_MIN_CCR, ESC_MAX_CCR) > ESC_MIN_CCR*1.2 ) {
 	  sbus_data_ready_flag = 0;
 	  while (sbus_data_ready_flag == 0) { HAL_Delay(1); }
 }

  reset_pid();
  LoopTimer = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // f he thong ~250 Hz
    while ((HAL_GetTick() - LoopTimer) < 4) {}
    uint32_t now = HAL_GetTick();
    dt = (now - LoopTimer) * 0.001f; // miliseconds -> second
    if (dt < 0.001f) dt = 0.001f;    // clamp to avoid div-by-zero/noise
    if (dt > 0.05f)  dt = 0.05f;  // chan tren de tranh dt lon lam thanh phan I lon,
    LoopTimer = now;   

    if (sbus_data_ready_flag) {
      sbus_data_ready_flag = 0;
      throttle_channel_value = (uint16_t)map(rc_channels[2], SBUS_MIN, SBUS_MAX, ESC_MIN_CCR, ESC_MAX_CCR);
      // Dùng bộ lọc EMA để làm mượt ga ??? co can cai nay khong
      throttle_pulse_filt = (throttle_pulse_filt * 0.75f) + (throttle_channel_value * 0.25f);

      // 2. Ánh xạ các kênh điều khiển (Roll, Pitch, Yaw)
      DesiredAngleRoll = fmap(rc_channels[0], SBUS_ROLL_MIN, SBUS_ROLL_MAX, -30, 30);
      DesiredAnglePitch = fmap(rc_channels[1], SBUS_PITCH_MIN, SBUS_PITCH_MAX, -30, 30);
      DesiredRateYaw = fmap(rc_channels[3], SBUS_YAWRATE_MIN, SBUS_YAWRATE_MAX, -150, 150);
      DesiredRateYaw = 0;
    }     
    //     // nhan input roi nhung cu hardcode test truoc
    // DesiredAngleRoll  = 0.0f;
    // DesiredAnglePitch = 0.0f;
    // DesiredRateYaw    = 0.0f;

    // Logic Fail 1: Mat song >50ms
    bool fs = (HAL_GetTick() - last_sbus_ms) > 50;
    if(fs) {throttle_channel_value = ESC_MIN_CCR;} // neu khong nhan tin hieu lau qua thi tat??? o day nen co logic fail safe khac
    else {throttle_channel_value = (uint16_t)throttle_pulse_filt;}

    //Logic Fail 2: Tat dong co thu cong
      if (throttle_channel_value < ESC_MIN_CCR*1.2)
      {
        throttle_channel_value = ESC_MIN_CCR; // tat dong co
        reset_pid();
        InputRoll = 0; // Tắt điều khiển khi disarm
        InputPitch = 0;
        InputYaw = 0;
        DesiredAngleRoll = 0.0f;
        DesiredAnglePitch = 0.0f;
        DesiredRateYaw = 0.0f;
        m1=ESC_MIN_CCR;
        m2=ESC_MIN_CCR;
        m3=ESC_MIN_CCR;
        m4=ESC_MIN_CCR;
        TIM3->CCR1 = throttle_channel_value;
        TIM3->CCR2 = throttle_channel_value;
        TIM3->CCR3 = throttle_channel_value;
        TIM3->CCR4 = throttle_channel_value;
        continue;
      }
          
    gyro_signal();
    //calib gyro && accelerometer
    if (mpu6050_gyro_flag){
    	mpu6050_gyro_flag = 0;
    	RatePitch -= RateCalibrationPitch;
		RateRoll -= RateCalibrationRoll;
		RateYaw -= RateCalibrationYaw;
    }
    if (mpu6050_acc_flag) {
    	mpu6050_acc_flag = 0;
		AnglePitch -= AngleCalibrationPitch;
		AngleRoll -= AngleCalibrationRoll;
    }

    

    kalman_1d(&KalmanAngleRoll,  &KalmanUncertaintyAngleRoll,  RateRoll,  AngleRoll,  dt);
    kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, dt);    
    
//    if (gy951_data_ready_flag)
//    {
//        gy951_data_ready_flag = 0;
//
//        // Dung sscanf de "boc tach" chuoi data
//        // Dinh dang: #YPR=float,float,float
//        items_matched = sscanf((char*)gy951_data_to_process, "#YPR=%f,%f,%f", &Yaw, &Pitch, &Roll);
//
//
//    }
    //PID outer Loop
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll,
                 PrevErrorAngleRoll, PrevItermAngleRoll, dt,
                 ANGLE_I_LIMIT, ANGLE_RATE_LIMIT); 
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch,
                 PrevErrorAnglePitch, PrevItermAnglePitch, dt,
                 ANGLE_I_LIMIT, ANGLE_RATE_LIMIT); 
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    //PID inner Loop
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
    
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll,
                 PrevErrorRateRoll, PrevItermRateRoll, dt,
                 RATE_I_LIMIT, RATE_OUT_LIMIT); 
    InputRoll = PIDReturn[0]*0.1;
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch, dt,
                 RATE_I_LIMIT, RATE_OUT_LIMIT); 
    InputPitch = 0.05*PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw,
                 PrevErrorRateYaw, PrevItermRateYaw, dt,
                 RATE_I_LIMIT, RATE_OUT_LIMIT); 
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];
//    throttle_channel_value = 60;
    m1 = pulse_saturate((uint16_t)(throttle_channel_value - InputRoll - InputPitch - InputYaw));
    m2 = pulse_saturate((uint16_t)(throttle_channel_value - InputRoll + InputPitch + InputYaw));
    m3 = pulse_saturate((uint16_t)(throttle_channel_value + InputRoll + InputPitch - InputYaw));
    m4 = pulse_saturate((uint16_t)(throttle_channel_value + InputRoll - InputPitch + InputYaw));

    // //gui cho dong co power (us)
     TIM3->CCR1 = m1;
     TIM3->CCR2 = m2;
     TIM3->CCR3 = m3;
     TIM3->CCR4 = m4;

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 50;
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
  huart1.Init.BaudRate = 57600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
