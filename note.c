// ...existing code...
TIM_HandleTypeDef htim3;
static void MX_TIM3_Init(void);
// ...existing code...

static inline uint16_t us_saturate(int v) {
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return (uint16_t)v;
}
static inline void set_motor_us(TIM_HandleTypeDef* htim, uint32_t ch, uint16_t us) {
  __HAL_TIM_SET_COMPARE(htim, ch, us); // timer tick = 1us
}
// ...existing code...

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100 - 1;       // 100 MHz / 100 = 1 MHz -> 1 tick = 1us
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500 - 1;         // 2500us => 400 Hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;               // khởi động ở 1000us (cutoff)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim3); // đảm bảo GPIO ở chế độ AF cho TIM3 CH1..CH4
}
// ...existing code...

int main(void)
{
  // ...existing code...
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init(); // NEW: bật TIM3 PWM
  // Start PWM channels
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // đưa tất cả motor về cutoff 1000us
//   static inline void set_motor_us(TIM_HandleTypeDef* htim, uint32_t ch, uint16_t us) {
//   __HAL_TIM_SET_COMPARE(htim, ch, us); // timer tick = 1us
// }
  set_motor_us(&htim3, TIM_CHANNEL_1, 1000);
  set_motor_us(&htim3, TIM_CHANNEL_2, 1000);
  set_motor_us(&htim3, TIM_CHANNEL_3, 1000);
  set_motor_us(&htim3, TIM_CHANNEL_4, 1000);

  // ...existing code (IMU init, calib)...

  while (1)
  {
    // ...existing code (dt, IMU, Kalman, PID)...

    // Ví dụ mixer (test không cánh quạt): throttle cố định
    int throttle = 1200; // idle test
    //us_saturate gioi han tat ca ve 1000 - 2000
    uint16_t m1 = us_saturate((int)(throttle - InputRoll - InputPitch - InputYaw));
    uint16_t m2 = us_saturate((int)(throttle - InputRoll + InputPitch + InputYaw));
    uint16_t m3 = us_saturate((int)(throttle + InputRoll + InputPitch - InputYaw));
    uint16_t m4 = us_saturate((int)(throttle + InputRoll - InputPitch + InputYaw));

    set_motor_us(&htim3, TIM_CHANNEL_1, m1);
    set_motor_us(&htim3, TIM_CHANNEL_2, m2);
    set_motor_us(&htim3, TIM_CHANNEL_3, m3);
    set_motor_us(&htim3, TIM_CHANNEL_4, m4);
  }
}
// ...existing code...