// ...existing code...
/* USER CODE BEGIN PD */
#define RAD2DEG 57.2957795f
#define MOTOR_MIN_US       1000
#define MOTOR_MAX_US       2000
// Giới hạn cho vòng Angle (Outer)
#define ANGLE_RATE_LIMIT 250.0f  // deg/s
#define ANGLE_I_LIMIT    100.0f
// Giới hạn cho vòng Rate (Inner)
#define RATE_OUT_LIMIT   400.0f
#define RATE_I_LIMIT     400.0f

// CHUẨN HÓA [-1..1]
#define ANGLE_ERR_MAX    20.0f   // deg (scale lỗi góc)
#define RATE_MAX         250.0f  // deg/s (rate mong muốn max)

static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}
/* USER CODE END PD */
// ...existing code...
// ...existing code...
//PID variable
float PIDReturn[]={0, 0, 0};
//-------Rate/innerLoop
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
// ...existing code...
float DRateRoll = 0.0f; float DRatePitch = 0.0f;
float DRateYaw  = 0.0f;

// >>> XÓA khai báo trùng PIDReturn ở dưới (giữ 1 mảng PIDReturn duy nhất) <<<
// float PIDReturn[]={0, 0, 0};
//-------Angle/OuterLoop
float DesiredAngleRoll, DesiredAnglePitch;
// ...existing code...
// ...existing code...
static void MX_TIM3_Init(void)
{
  // ...existing code...
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;               // khởi động an toàn 1000 µs
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // ...existing code...
}
// ...existing code...
// ...existing code...
  while (1)
  {
    // Timing gate ~250 Hz
    while ((HAL_GetTick() - LoopTimer) < 4) {}
    uint32_t now = HAL_GetTick();
    dt = (now - LoopTimer) / (float)HAL_GetTickFreq(); // ms -> s tự động theo tick freq
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.02f)  dt = 0.02f;
    LoopTimer = now;

    gyro_signal();
    // calib offset
    RatePitch -= RateCalibrationPitch;
    RateRoll  -= RateCalibrationRoll;
    RateYaw   -= RateCalibrationYaw;
    AnglePitch -= AngleCalibrationPitch;
    AngleRoll  -= AngleCalibrationRoll;

    // Kalman
    kalman_1d(&KalmanAngleRoll,  &KalmanUncertaintyAngleRoll,  RateRoll,  AngleRoll,  dt);
    kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, dt);

    // Setpoint cân bằng
    DesiredAngleRoll  = 0.0f;
    DesiredAnglePitch = 0.0f;
    DesiredRateYaw    = 0.0f;

    // ===== Outer (Angle) chuẩn hóa [-1..1] =====
    float eAngleRoll_n  = clampf((DesiredAngleRoll  - KalmanAngleRoll)  / ANGLE_ERR_MAX, -1.0f, 1.0f);
    float eAnglePitch_n = clampf((DesiredAnglePitch - KalmanAnglePitch) / ANGLE_ERR_MAX, -1.0f, 1.0f);

    pid_equation(eAngleRoll_n,  PAngleRoll,  IAngleRoll,  DAngleRoll,
                 PrevErrorAngleRoll,  PrevItermAngleRoll,  dt,
                 1.0f, 1.0f);
    DesiredRateRoll = PIDReturn[0] * RATE_MAX;
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(eAnglePitch_n, PAnglePitch, IAnglePitch, DAnglePitch,
                 PrevErrorAnglePitch, PrevItermAnglePitch, dt,
                 1.0f, 1.0f);
    DesiredRatePitch = PIDReturn[0] * RATE_MAX;
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    // ===== Inner (Rate) chuẩn hóa [-1..1] =====
    float eRateRoll_n  = clampf((DesiredRateRoll  - RateRoll)  / RATE_MAX, -1.0f, 1.0f);
    float eRatePitch_n = clampf((DesiredRatePitch - RatePitch) / RATE_MAX, -1.0f, 1.0f);
    float eRateYaw_n   = clampf((DesiredRateYaw   - RateYaw)   / RATE_MAX, -1.0f, 1.0f);

    pid_equation(eRateRoll_n,  PRateRoll,  IRateRoll,  DRateRoll,
                 PrevErrorRateRoll,  PrevItermRateRoll,  dt,
                 1.0f, 1.0f);
    float uRoll_n = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(eRatePitch_n, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch, dt,
                 1.0f, 1.0f);
    float uPitch_n = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(eRateYaw_n,   PRateYaw,   IRateYaw,   DRateYaw,
                 PrevErrorRateYaw,   PrevItermRateYaw,   dt,
                 1.0f, 1.0f);
    float uYaw_n = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    // Map từ [-1,1] → µs theo headroom động
    int throttle = 1200;            // hoặc InputThrottle nếu có RC
    float headroom = fminf((float)(throttle - MOTOR_MIN_US),
                           (float)(MOTOR_MAX_US - throttle));
    InputRoll  = uRoll_n  * headroom;
    InputPitch = uPitch_n * headroom;
    InputYaw   = uYaw_n   * headroom;

    // Mixer X
    uint16_t m1 = us_saturate((int)(throttle - InputRoll - InputPitch - InputYaw));
    uint16_t m2 = us_saturate((int)(throttle - InputRoll + InputPitch + InputYaw));
    uint16_t m3 = us_saturate((int)(throttle + InputRoll + InputPitch - InputYaw));
    uint16_t m4 = us_saturate((int)(throttle + InputRoll - InputPitch + InputYaw));

    set_motor_us(&htim3, TIM_CHANNEL_1, m1);
    set_motor_us(&htim3, TIM_CHANNEL_2, m2);
    set_motor_us(&htim3, TIM_CHANNEL_3, m3);
    set_motor_us(&htim3, TIM_CHANNEL_4, m4);

    // Telemetry cho CubeMonitor
    Throttle_us = throttle;
    M1_us = m1; M2_us = m2; M3_us = m3; M4_us = m4;
    Throttle_pct = us_to_percent((uint16_t)Throttle_us);
    M1_pct = us_to_percent(M1_us);
    M2_pct = us_to_percent(M2_us);
    M3_pct = us_to_percent(M3_us);
    M4_pct = us_to_percent(M4_us);
  }
// ...existing code...