#include "rc_ppm.h"

volatile uint16_t rc_channels[RC_MAX_CHANNELS] = {0};
volatile uint8_t  rc_channel_count = 0;
volatile uint8_t  rc_frame_ready = 0;

static TIM_HandleTypeDef* s_htim = NULL;
static uint32_t last_ts = 0;
static uint8_t  idx = 0;

void RC_PPM_Init(TIM_HandleTypeDef* htim) {
  s_htim = htim;
  last_ts = 0;
  idx = 0;
  rc_channel_count = 0;
  rc_frame_ready = 0;
}

void RC_PPM_OnCapture(TIM_HandleTypeDef* htim, uint32_t ts_us) {
  if (htim != s_htim) return;
  uint32_t dt = ts_us - last_ts;  // TIM2 là 32-bit, trừ tràn an toàn
  last_ts = ts_us;

  if (dt > PPM_SYNC_US) {
    rc_channel_count = idx;
    idx = 0;
    rc_frame_ready = 1;
  } else if (dt >= PPM_MIN_US && dt <= PPM_MAX_US) {
    if (idx < RC_MAX_CHANNELS) {
      rc_channels[idx++] = (uint16_t)dt;
    }
  }
}