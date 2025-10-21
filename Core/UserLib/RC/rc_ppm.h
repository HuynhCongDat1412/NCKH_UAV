#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_MAX_CHANNELS   8
#define PPM_SYNC_US       3000u   // khoảng nghỉ >3ms coi là sync
#define PPM_MIN_US        800u
#define PPM_MAX_US        2200u

extern volatile uint16_t rc_channels[RC_MAX_CHANNELS];
extern volatile uint8_t  rc_channel_count;
extern volatile uint8_t  rc_frame_ready;

void RC_PPM_Init(TIM_HandleTypeDef* htim);
void RC_PPM_OnCapture(TIM_HandleTypeDef* htim, uint32_t ts_us);

#ifdef __cplusplus
}
#endif