#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include "mpu6050.h"

void IMU_Calibrate(mpu_device_t* mpu_dev);
void IMU_Kalman1D(mpu_device_t* mpu_dev);
#endif