#include "imu.h"

#define CALIB_SAMPLE_NUM 2000

void IMU_Calibrate(mpu_device_t* mpu_dev){
    uint16_t calib_num = 0;
    int32_t angle_sum = 0;
    for (uint8_t i = 0; i < CALIB_SAMPLE_NUM; i++)
    {
        if (MPU6050_ReadSensor(mpu_dev)){
            calib_num++;
            angle_sum += mpu_dev->
        }        
    }
    
}