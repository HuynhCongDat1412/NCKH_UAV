#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <stdbool.h>

/** DEFINE PUBLIC
 * @brief provide default address of MPU6050
 * imu.c will use this address
 */
#define MPU6050_ADDR_AD0_LOW (0x68 << 1)
#define MPU6050_ADDR_AD0_HIGH (0x69 << 1)

/** @brief cau hinh lowPassFilter, loc nhieu LPF cam bien*/
typedef enum {
    MPU_LPF_260HZ   = 0x00,
    MPU_LPF_184HZ   = 0x01,
    MPU_LPF_94HZ    = 0x02,
    MPU_LPF_44HZ    = 0x03,
    MPU_LPF_21HZ    = 0x04,
    MPU_LPF_10HZ    = 0x05,
    MPU_LPF_5HZ     = 0x06
} MPU_LPF_e;

/** @brief cau hinh do nhay cua gyro, goc = gia_tri/do nhay */
typedef enum {
    MPU_GYRO_SCALE_250DPS   = 0x00, // (131.0 LSB/dps)
    MPU_GYRO_SCALE_500DPS   = 0x08, // (65.5 LSB/dps)
    MPU_GYRO_SCALE_1000DPS  = 0x10, // (32.8 LSB/dps)
    MPU_GYRO_SCALE_2000DPS  = 0x18, // (16.4 LSB/dps)
}MPU_GyroScale_e;

/** @brief cau hinh do nhay cua accel, goc = gia_tri/do nhay */
typedef enum {
    MPU_ACCEL_SCALE_2G  = 0x00, // (16384 LSB/g)
    MPU_ACCEL_SCALE_4G  = 0x08, // (8192 LSB/g)
    MPU_ACCEL_SCALE_8G  = 0x10, // (4096 LSB/g)
    MPU_ACCEL_SCALE_16G = 0x18  // (2048 LSB/g)
} MPU_AccelScale_e;

/** @brief cau hinh cho mpu, dia chi, lpf, do nhay, chu ki lay mau */
typedef struct {
    uint8_t             i2c_addr;
    MPU_LPF_e           lpf_config;
    MPU_GyroScale_e     gyro_scale;
    MPU_AccelScale_e    accel_scale;
    uint8_t             sample_rate_div; //chu ki lay mau (1kHz/(1+x)) vd x=0x03 => 250Hz
} mpu_config_t;

/** TYPEDEF PUBLIC
 * @brief Struct de quan ly 1 thiet bi mpu
 * imu.c se dung bien nay
 */
typedef struct {
    // configure value
    uint8_t         i2c_addr;
    float           gyro_scale_factor;
    float           accel_scale_factor;
    // value for sensor's output
    float RateRoll, RatePitch, RateYaw;
    float AngleRoll, AnglePitch;
} mpu_device_t;

/**
 * @brief init mpu_device bang cau hinh trong mpu_config
 * @param mpu_dev struct represent for 1 mpu device
 * @param mpu_config cau hinh cho mpu
 * @retval status true = done, false = fail
 */
bool MPU6050_Init(mpu_device_t* mpu_dev,const mpu_config_t* mpu_config);

/**
 * @brief Read Rate from Gyro, read acceleration from accelerometor then calculate for angle
 * @param mpu_dev struct represent for 1 mpu device
 * @retval status true = done, false = fail
 */
bool MPU6050_ReadSensor(mpu_device_t *mpu_dev);
#endif