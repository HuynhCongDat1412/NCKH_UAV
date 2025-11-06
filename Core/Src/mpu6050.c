#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

//dia chi thanh ghi can thiet
#define MPU_REG_PWR_MGMT_1      0x6B //thanh ghi wake up MPU, set = 0 de khoi dong
#define MPU_REG_LPF_CONFIG      0x1A //thanh ghi cau hinh LPF
#define MPU_REG_GYRO_CONFIG     0x1B //thanh ghi cau hinh do nhay cam bien
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_SMPLRT_DIV      0x19 //thanh ghi cau hinh chu ki lay mau
#define MPU_REG_GYRO_XOUT_H     0x43 //thanh ghi bat dau 6 byte Gyro
#define MPU_REG_ACCEL_XOUT_H    0x3B //thanh ghi bat dau 6 byte Accel

//private define
#define MPU_WAKE_UP_VALUE 0x00
#define RAD2DEG 57.2957795f
#define SQRT_EPSILON 1e-6f //so epsilon rat nho, tranh sprt(0) roi chia cho 0
#define MPU_RETRY_MAX 4

bool MPU6050_Init(mpu_device_t* mpu_dev,const mpu_config_t* mpu_config) {
    //1. luu dia chi i2c
    mpu_dev->i2c_addr = mpu_config->i2c_addr;

    uint8_t val; //bien nhap
    HAL_StatusTypeDef status; //bien kiem tra loi

    //2. khoi dong mpu
    val = MPU_WAKE_UP_VALUE;
    status = I2C_Write8_Retry(mpu_dev->i2c_addr, MPU_REG_PWR_MGMT_1, &val, MPU_RETRY_MAX);
    if (status != HAL_OK) return false;
    HAL_Delay(5);

    //3. Set LPF
    val = mpu_config->lpf_config;
    status = I2C_Write8_Retry(mpu_dev->i2c_addr, MPU_REG_LPF_CONFIG, &val,MPU_RETRY_MAX);
    if (status != HAL_OK) return false;    

    //4. Set Gyro Scale Factor
    val = mpu_config->gyro_scale;
    status = I2C_Write8_Retry(mpu_dev->i2c_addr, MPU_REG_GYRO_CONFIG, &val,MPU_RETRY_MAX);
    if (status != HAL_OK) return false;    

    //5. Set Accel Scale Factor
    val = mpu_config->accel_scale;
    status = I2C_Write8_Retry(mpu_dev->i2c_addr, MPU_REG_ACCEL_CONFIG, &val, MPU_RETRY_MAX);
    if (status != HAL_OK) return false;    

    //6. Set Sample Rate
    val = mpu_config->sample_rate_div;
    status = I2C_Write8_Retry(mpu_dev->i2c_addr, MPU_REG_SMPLRT_DIV, &val, MPU_RETRY_MAX);
    if (status != HAL_OK) return false;    

    //7. tuong duong gyro scale factor
    switch (mpu_config->gyro_scale){
        case MPU_GYRO_SCALE_250DPS:  mpu_dev->gyro_scale_factor = 131.0f; break;
        case MPU_GYRO_SCALE_500DPS:  mpu_dev->gyro_scale_factor = 65.5f;  break;
        case MPU_GYRO_SCALE_1000DPS: mpu_dev->gyro_scale_factor = 32.8f;  break;
        case MPU_GYRO_SCALE_2000DPS: mpu_dev->gyro_scale_factor = 16.4f;  break;
        default: mpu_dev->gyro_scale_factor = 65.5f; break; // Mặc định an toàn
    }

    //7. tuong duong gyro scale factor
    switch (mpu_config->accel_scale){
        case MPU_ACCEL_SCALE_2G:    mpu_dev->accel_scale_factor = 16384.0f; break;
        case MPU_ACCEL_SCALE_4G:    mpu_dev->accel_scale_factor = 8192.0f;  break;
        case MPU_ACCEL_SCALE_8G:    mpu_dev->accel_scale_factor = 4096.0f;  break;
        case MPU_ACCEL_SCALE_16G:   mpu_dev->accel_scale_factor = 2048.0f;  break;
        default: mpu_dev->accel_scale_factor = 4096.0f; break; // Mặc định an toàn
    }
    return true;
}

bool MPU6050_ReadSensor(mpu_device_t *mpu_dev) {
    uint8_t gyro_buf[6];  // Buffer (bộ đệm) cho Gyro
    uint8_t accel_buf[6]; // Buffer cho Accel
    
    // 1. Đọc 6 byte Gyro (GỌI LỚP 1)
    if (I2C_Read(mpu_dev->i2c_addr, MPU_REG_GYRO_XOUT_H, gyro_buf, 6) != HAL_OK) {
        return false; // Đọc thất bại
    }
    
    // 2. "Phiên dịch" 6 byte thô sang int16_t (Big-Endian)
    int16_t gx_raw = (int16_t)((gyro_buf[0] << 8) | gyro_buf[1]);
    int16_t gy_raw = (int16_t)((gyro_buf[2] << 8) | gyro_buf[3]);
    int16_t gz_raw = (int16_t)((gyro_buf[4] << 8) | gyro_buf[5]);
    
    // 3. "Chế biến" (Scale) và LƯU VÀO HỒ SƠ (mpu_dev)
    // Dùng hệ số scale đã lưu trong "Hồ sơ" lúc Init
    mpu_dev->RateRoll  = (float)gx_raw / mpu_dev->gyro_scale_factor;
    mpu_dev->RatePitch = (float)gy_raw / mpu_dev->gyro_scale_factor;
    mpu_dev->RateYaw   = (float)gz_raw / mpu_dev->gyro_scale_factor;

    // 4. Đọc 6 byte Accel (GỌI LỚP 1)
    if (I2C_Read(mpu_dev->i2c_addr, MPU_REG_ACCEL_XOUT_H, accel_buf, 6) != HAL_OK) {
        return false;
    }
    
    // 5. "Phiên dịch" 6 byte thô sang int16_t
    int16_t ax_raw = (int16_t)((accel_buf[0] << 8) | accel_buf[1]);
    int16_t ay_raw = (int16_t)((accel_buf[2] << 8) | accel_buf[3]);
    int16_t az_raw = (int16_t)((accel_buf[4] << 8) | accel_buf[5]);
    
    // 6. "Chế biến" (Scale)
    float AccX = (float)ax_raw / mpu_dev->accel_scale_factor;
    float AccY = (float)ay_raw / mpu_dev->accel_scale_factor;
    float AccZ = (float)az_raw / mpu_dev->accel_scale_factor;
    
    // 7. Tính toán góc (lấy từ main.c cũ)
    float denomR = sqrtf(AccX*AccX + AccZ*AccZ) + SQRT_EPSILON;
    float denomP = sqrtf(AccY*AccY + AccZ*AccZ) + SQRT_EPSILON;
    
    // 8. LƯU VÀO HỒ SƠ (mpu_dev)
    mpu_dev->AngleRoll  = atanf(AccY / denomR) * RAD2DEG;
    mpu_dev->AnglePitch = -atanf(AccX / denomP) * RAD2DEG;
    
    return true; // Đọc thành công
}
