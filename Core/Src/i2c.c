#include "i2c.h"

#define I2C_TIMEOUT 3 //timeout cho viec giao tiep i2c
#define I2C_RETRY_DELAY 5 //timeout cho moi lan delay khi doc/ghi thanh ghi
HAL_StatusTypeDef I2C_Write8(uint16_t devAddr,uint8_t reg,uint8_t *pDataIn)
{
    return HAL_I2C_Mem_Write(&hi2c1, devAddr, reg, I2C_MEMADD_SIZE_8BIT, pDataIn, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef I2C_Read(uint16_t devAddr,uint8_t reg,uint8_t *pDataOut, uint8_t dataSize){
    return HAL_I2C_Mem_Read(&hi2c1, devAddr, reg, I2C_MEMADD_SIZE_8BIT, pDataOut, dataSize, I2C_TIMEOUT);
}

HAL_StatusTypeDef I2C_Write8_Retry(uint16_t devAddr,uint8_t reg,uint8_t *pDataIn, uint8_t retryNum) {
    for (int i = 0; i < retryNum; ++i) {
        if (I2C_Write8(devAddr, reg, pDataIn) == HAL_OK) return HAL_OK;
        HAL_Delay(I2C_RETRY_DELAY);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef I2C_Read_Retry(uint16_t devAddr,uint8_t reg,uint8_t *pDataOut, uint8_t dataSize, uint8_t retryNum)
{
    for (uint8_t i = 0; i < retryNum; i++)
    {
        if(I2C_Read(devAddr,reg,pDataOut,dataSize) == HAL_OK) return HAL_OK;
        HAL_Delay(I2C_RETRY_DELAY);
    }
    return HAL_ERROR;    
}