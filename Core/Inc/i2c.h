#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "main.h"

// I2C handle from main
extern I2C_HandleTypeDef hi2c1;
/**
 * @brief write 1 byte (8bit) to register of device address          
 * @param devAddr 7bit address must be shifted to the left before calling the interface 
 * @param reg register address in device, 8 bit register
 * @param pData pointer to value's address 
 * @retval HAL_status (HAL_OK, HAL_ERROR, HAL_TIMEOUT)
 */
HAL_StatusTypeDef I2C_Write8(uint16_t devAddr,uint8_t reg,uint8_t *pDataIn);

/**
 * @brief read 1 byte (8bit) from register of device address
 * @param devAddr 7bit address must be shifted to the left before calling the interface 
 * @param reg register address in device, 8 bit register
 * @param pDataOut Data read from register of Device
 * @retval HAL_status (HAL_OK, HAL_ERROR, HAL_TIMEOUT)
 */
HAL_StatusTypeDef I2C_Read(uint16_t devAddr,uint8_t reg,uint8_t *pDataOut, uint8_t dataSize);

/**
 * @brief I2C_Write8 and I2C_Read with some retry if fail
 * @param retryNum number of retry
 */
HAL_StatusTypeDef I2C_Write8_Retry(uint16_t devAddr,uint8_t reg,uint8_t *pDataIn, uint8_t retryNum);
HAL_StatusTypeDef I2C_Read_Retry(uint16_t devAddr,uint8_t reg,uint8_t *pDataOut, uint8_t dataSize, uint8_t retryNum);
#endif