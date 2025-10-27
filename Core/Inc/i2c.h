#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void I2C_Bus_Init(void);

/**
 * @brief Ghi 1 byte vào 1 thanh ghi (Mem) của một thiết bị I2C
 * @param dev_addr Địa chỉ 7-bit của thiết bị (đã dịch trái 1 bit)
 * @param reg Địa chỉ thanh ghi để ghi
 * @param val Giá trị 1 byte để ghi
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef I2C_Mem_Write8(uint8_t dev_addr, uint8_t reg, uint8_t val);

/**
 * @brief Đọc nhiều byte từ 1 thanh ghi (Mem) của một thiết bị I2C
 * @param dev_addr Địa chỉ 7-bit của thiết bị (đã dịch trái 1 bit)
 * @param reg Địa chỉ thanh ghi bắt đầu đọc
 * @param buf Con trỏ đến buffer để lưu dữ liệu
 * @param len Số byte cần đọc
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef I2C_Mem_Read(uint8_t dev_addr, uint8_t reg, uint8_t* buf, uint16_t len);

#endif /* INC_I2C_H_ */