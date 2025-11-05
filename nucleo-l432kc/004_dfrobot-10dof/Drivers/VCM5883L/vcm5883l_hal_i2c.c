/*
 * vcm5883l_hal_i2c.c
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#include "vcm5883l_hal_i2c.h"

bool vcm5883l_i2c_is_device_ready(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr)
{
    return (HAL_I2C_IsDeviceReady(hi2c, dev_addr, 5, VCM5883L_I2C_TIMEOUT) == HAL_OK);
}

bool vcm5883l_i2c_write(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                       uint8_t *data, uint16_t size)
{
    return (HAL_I2C_Master_Transmit(hi2c, dev_addr, data, size, VCM5883L_I2C_TIMEOUT) == HAL_OK);
}

bool vcm5883l_i2c_read(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                      uint8_t *data, uint16_t size)
{
    return (HAL_I2C_Master_Receive(hi2c, dev_addr, data, size, VCM5883L_I2C_TIMEOUT) == HAL_OK);
}

bool vcm5883l_i2c_mem_read(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                          uint8_t reg_addr, uint8_t *data, uint16_t size)
{
    return (HAL_I2C_Mem_Read(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                             data, size, VCM5883L_I2C_TIMEOUT) == HAL_OK);
}

bool vcm5883l_i2c_mem_write(vcm5883l_i2c_handle_t *hi2c, uint8_t dev_addr,
                                           uint8_t reg_addr, uint8_t *data, uint16_t size)
{
    return (HAL_I2C_Mem_Write(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                              data, size, VCM5883L_I2C_TIMEOUT) == HAL_OK);
}
