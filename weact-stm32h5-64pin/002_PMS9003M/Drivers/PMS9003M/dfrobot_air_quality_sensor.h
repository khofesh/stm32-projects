/*!
 * @file dfrobot_air_quality_sensor.h
 * @brief STM32 HAL version of DFRobot Air Quality Sensor library
 * @note  Converted from Arduino library for STM32H562RGTX using I2C3
 */

#pragma once

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// PM concentration register addresses (µg/m³)
#define PARTICLE_PM1_0_STANDARD   0x05
#define PARTICLE_PM2_5_STANDARD   0x07
#define PARTICLE_PM10_STANDARD    0x09
#define PARTICLE_PM1_0_ATMOSPHERE 0x0B
#define PARTICLE_PM2_5_ATMOSPHERE 0x0D
#define PARTICLE_PM10_ATMOSPHERE  0x0F

// particle count register addresses (particles per 0.1L air)
#define PARTICLENUM_0_3_UM_EVERY0_1L_AIR 0x11
#define PARTICLENUM_0_5_UM_EVERY0_1L_AIR 0x13
#define PARTICLENUM_1_0_UM_EVERY0_1L_AIR 0x15
#define PARTICLENUM_2_5_UM_EVERY0_1L_AIR 0x17
#define PARTICLENUM_5_0_UM_EVERY0_1L_AIR 0x19
#define PARTICLENUM_10_UM_EVERY0_1L_AIR  0x1B

// version register
#define PARTICLENUM_GAIN_VERSION 0x1D

// power mode register
#define POWER_MODE_REG 0x01
#define LOW_POWER_MODE 1
#define AWAKE_MODE     2

// i2c address
#define DFROBOT_AIR_SENSOR_DEFAULT_ADDR 0x19

// i2c timeout (ms)
#define I2C_TIMEOUT 1000

// pms9003m struct
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
} DFRobot_AirQualitySensor_t;

bool DFRobot_AirSensor_Init(DFRobot_AirQualitySensor_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t addr);
bool DFRobot_AirSensor_Begin(DFRobot_AirQualitySensor_t *sensor);
uint16_t DFRobot_AirSensor_GetParticleConcentration_ugm3(DFRobot_AirQualitySensor_t *sensor, uint8_t type);
uint16_t DFRobot_AirSensor_GetParticleNum_Every0_1L(DFRobot_AirQualitySensor_t *sensor, uint8_t type);
uint8_t DFRobot_AirSensor_GetVersion(DFRobot_AirQualitySensor_t *sensor);
bool DFRobot_AirSensor_SetLowPower(DFRobot_AirQualitySensor_t *sensor);
bool DFRobot_AirSensor_Awake(DFRobot_AirQualitySensor_t *sensor);

HAL_StatusTypeDef DFRobot_AirSensor_WriteReg(DFRobot_AirQualitySensor_t *sensor, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef DFRobot_AirSensor_ReadReg(DFRobot_AirQualitySensor_t *sensor, uint8_t reg, uint8_t *data, uint8_t len);
