/*!
 * @file dfrobot_air_quality_sensor.h
 * @brief STM32 HAL version of DFRobot Air Quality Sensor library
 * @note  Converted from Arduino library for STM32H562RGTX using I2C3
 */

#include "dfrobot_air_quality_sensor.h"

bool DFRobot_AirSensor_Init(DFRobot_AirQualitySensor_t *sensor, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
	if (sensor == NULL || hi2c == NULL)
	{
		return false;
	}

	sensor->hi2c = hi2c;
	// convert to 8-bit address format - typical stm32
	sensor->i2c_addr = addr << 1;

	return true;
}

bool DFRobot_AirSensor_Begin(DFRobot_AirQualitySensor_t *sensor)
{
	if (sensor == NULL)
	{
		return false;
	}

	// is device ready ?
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(sensor->hi2c, sensor->i2c_addr, 3, I2C_TIMEOUT);

	return (status == HAL_OK);
}

uint16_t DFRobot_AirSensor_GetParticleConcentration_ugm3(DFRobot_AirQualitySensor_t *sensor, uint8_t type)
{
	if (sensor == NULL)
	{
		return 0;
	}

	uint8_t buf[2];
	HAL_StatusTypeDef status = DFRobot_AirSensor_ReadReg(sensor, type, buf, 2);

	if (status != HAL_OK)
	{
		return 0;
	}

	// combine high and low bytes (big-endian)
	uint16_t concentration = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];

	return concentration;
}

uint16_t DFRobot_AirSensor_GetParticleNum_Every0_1L(DFRobot_AirQualitySensor_t *sensor, uint8_t type)
{
    if (sensor == NULL)
    {
        return 0;
    }

    uint8_t buf[2];
    HAL_StatusTypeDef status = DFRobot_AirSensor_ReadReg(sensor, type, buf, 2);

    if (status != HAL_OK)
    {
        return 0;
    }

    // combine high and low bytes (big-endian)
    uint16_t particle_num = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];

    return particle_num;
}

uint8_t DFRobot_AirSensor_GetVersion(DFRobot_AirQualitySensor_t *sensor)
{
    if (sensor == NULL)
    {
        return 0;
    }

    uint8_t version = 0;
    HAL_StatusTypeDef status = DFRobot_AirSensor_ReadReg(sensor, PARTICLENUM_GAIN_VERSION, &version, 1);

    if (status != HAL_OK)
    {
        return 0;
    }

    return version;
}

bool DFRobot_AirSensor_SetLowPower(DFRobot_AirQualitySensor_t *sensor)
{
    if (sensor == NULL)
    {
        return false;
    }

    uint8_t mode = LOW_POWER_MODE;
    HAL_StatusTypeDef status = DFRobot_AirSensor_WriteReg(sensor, POWER_MODE_REG, &mode, 1);

    return (status == HAL_OK);
}

bool DFRobot_AirSensor_Awake(DFRobot_AirQualitySensor_t *sensor)
{
    if (sensor == NULL)
    {
        return false;
    }

    uint8_t mode = AWAKE_MODE;
    HAL_StatusTypeDef status = DFRobot_AirSensor_WriteReg(sensor, POWER_MODE_REG, &mode, 1);

    return (status == HAL_OK);
}

HAL_StatusTypeDef DFRobot_AirSensor_WriteReg(DFRobot_AirQualitySensor_t *sensor, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (sensor == NULL)
    {
        return HAL_ERROR;
    }

    // create buffer with register address followed by data
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg;

    for (uint8_t i = 0; i < len; i++)
    {
        tx_buf[i + 1] = data[i];
    }

    return HAL_I2C_Master_Transmit(sensor->hi2c, sensor->i2c_addr, tx_buf, len + 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef DFRobot_AirSensor_ReadReg(DFRobot_AirQualitySensor_t *sensor, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (sensor == NULL)
    {
        return HAL_ERROR;
    }

    // first write the register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(sensor->hi2c, sensor->i2c_addr, &reg, 1, I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return status;
    }

    // then read the data
    return HAL_I2C_Master_Receive(sensor->hi2c, sensor->i2c_addr, data, len, I2C_TIMEOUT);
}

