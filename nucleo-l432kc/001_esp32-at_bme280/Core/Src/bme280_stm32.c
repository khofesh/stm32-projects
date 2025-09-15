/*
 * bme280_stm32.c
 *
 *  Created on: Jun 17, 2025
 *      Author: fahmad
 */


#include "bme280_stm32.h"

#define BME280_I2C BME280_I2C
#define BME280_TIM BME280_TIM

// i2c handle
extern I2C_HandleTypeDef BME280_I2C;

// TIM6
extern TIM_HandleTypeDef BME280_TIM;

// BME280 i2c address
//The default for the SparkFun Environmental Combo board is 0x77 (jumper open).
//If you close the jumper it is 0x76
// https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/blob/master/examples/Example2_I2CAddress/Example2_I2CAddress.ino
#define BME280_I2C_ADDR_SECONDARY (0x76 << 1)
#define BME280_I2C_ADDR_PRIMARY (0x77 << 1)

#define BME280_I2C_TIMEOUT 1000

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    if (HAL_I2C_Mem_Read(&BME280_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                         reg_data, len, BME280_I2C_TIMEOUT) == HAL_OK)
    {
        return BME280_OK;
    }
    else
    {
        return BME280_E_COMM_FAIL;
    }
}


int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    if (HAL_I2C_Mem_Write(&BME280_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)reg_data, len, BME280_I2C_TIMEOUT) == HAL_OK)
    {
        return BME280_OK;
    }
    else
    {
        return BME280_E_COMM_FAIL;
    }
}

static void delay_us_internal(uint32_t us) {
    // start timer if not already running
    if (!(BME280_TIM.Instance->CR1 & TIM_CR1_CEN)) {
        HAL_TIM_Base_Start(&BME280_TIM);
    }

    __HAL_TIM_SET_COUNTER(&BME280_TIM, 0);
    while(__HAL_TIM_GET_COUNTER(&BME280_TIM) < us);
}

// can't use TIM6 lol
void user_delay_us(uint32_t period, void *intf_ptr)
{
	delay_us_internal(period);
}

int8_t bme280_interface_init(struct bme280_dev *dev, uint8_t intf)
{
    int8_t rslt = BME280_OK;
    static uint8_t dev_addr;

    if (dev != NULL)
    {
        if (intf == BME280_I2C_INTF)
        {
            dev_addr = BME280_I2C_ADDR_PRIMARY;
            dev->read = user_i2c_read;
            dev->write = user_i2c_write;
            dev->intf = BME280_I2C_INTF;
        }

        dev->delay_us = user_delay_us;
        dev->intf_ptr = &dev_addr;
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

int8_t bme280_init_sensor(struct bme280_dev *dev)
{
    int8_t rslt;

    struct bme280_settings settings;

    // Initialize the interface
    rslt = bme280_interface_init(dev, BME280_I2C_INTF);

    if (rslt == BME280_OK)
    {
        // Initialize the sensor
        rslt = bme280_init(dev);

        if (rslt == BME280_OK)
        {
            /* Configuring the over-sampling rate, filter coefficient and standby time */
            /* Overwrite the desired settings */
            settings.filter = BME280_FILTER_COEFF_2;

            /* Over-sampling rate for humidity, temperature and pressure */
            settings.osr_h = BME280_OVERSAMPLING_1X;
            settings.osr_p = BME280_OVERSAMPLING_1X;
            settings.osr_t = BME280_OVERSAMPLING_1X;

            /* Setting the standby time */
            settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

            rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev);

            if (rslt == BME280_OK)
            {
                rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
            }
        }
    }

    return rslt;
}

int8_t bme280_read_sensor_data(struct bme280_data *comp_data, struct bme280_dev *dev)
{
    int8_t rslt;

    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, dev);

    return rslt;
}

