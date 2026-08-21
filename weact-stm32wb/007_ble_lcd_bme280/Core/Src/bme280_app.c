/**
 ******************************************************************************
 * @file    bme280_app.c
 * @brief   BME280 acquisition on I2C1, non blocking forced mode sampling
 ******************************************************************************
 */

#include "bme280_app.h"
#include "bme280.h"
#include "main.h"

/* the BME280 sits on I2C1 (PB8 = SCL, PB9 = SDA), see README.md */
extern I2C_HandleTypeDef hi2c1;

#define BME280_I2C_TIMEOUT      100U

static struct bme280_dev      s_dev;
static struct bme280_settings s_settings;
static uint8_t                s_addr = BME280_I2C_ADDR_PRIM;
static bme280_app_data_t      s_data;
static uint8_t                s_have_data;
static uint8_t                s_inited;
static uint32_t               s_meas_delay_ms;
static uint32_t               s_err_count;

static BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;

    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), (uint16_t)reg_addr,
                         I2C_MEMADD_SIZE_8BIT, reg_data, (uint16_t)len,
                         BME280_I2C_TIMEOUT) != HAL_OK)
    {
        return BME280_E_COMM_FAIL;
    }

    return BME280_INTF_RET_SUCCESS;
}

static BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;

    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), (uint16_t)reg_addr,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, (uint16_t)len,
                          BME280_I2C_TIMEOUT) != HAL_OK)
    {
        return BME280_E_COMM_FAIL;
    }

    return BME280_INTF_RET_SUCCESS;
}

static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    HAL_Delay((period + 999U) / 1000U);
}

uint8_t BME280_APP_Init(void)
{
    uint32_t delay_us;

    s_dev.intf     = BME280_I2C_INTF;
    s_dev.intf_ptr = &s_addr;
    s_dev.read     = bme280_i2c_read;
    s_dev.write    = bme280_i2c_write;
    s_dev.delay_us = bme280_delay_us;

    if (bme280_init(&s_dev) != BME280_OK)
    {
        /* retry on the secondary address, the module strap may pull SDO high */
        s_addr = BME280_I2C_ADDR_SEC;
        if (bme280_init(&s_dev) != BME280_OK)
        {
            return 1;
        }
    }

    /* indoor navigation-ish profile, forced mode so the part idles between reads */
    s_settings.osr_h        = BME280_OVERSAMPLING_1X;
    s_settings.osr_t        = BME280_OVERSAMPLING_2X;
    s_settings.osr_p        = BME280_OVERSAMPLING_16X;
    s_settings.filter       = BME280_FILTER_COEFF_16;
    s_settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &s_settings, &s_dev) != BME280_OK)
    {
        return 1;
    }

    if (bme280_cal_meas_delay(&delay_us, &s_settings) != BME280_OK)
    {
        return 1;
    }
    /* round up, and add one millisecond of margin for the internal oscillator
       tolerance so the read never races the end of the conversion */
    s_meas_delay_ms = ((delay_us + 999U) / 1000U) + 1U;

    s_have_data = 0;
    s_err_count = 0;
    s_inited    = 1;

    return 0;
}

uint8_t BME280_APP_StartMeasurement(void)
{
    /* without this a failed init would retry on every scheduled cycle, each
       attempt burning two 100 ms HAL I2C timeouts and stalling the sequencer */
    if (s_inited == 0)
    {
        return 1;
    }

    if (bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &s_dev) != BME280_OK)
    {
        s_err_count++;
        return 1;
    }

    return 0;
}

uint32_t BME280_APP_GetMeasurementDelayMs(void)
{
    return s_meas_delay_ms;
}

uint8_t BME280_APP_ReadMeasurement(bme280_app_data_t *out)
{
    struct bme280_data raw;

    if (s_inited == 0)
    {
        return 1;
    }

    if (bme280_get_sensor_data(BME280_ALL, &raw, &s_dev) != BME280_OK)
    {
        s_err_count++;
        return 1;
    }

    s_data.temperature = raw.temperature;
    s_data.pressure    = raw.pressure;
    s_data.humidity    = raw.humidity;
    s_have_data        = 1;
    s_err_count        = 0;

    if (out != NULL)
    {
        *out = s_data;
    }

    return 0;
}

uint32_t BME280_APP_GetErrorCount(void)
{
    return s_err_count;
}

uint8_t BME280_APP_GetData(bme280_app_data_t *out)
{
    if ((out == NULL) || (s_have_data == 0))
    {
        return 1;
    }

    *out = s_data;

    return 0;
}
