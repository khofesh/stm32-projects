/*
 * itg3200.c - ITG-3200/I2C library for STM32
 * Converted from Arduino library v0.5
 * Original Copyright 2010-2011 Filipe Vieira & various contributors
 * http://code.google.com/p/itg-3200driver
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#include "itg3200.h"
#include <string.h>

#include "stm32l4xx_hal.h"

#define ITG3200_I2C_TIMEOUT 100


__weak void itg3200_delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP();
    }
}

/**
 * @brief Initialize ITG3200 with default settings
 */
void itg3200_init_default(itg3200_t *dev, void *i2c_handle, uint16_t address) {
    dev->i2c_handle = i2c_handle;
    dev->dev_address = address;

    itg3200_set_gains(dev, 1.0f, 1.0f, 1.0f);
    itg3200_set_offsets(dev, 0, 0, 0);
    itg3200_set_rev_polarity(dev, false, false, false);

    // Fast sample rate - divisor = 0 filter = 0 clocksrc = PLL_XGYRO_REF
    itg3200_init(dev, i2c_handle, address, NO_SR_DIVIDER, RANGE_2000,
                 BW256_SR8, PLL_XGYRO_REF, true, true);
}

/**
 * @brief Initialize ITG3200 with custom settings
 */
void itg3200_init(itg3200_t *dev, void *i2c_handle, uint16_t address,
                  uint8_t sample_rate_div, uint8_t range, uint8_t filter_bw,
                  uint8_t clock_src, bool itg_ready, bool int_raw_data_ready) {
    dev->i2c_handle = i2c_handle;
    dev->dev_address = address;

    itg3200_set_sample_rate_div(dev, sample_rate_div);
    itg3200_set_fs_range(dev, range);
    itg3200_set_filter_bw(dev, filter_bw);
    itg3200_set_clock_source(dev, clock_src);
    itg3200_set_itg_ready(dev, itg_ready);
    itg3200_set_raw_data_ready(dev, int_raw_data_ready);

    itg3200_delay_ms(GYRO_START_UP_DELAY);
}

/**
 * @brief Get device address
 */
uint8_t itg3200_get_dev_addr(itg3200_t *dev) {
    return dev->dev_address;
}

/**
 * @brief Set device address
 */
void itg3200_set_dev_addr(itg3200_t *dev, uint16_t addr) {
    itg3200_write_mem(dev, WHO_AM_I, addr);
    dev->dev_address = addr;
}

/**
 * @brief Get sample rate divider
 */
uint8_t itg3200_get_sample_rate_div(itg3200_t *dev) {
    itg3200_read_mem(dev, SMPLRT_DIV, 1, &dev->buff[0]);
    return dev->buff[0];
}

/**
 * @brief Set sample rate divider
 */
void itg3200_set_sample_rate_div(itg3200_t *dev, uint8_t sample_rate) {
    itg3200_write_mem(dev, SMPLRT_DIV, sample_rate);
}

/**
 * @brief Get full scale range
 */
uint8_t itg3200_get_fs_range(itg3200_t *dev) {
    itg3200_read_mem(dev, DLPF_FS, 1, &dev->buff[0]);
    return ((dev->buff[0] & DLPFFS_FS_SEL) >> 3);
}

/**
 * @brief Set full scale range
 */
void itg3200_set_fs_range(itg3200_t *dev, uint8_t range) {
    itg3200_read_mem(dev, DLPF_FS, 1, &dev->buff[0]);
    itg3200_write_mem(dev, DLPF_FS, ((dev->buff[0] & ~DLPFFS_FS_SEL) | (range << 3)));
}

/**
 * @brief Get filter bandwidth
 */
uint8_t itg3200_get_filter_bw(itg3200_t *dev) {
    itg3200_read_mem(dev, DLPF_FS, 1, &dev->buff[0]);
    return (dev->buff[0] & DLPFFS_DLPF_CFG);
}

/**
 * @brief Set filter bandwidth
 */
void itg3200_set_filter_bw(itg3200_t *dev, uint8_t bw) {
    itg3200_read_mem(dev, DLPF_FS, 1, &dev->buff[0]);
    itg3200_write_mem(dev, DLPF_FS, ((dev->buff[0] & ~DLPFFS_DLPF_CFG) | bw));
}

/**
 * @brief Check if interrupt is active on low
 */
bool itg3200_is_int_active_on_low(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTCFG_ACTL) >> 7);
}

/**
 * @brief Set interrupt logic level
 */
void itg3200_set_int_logic_lvl(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_ACTL) | (state << 7)));
}

/**
 * @brief Check if interrupt is open drain
 */
bool itg3200_is_int_open_drain(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTCFG_OPEN) >> 6);
}

/**
 * @brief Set interrupt drive type
 */
void itg3200_set_int_drive_type(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_OPEN) | (state << 6)));
}

/**
 * @brief Check if latch until cleared
 */
bool itg3200_is_latch_until_cleared(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

/**
 * @brief Set latch mode
 */
void itg3200_set_latch_mode(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_LATCH_INT_EN) | (state << 5)));
}

/**
 * @brief Check if any register clear mode
 */
bool itg3200_is_any_reg_clr_mode(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

/**
 * @brief Set latch clear mode
 */
void itg3200_set_latch_clear_mode(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | (state << 4)));
}

/**
 * @brief Check if ITG ready interrupt is enabled
 */
bool itg3200_is_itg_ready_on(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

/**
 * @brief Set ITG ready interrupt
 */
void itg3200_set_itg_ready(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_ITG_RDY_EN) | (state << 2)));
}

/**
 * @brief Check if raw data ready interrupt is enabled
 */
bool itg3200_is_raw_data_ready_on(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    return (dev->buff[0] & INTCFG_RAW_RDY_EN);
}

/**
 * @brief Set raw data ready interrupt
 */
void itg3200_set_raw_data_ready(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, INT_CFG, 1, &dev->buff[0]);
    itg3200_write_mem(dev, INT_CFG, ((dev->buff[0] & ~INTCFG_RAW_RDY_EN) | state));
}

/**
 * @brief Check if ITG is ready
 */
bool itg3200_is_itg_ready(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_STATUS, 1, &dev->buff[0]);
    return ((dev->buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

/**
 * @brief Check if raw data is ready
 */
bool itg3200_is_raw_data_ready(itg3200_t *dev) {
    itg3200_read_mem(dev, INT_STATUS, 1, &dev->buff[0]);
    return (dev->buff[0] & INTSTATUS_RAW_DATA_RDY);
}

/**
 * @brief Read temperature
 */
void itg3200_read_temp(itg3200_t *dev, float *temp) {
    itg3200_read_mem(dev, TEMP_OUT, 2, dev->buff);
    *temp = 35.0f + (((int16_t)((dev->buff[0] << 8) | dev->buff[1]) + 13200) / 280.0f);
}

/**
 * @brief Read raw gyroscope data
 */
void itg3200_read_gyro_raw(itg3200_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    itg3200_read_mem(dev, GYRO_XOUT, 6, dev->buff);
    *gyro_x = (int16_t)((dev->buff[0] << 8) | dev->buff[1]);
    *gyro_y = (int16_t)((dev->buff[2] << 8) | dev->buff[3]);
    *gyro_z = (int16_t)((dev->buff[4] << 8) | dev->buff[5]);
}

/**
 * @brief Read raw gyroscope data into array
 */
void itg3200_read_gyro_raw_xyz(itg3200_t *dev, int16_t *gyro_xyz) {
    itg3200_read_gyro_raw(dev, &gyro_xyz[0], &gyro_xyz[1], &gyro_xyz[2]);
}

/**
 * @brief Set reverse polarity
 */
void itg3200_set_rev_polarity(itg3200_t *dev, bool x_pol, bool y_pol, bool z_pol) {
    dev->polarities[0] = x_pol ? -1.0f : 1.0f;
    dev->polarities[1] = y_pol ? -1.0f : 1.0f;
    dev->polarities[2] = z_pol ? -1.0f : 1.0f;
}

/**
 * @brief Set gains for each axis
 */
void itg3200_set_gains(itg3200_t *dev, float x_gain, float y_gain, float z_gain) {
    dev->gains[0] = x_gain;
    dev->gains[1] = y_gain;
    dev->gains[2] = z_gain;
}

/**
 * @brief Set offsets for each axis
 */
void itg3200_set_offsets(itg3200_t *dev, int32_t x_offset, int32_t y_offset, int32_t z_offset) {
    dev->offsets[0] = x_offset;
    dev->offsets[1] = y_offset;
    dev->offsets[2] = z_offset;
}

/**
 * @brief Zero calibrate the gyroscope
 * @note Assumes gyroscope is stationary during calibration
 */
void itg3200_zero_calibrate(itg3200_t *dev, uint32_t tot_samples, uint32_t sample_delay_ms) {
    int16_t xyz[3];
    float tmp_offsets[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t i = 0; i < tot_samples; i++) {
        itg3200_delay_ms(sample_delay_ms);
        itg3200_read_gyro_raw_xyz(dev, xyz);
        tmp_offsets[0] += xyz[0];
        tmp_offsets[1] += xyz[1];
        tmp_offsets[2] += xyz[2];
    }

    itg3200_set_offsets(dev,
                        -(int32_t)(tmp_offsets[0] / tot_samples),
                        -(int32_t)(tmp_offsets[1] / tot_samples),
                        -(int32_t)(tmp_offsets[2] / tot_samples));
}

/**
 * @brief Read calibrated raw gyroscope data
 */
void itg3200_read_gyro_raw_cal(itg3200_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    itg3200_read_gyro_raw(dev, gyro_x, gyro_y, gyro_z);
    *gyro_x += dev->offsets[0];
    *gyro_y += dev->offsets[1];
    *gyro_z += dev->offsets[2];
}

/**
 * @brief Read calibrated raw gyroscope data into array
 */
void itg3200_read_gyro_raw_cal_xyz(itg3200_t *dev, int16_t *gyro_xyz) {
    itg3200_read_gyro_raw_cal(dev, &gyro_xyz[0], &gyro_xyz[1], &gyro_xyz[2]);
}

/**
 * @brief Read gyroscope data with gains and offsets applied (in deg/s)
 */
void itg3200_read_gyro(itg3200_t *dev, float *gyro_x, float *gyro_y, float *gyro_z) {
    int16_t x, y, z;

    itg3200_read_gyro_raw_cal(dev, &x, &y, &z);
    *gyro_x = x / 14.375f * dev->polarities[0] * dev->gains[0];
    *gyro_y = y / 14.375f * dev->polarities[1] * dev->gains[1];
    *gyro_z = z / 14.375f * dev->polarities[2] * dev->gains[2];
}

/**
 * @brief Read gyroscope data into array (in deg/s)
 */
void itg3200_read_gyro_xyz(itg3200_t *dev, float *gyro_xyz) {
    itg3200_read_gyro(dev, &gyro_xyz[0], &gyro_xyz[1], &gyro_xyz[2]);
}

/**
 * @brief Reset the device
 */
void itg3200_reset(itg3200_t *dev) {
    itg3200_write_mem(dev, PWR_MGM, PWRMGM_HRESET);
    itg3200_delay_ms(GYRO_START_UP_DELAY);
}

/**
 * @brief Check if device is in low power mode
 */
bool itg3200_is_low_power(itg3200_t *dev) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    return ((dev->buff[0] & PWRMGM_SLEEP) >> 6);
}

/**
 * @brief Set power mode
 */
void itg3200_set_power_mode(itg3200_t *dev, bool state) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    itg3200_write_mem(dev, PWR_MGM, ((dev->buff[0] & ~PWRMGM_SLEEP) | (state << 6)));
}

/**
 * @brief Check if X gyro is in standby
 */
bool itg3200_is_x_gyro_standby(itg3200_t *dev) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    return ((dev->buff[0] & PWRMGM_STBY_XG) >> 5);
}

/**
 * @brief Check if Y gyro is in standby
 */
bool itg3200_is_y_gyro_standby(itg3200_t *dev) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    return ((dev->buff[0] & PWRMGM_STBY_YG) >> 4);
}

/**
 * @brief Check if Z gyro is in standby
 */
bool itg3200_is_z_gyro_standby(itg3200_t *dev) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    return ((dev->buff[0] & PWRMGM_STBY_ZG) >> 3);
}

/**
 * @brief Set X gyro standby mode
 */
void itg3200_set_x_gyro_standby(itg3200_t *dev, bool status) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    itg3200_write_mem(dev, PWR_MGM, ((dev->buff[0] & ~PWRMGM_STBY_XG) | (status << 5)));
}

/**
 * @brief Set Y gyro standby mode
 */
void itg3200_set_y_gyro_standby(itg3200_t *dev, bool status) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    itg3200_write_mem(dev, PWR_MGM, ((dev->buff[0] & ~PWRMGM_STBY_YG) | (status << 4)));
}

/**
 * @brief Set Z gyro standby mode
 */
void itg3200_set_z_gyro_standby(itg3200_t *dev, bool status) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    itg3200_write_mem(dev, PWR_MGM, ((dev->buff[0] & ~PWRMGM_STBY_ZG) | (status << 3)));
}

/**
 * @brief Get clock source
 */
uint8_t itg3200_get_clock_source(itg3200_t *dev) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    return (dev->buff[0] & PWRMGM_CLK_SEL);
}

/**
 * @brief Set clock source
 */
void itg3200_set_clock_source(itg3200_t *dev, uint8_t clk_source) {
    itg3200_read_mem(dev, PWR_MGM, 1, &dev->buff[0]);
    itg3200_write_mem(dev, PWR_MGM, ((dev->buff[0] & ~PWRMGM_CLK_SEL) | clk_source));
}

/**
 * @brief Write to device memory (register)
 */
void itg3200_write_mem(itg3200_t *dev, uint8_t addr, uint8_t val) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)dev->i2c_handle;

    HAL_I2C_Mem_Write(hi2c,
                      dev->dev_address << 1,  // Device address (shifted for HAL)
                      addr,                    // Register address
                      I2C_MEMADD_SIZE_8BIT,   // Register address size
                      &val,                    // Data to write
                      1,                       // Number of bytes
                      ITG3200_I2C_TIMEOUT);   // Timeout
}

/**
 * @brief Read from device memory (register)
 */
void itg3200_read_mem(itg3200_t *dev, uint8_t addr, uint8_t nbytes, uint8_t *buffer) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)dev->i2c_handle;

    HAL_I2C_Mem_Read(hi2c,
                     dev->dev_address << 1,  // Device address (shifted for HAL)
                     addr,                    // Register address
                     I2C_MEMADD_SIZE_8BIT,   // Register address size
                     buffer,                  // Buffer to store data
                     nbytes,                  // Number of bytes to read
                     ITG3200_I2C_TIMEOUT);   // Timeout
}
