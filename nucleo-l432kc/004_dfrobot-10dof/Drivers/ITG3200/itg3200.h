/*
 * itg3200.h - ITG-3200/I2C library for STM32
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
 *
 *
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* I2C address options */
#define ITG3200_ADDR_AD0_HIGH  0x69   // AD0=1 0x69 I2C address when AD0 is connected to HIGH (VCC)
#define ITG3200_ADDR_AD0_LOW   0x68   // AD0=0 0x68 I2C address when AD0 is connected to LOW (GND)
// "The LSB bit of the 7 bit address is determined by the logic level on pin 9.
// This allows two ITG-3200 devices to be connected to the same I2C bus.
// One device should have pin9 (or bit0) LOW and the other should be HIGH." source: ITG3200 datasheet
// Note that pin9 (AD0 - I2C Slave Address LSB) may not be available on some breakout boards so check
// the schematics of your breakout board for the correct address to use.

#define GYRO_START_UP_DELAY    70     // 50ms from gyro startup + 20ms register r/w startup

/* registers */
#define WHO_AM_I               0x00   // RW   SETUP: I2C address
#define SMPLRT_DIV             0x15   // RW   SETUP: Sample Rate Divider
#define DLPF_FS                0x16   // RW   SETUP: Digital Low Pass Filter/ Full Scale range
#define INT_CFG                0x17   // RW   Interrupt: Configuration
#define INT_STATUS             0x1A   // R    Interrupt: Status
#define TEMP_OUT               0x1B   // R    SENSOR: Temperature 2bytes
#define GYRO_XOUT              0x1D   // R    SENSOR: Gyro X 2bytes
#define GYRO_YOUT              0x1F   // R    SENSOR: Gyro Y 2bytes
#define GYRO_ZOUT              0x21   // R    SENSOR: Gyro Z 2bytes
#define PWR_MGM                0x3E   // RW   Power Management

/* bit maps */
#define DLPFFS_FS_SEL          0x18   // 00011000
#define DLPFFS_DLPF_CFG        0x07   // 00000111
#define INTCFG_ACTL            0x80   // 10000000
#define INTCFG_OPEN            0x40   // 01000000
#define INTCFG_LATCH_INT_EN    0x20   // 00100000
#define INTCFG_INT_ANYRD_2CLEAR 0x10  // 00010000
#define INTCFG_ITG_RDY_EN      0x04   // 00000100
#define INTCFG_RAW_RDY_EN      0x01   // 00000001
#define INTSTATUS_ITG_RDY      0x04   // 00000100
#define INTSTATUS_RAW_DATA_RDY 0x01   // 00000001
#define PWRMGM_HRESET          0x80   // 10000000
#define PWRMGM_SLEEP           0x40   // 01000000
#define PWRMGM_STBY_XG         0x20   // 00100000
#define PWRMGM_STBY_YG         0x10   // 00010000
#define PWRMGM_STBY_ZG         0x08   // 00001000
#define PWRMGM_CLK_SEL         0x07   // 00000111

/* register parameters */
// Sample Rate Divider
#define NO_SR_DIVIDER          0      // default FsampleHz=SampleRateHz/(divider+1)

// Gyro Full Scale Range
#define RANGE_2000             3      // default

// Digital Low Pass Filter BandWidth and SampleRate
#define BW256_SR8              0      // default 256Khz BW and 8Khz SR
#define BW188_SR1              1
#define BW098_SR1              2
#define BW042_SR1              3
#define BW020_SR1              4
#define BW010_SR1              5
#define BW005_SR1              6

// Interrupt Active logic level
#define ACTIVE_ON_HIGH         0      // default
#define ACTIVE_ON_LOW          1

// Interrupt drive type
#define PUSH_PULL              0      // default
#define OPEN_DRAIN             1

// Interrupt Latch mode
#define PULSE_50US             0      // default
#define UNTIL_INT_CLEARED      1

// Interrupt Latch clear method
#define READ_STATUS_REG        0      // default
#define READ_ANY_REG           1

// Power management
#define NORMAL                 0      // default
#define STANDBY                1

// Clock Source - user parameters
#define INTERNAL_OSC           0      // default
#define PLL_XGYRO_REF          1
#define PLL_YGYRO_REF          2
#define PLL_ZGYRO_REF          3
#define PLL_EXTERNAL_32        4      // 32.768 kHz
#define PLL_EXTERNAL_19        5      // 19.2 Mhz

/* device structure */
typedef struct {
	uint8_t dev_address;
	uint8_t buff[6];
	float gains[3];
	int32_t offsets[3];
	float polarities[3];
	void *i2c_handle;
} itg3200_t;

/* function prototypes */

// Initialization
void itg3200_init_default(itg3200_t *dev, void *i2c_handle, uint16_t address);
void itg3200_init(itg3200_t *dev, void *i2c_handle, uint16_t address,
                  uint8_t sample_rate_div, uint8_t range, uint8_t filter_bw,
                  uint8_t clock_src, bool itg_ready, bool int_raw_data_ready);

// Device Address
uint8_t itg3200_get_dev_addr(itg3200_t *dev);
void itg3200_set_dev_addr(itg3200_t *dev, uint16_t addr);

// Sample Rate Divider
uint8_t itg3200_get_sample_rate_div(itg3200_t *dev);
void itg3200_set_sample_rate_div(itg3200_t *dev, uint8_t sample_rate);

// Full Scale Range
uint8_t itg3200_get_fs_range(itg3200_t *dev);
void itg3200_set_fs_range(itg3200_t *dev, uint8_t range);

// Filter Bandwidth
uint8_t itg3200_get_filter_bw(itg3200_t *dev);
void itg3200_set_filter_bw(itg3200_t *dev, uint8_t bw);

// Interrupt Configuration
bool itg3200_is_int_active_on_low(itg3200_t *dev);
void itg3200_set_int_logic_lvl(itg3200_t *dev, bool state);
bool itg3200_is_int_open_drain(itg3200_t *dev);
void itg3200_set_int_drive_type(itg3200_t *dev, bool state);
bool itg3200_is_latch_until_cleared(itg3200_t *dev);
void itg3200_set_latch_mode(itg3200_t *dev, bool state);
bool itg3200_is_any_reg_clr_mode(itg3200_t *dev);
void itg3200_set_latch_clear_mode(itg3200_t *dev, bool state);

// Interrupt Enable
bool itg3200_is_itg_ready_on(itg3200_t *dev);
void itg3200_set_itg_ready(itg3200_t *dev, bool state);
bool itg3200_is_raw_data_ready_on(itg3200_t *dev);
void itg3200_set_raw_data_ready(itg3200_t *dev, bool state);

// Interrupt Status
bool itg3200_is_itg_ready(itg3200_t *dev);
bool itg3200_is_raw_data_ready(itg3200_t *dev);

// Sensor Reading
void itg3200_read_temp(itg3200_t *dev, float *temp);
void itg3200_read_gyro_raw(itg3200_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void itg3200_read_gyro_raw_xyz(itg3200_t *dev, int16_t *gyro_xyz);

// Calibration
void itg3200_set_rev_polarity(itg3200_t *dev, bool x_pol, bool y_pol, bool z_pol);
void itg3200_set_gains(itg3200_t *dev, float x_gain, float y_gain, float z_gain);
void itg3200_set_offsets(itg3200_t *dev, int32_t x_offset, int32_t y_offset, int32_t z_offset);
void itg3200_zero_calibrate(itg3200_t *dev, uint32_t tot_samples, uint32_t sample_delay_ms);

// Calibrated Reading
void itg3200_read_gyro_raw_cal(itg3200_t *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void itg3200_read_gyro_raw_cal_xyz(itg3200_t *dev, int16_t *gyro_xyz);
void itg3200_read_gyro(itg3200_t *dev, float *gyro_x, float *gyro_y, float *gyro_z);
void itg3200_read_gyro_xyz(itg3200_t *dev, float *gyro_xyz);

// Power Management
void itg3200_reset(itg3200_t *dev);
bool itg3200_is_low_power(itg3200_t *dev);
void itg3200_set_power_mode(itg3200_t *dev, bool state);
bool itg3200_is_x_gyro_standby(itg3200_t *dev);
bool itg3200_is_y_gyro_standby(itg3200_t *dev);
bool itg3200_is_z_gyro_standby(itg3200_t *dev);
void itg3200_set_x_gyro_standby(itg3200_t *dev, bool status);
void itg3200_set_y_gyro_standby(itg3200_t *dev, bool status);
void itg3200_set_z_gyro_standby(itg3200_t *dev, bool status);
uint8_t itg3200_get_clock_source(itg3200_t *dev);
void itg3200_set_clock_source(itg3200_t *dev, uint8_t clk_source);

// Low-level I2C
void itg3200_write_mem(itg3200_t *dev, uint8_t addr, uint8_t val);
void itg3200_read_mem(itg3200_t *dev, uint8_t addr, uint8_t nbytes, uint8_t *buffer);

// HAL delay function (to be implemented by user)
void itg3200_delay_ms(uint32_t ms);


