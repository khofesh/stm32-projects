/*
 * vcm5883l.h
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "vcm5883l_hal_i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* i2c address */
#define VCM5883L_ADDRESS             (0x0C << 1)  // Left shifted for HAL

/* register addresses */
#define QMC5883_REG_OUT_X_M          (0x01)
#define QMC5883_REG_OUT_X_L          (0x00)
#define QMC5883_REG_OUT_Z_M          (0x05)
#define QMC5883_REG_OUT_Z_L          (0x04)
#define QMC5883_REG_OUT_Y_M          (0x03)
#define QMC5883_REG_OUT_Y_L          (0x02)
#define QMC5883_REG_STATUS           (0x06)
#define QMC5883_REG_CONFIG_1         (0x09)
#define QMC5883_REG_CONFIG_2         (0x0A)
#define QMC5883_REG_IDENT_B          (0x0B)
#define QMC5883_REG_IDENT_C          (0x20)
#define QMC5883_REG_IDENT_D          (0x21)

#define VCM5883L_REG_OUT_X_L          0x00
#define VCM5883L_REG_OUT_X_H          0x01
#define VCM5883L_REG_OUT_Y_L          0x02
#define VCM5883L_REG_OUT_Y_H          0x03
#define VCM5883L_REG_OUT_Z_L          0x04
#define VCM5883L_REG_OUT_Z_H          0x05
#define VCM5883L_CTR_REG1             0x0B
#define VCM5883L_CTR_REG2             0x0A

/* sampling rate options */
typedef enum {
    VCM5883L_SAMPLES_1 = 0b00,
    VCM5883L_SAMPLES_2 = 0b01,
    VCM5883L_SAMPLES_4 = 0b10,
    VCM5883L_SAMPLES_8 = 0b11
} vcm5883l_samples_t;

/* data rate options (ODR - Output Data Rate) */
typedef enum {
    VCM5883L_DATARATE_200HZ = 0b00,
    VCM5883L_DATARATE_100HZ = 0b01,
    VCM5883L_DATARATE_50HZ  = 0b10,
    VCM5883L_DATARATE_10HZ  = 0b11
} vcm5883l_data_rate_t;

/* measurement range option */
typedef enum {
	VCM5883L_RANGE_8GA = 0b01  // Default 8 Gauss range
} vcm5883l_range_t;

/* measurement mode options */
typedef enum {
    VCM5883L_SINGLE    = 0b0,  // Single measurement mode
    VCM5883L_CONTINOUS = 0b1   // Continuous measurement mode
} vcm5883l_mode_t;

/* vector data structure for magnetometer readings */
typedef struct {
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
	float angle_xy;
	float angle_xz;
	float angle_yz;
	float heading_degrees;
} vcm5883l_vector_t;

typedef struct {
	vcm5883l_i2c_handle_t *hi2c;
    uint8_t i2c_address;
    float declination_angle;
    float mg_per_digit;
    vcm5883l_vector_t vector;
} vcm5883l_handle_t;

/* function prototypes */

/**
 * @brief Initialize VCM5883L sensor
 * @param handle Pointer to device handle
 * @param hi2c Pointer to I2C handle
 * @return true if initialization succeeded, false otherwise
 */
bool vcm5883l_init(vcm5883l_handle_t *handle, vcm5883l_i2c_handle_t *hi2c);

/**
 * @brief Read raw magnetometer data
 * @param handle Pointer to device handle
 * @return Vector structure containing raw data and calculated angles
 */
vcm5883l_vector_t vcm5883l_read_raw(vcm5883l_handle_t *handle);

/**
 * @brief Set measurement range
 * @param handle Pointer to device handle
 * @param range Range setting (VCM5883L only supports 8GA)
 */
void vcm5883l_set_range(vcm5883l_handle_t *handle, vcm5883l_range_t range);

/**
 * @brief Get current measurement range
 * @param handle Pointer to device handle
 * @return Current range setting
 */
vcm5883l_range_t vcm5883l_get_range(vcm5883l_handle_t *handle);

/**
 * @brief Set measurement mode
 * @param handle Pointer to device handle
 * @param mode Measurement mode (single or continuous)
 */
void vcm5883l_set_measurement_mode(vcm5883l_handle_t *handle, vcm5883l_mode_t mode);

/**
 * @brief Get current measurement mode
 * @param handle Pointer to device handle
 * @return Current measurement mode
 */
vcm5883l_mode_t vcm5883l_get_measurement_mode(vcm5883l_handle_t *handle);

/**
 * @brief Set data output rate
 * @param handle Pointer to device handle
 * @param data_rate Data rate setting
 */
void vcm5883l_set_data_rate(vcm5883l_handle_t *handle, vcm5883l_data_rate_t data_rate);

/**
 * @brief Get current data output rate
 * @param handle Pointer to device handle
 * @return Current data rate setting
 */
vcm5883l_data_rate_t vcm5883l_get_data_rate(vcm5883l_handle_t *handle);

/**
 * @brief Set number of samples averaged
 * @param handle Pointer to device handle
 * @param samples Number of samples to average
 */
void vcm5883l_set_samples(vcm5883l_handle_t *handle, vcm5883l_samples_t samples);

/**
 * @brief Get current sample averaging setting
 * @param handle Pointer to device handle
 * @return Current sample setting
 */
vcm5883l_samples_t vcm5883l_get_samples(vcm5883l_handle_t *handle);

/**
 * @brief Set magnetic declination angle for heading calculation
 * @param handle Pointer to device handle
 * @param declination_angle Declination angle in radians
 */
void vcm5883l_set_declination_angle(vcm5883l_handle_t *handle, float declination_angle);

/**
 * @brief Calculate heading in degrees
 * @param handle Pointer to device handle
 * @note Updates the heading_degrees field in the handle's vector
 */
void vcm5883l_get_heading_degrees(vcm5883l_handle_t *handle);

/* Low-level register access functions */

/**
 * @brief Write a single byte to a register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param value Value to write
 * @return true if successful, false otherwise
 */
bool vcm5883l_write_register(vcm5883l_handle_t *handle, uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return true if successful, false otherwise
 */
bool vcm5883l_read_register(vcm5883l_handle_t *handle, uint8_t reg, uint8_t *value);

/**
 * @brief Read 16-bit value from a register pair
 * @param handle Pointer to device handle
 * @param reg Register address (low byte)
 * @param value Pointer to store read value
 * @return true if successful, false otherwise
 */
bool vcm5883l_read_register16(vcm5883l_handle_t *handle, uint8_t reg, int16_t *value);


#ifdef __cplusplus
}
#endif
