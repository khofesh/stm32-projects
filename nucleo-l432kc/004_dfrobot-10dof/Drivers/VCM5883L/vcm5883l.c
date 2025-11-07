/*
 * vcm5883l.c
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#include "vcm5883l.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool vcm5883l_init(vcm5883l_handle_t *handle, vcm5883l_i2c_handle_t *hi2c)
{
	if (handle == NULL || hi2c == NULL)
	{
		return false;
	}

	// initialize handle
	handle->hi2c = hi2c;
	handle->i2c_address = VCM5883L_ADDRESS;
	handle->declination_angle = 0.0f;
	handle->mg_per_digit = 4.35f; // default for 8GA range

	if (!vcm5883l_i2c_is_device_ready(hi2c, VCM5883L_ADDRESS))
	{
		return false;
	}

    // initialize control registers
	if (!vcm5883l_write_register(handle, VCM5883L_CTR_REG1, 0x00))
	{
		return false;
	}

	if (!vcm5883l_write_register(handle, VCM5883L_CTR_REG2, 0x4D))
	{
		return false;
	}

	return true;
}

vcm5883l_vector_t vcm5883l_read_raw(vcm5883l_handle_t *handle)
{
	int16_t x, y, z;

	vcm5883l_read_register16(handle, VCM5883L_REG_OUT_X_L, &x);
	vcm5883l_read_register16(handle, VCM5883L_REG_OUT_Y_L, &y);
	vcm5883l_read_register16(handle, VCM5883L_REG_OUT_Z_L, &z);

	// apply negative polarity for VCM5883L
	handle->vector.x_axis = -x;
	handle->vector.y_axis = -y;
	handle->vector.z_axis = -z;

	// calculate angles
    handle->vector.angle_xy = (atan2((double)handle->vector.y_axis,
                                     (double)handle->vector.x_axis) * (180.0 / M_PI) + 180.0);
    handle->vector.angle_xz = (atan2((double)handle->vector.z_axis,
                                     (double)handle->vector.x_axis) * (180.0 / M_PI) + 180.0);
    handle->vector.angle_yz = (atan2((double)handle->vector.z_axis,
                                     (double)handle->vector.y_axis) * (180.0 / M_PI) + 180.0);

    return handle->vector;
}

void vcm5883l_set_range(vcm5883l_handle_t *handle, vcm5883l_range_t range)
{
    // VCM5883L has fixed 8GA range
    // This function is included for API compatibility but does nothing
    (void)handle;
    (void)range;
}

vcm5883l_range_t vcm5883l_get_range(vcm5883l_handle_t *handle)
{
    (void)handle;
    return VCM5883L_RANGE_8GA;
}

void vcm5883l_set_measurement_mode(vcm5883l_handle_t *handle, vcm5883l_mode_t mode)
{
    uint8_t value;

    if (vcm5883l_read_register(handle, VCM5883L_CTR_REG2, &value))
    {
        value &= 0xFE;  // Clear bit 0
        value |= mode;   // Set mode bit
        vcm5883l_write_register(handle, VCM5883L_CTR_REG2, value);
    }
}

vcm5883l_mode_t vcm5883l_get_measurement_mode(vcm5883l_handle_t *handle)
{
    uint8_t value = 0;

    if (vcm5883l_read_register(handle, VCM5883L_CTR_REG2, &value))
    {
        value &= 0x01;  // Isolate bit 0
    }

    return (vcm5883l_mode_t)value;
}

void vcm5883l_set_data_rate(vcm5883l_handle_t *handle, vcm5883l_data_rate_t data_rate)
{
    uint8_t value;

    if (vcm5883l_read_register(handle, VCM5883L_CTR_REG2, &value))
    {
        value &= 0xF3;  // Clear bits 2-3
        value |= (data_rate << 2);  // Set data rate bits
        vcm5883l_write_register(handle, VCM5883L_CTR_REG2, value);
    }
}

/**
 * @brief Get current data output rate
 */
vcm5883l_data_rate_t vcm5883l_get_data_rate(vcm5883l_handle_t *handle)
{
    uint8_t value = 0;

    if (vcm5883l_read_register(handle, VCM5883L_CTR_REG2, &value)) {
        value &= 0x0C;  // Isolate bits 2-3
        value >>= 2;     // Shift to LSB position
    }

    return (vcm5883l_data_rate_t)value;
}

/**
 * @brief Set number of samples to average
 */
void vcm5883l_set_samples(vcm5883l_handle_t *handle, vcm5883l_samples_t samples)
{
    uint8_t value;

    // Note: Original code uses QMC5883_REG_CONFIG_1 for VCM5883L samples
    // This might be incorrect in the original Arduino code
    // Using CTR_REG1 here as it's the VCM5883L equivalent
    if (vcm5883l_read_register(handle, QMC5883_REG_CONFIG_1, &value)) {
        value &= 0x3F;  // Clear bits 6-7
        value |= (samples << 6);  // Set sample bits
        vcm5883l_write_register(handle, QMC5883_REG_CONFIG_1, value);
    }
}

/**
 * @brief Get current sample averaging setting
 */
vcm5883l_samples_t vcm5883l_get_samples(vcm5883l_handle_t *handle)
{
    uint8_t value = 0;

    if (vcm5883l_read_register(handle, QMC5883_REG_CONFIG_1, &value)) {
        value &= 0xC0;  // Isolate bits 6-7
        value >>= 6;     // Shift to LSB position
    }

    return (vcm5883l_samples_t)value;
}

/**
 * @brief Set magnetic declination angle for heading correction
 */
void vcm5883l_set_declination_angle(vcm5883l_handle_t *handle, float declination_angle)
{
    handle->declination_angle = declination_angle;
}

/**
 * @brief Calculate heading in degrees with declination correction
 */
void vcm5883l_get_heading_degrees(vcm5883l_handle_t *handle)
{
    float heading = atan2(handle->vector.y_axis, handle->vector.x_axis);

    // Apply declination angle correction
    heading += handle->declination_angle;

    // Normalize to 0-360 degrees
    if (heading < 0) {
        heading += 2.0 * M_PI;
    }
    if (heading > 2.0 * M_PI) {
        heading -= 2.0 * M_PI;
    }

    // Convert to degrees
    handle->vector.heading_degrees = heading * 180.0 / M_PI;
}

/**
 * @brief Write a single byte to a register
 */
bool vcm5883l_write_register(vcm5883l_handle_t *handle, uint8_t reg, uint8_t value)
{
    return vcm5883l_i2c_mem_write(handle->hi2c, handle->i2c_address, reg, &value, 1);
}

/**
 * @brief Read a single byte from a register
 */
bool vcm5883l_read_register(vcm5883l_handle_t *handle, uint8_t reg, uint8_t *value)
{
    return vcm5883l_i2c_mem_read(handle->hi2c, handle->i2c_address, reg, value, 1);
}

/**
 * @brief Read 16-bit value from a register pair (little-endian)
 */
bool vcm5883l_read_register16(vcm5883l_handle_t *handle, uint8_t reg, int16_t *value)
{
    uint8_t data[2];

    // Read 2 bytes (low byte first, then high byte - little endian)
    if (!vcm5883l_i2c_mem_read(handle->hi2c, handle->i2c_address, reg, data, 2)) {
        return false;
    }

    // Combine bytes (VCM5883L uses little-endian format)
    *value = (int16_t)((data[1] << 8) | data[0]);

    return true;
}

