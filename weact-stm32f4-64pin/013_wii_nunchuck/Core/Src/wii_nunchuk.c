/*
 * wii_nunchuck.c
 *
 *  Created on: Aug 31, 2025
 *      Author: fahmad
 */

#include "wii_nunchuk.h"
#include <string.h>

#define WII_NUNCHUK_MAX_ERRORS 5

/* initialization sequences for different nunchuk types */
static const uint8_t init_seq_black[][2] = {
    {0xF0, 0x55}, // initialize first register
    {0xFB, 0x00}  // initialize second register
};

static const uint8_t init_seq_white[][2] = {
    {0x40, 0x00}, // initialize
    {0x00, 0x00}  // follow-up (actually just 1 byte: 0x00)
};

static wii_nunchuk_result_t wii_nunchuk_start_init_step(wii_nunchuk_handle_t *handle);
static wii_nunchuk_result_t wii_nunchuk_request_data(wii_nunchuk_handle_t *handle);
static void wii_nunchuk_decode_raw_data(wii_nunchuk_handle_t *handle);
static bool wii_nunchuk_validate_data(const uint8_t *data);

wii_nunchuk_result_t wii_nunchuk_init(wii_nunchuk_handle_t *handle,
                                      I2C_HandleTypeDef *hi2c,
                                      wii_nunchuk_type_t type)
{
    // Validate parameters
    if (handle == NULL || hi2c == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    // Initialize handle structure
    memset(handle, 0, sizeof(wii_nunchuk_handle_t));
    handle->hi2c = hi2c;
    handle->type = type;
    handle->state = WII_NUNCHUK_STATE_INITIALIZING;
    handle->init_step = 0;
    handle->error_count = 0;
    handle->transfer_complete = true;
    handle->data_ready = false;
    handle->init_timeout = HAL_GetTick() + 5000; // 5 second timeout for init

    // Start the first initialization step
    return wii_nunchuk_start_init_step(handle);
}

wii_nunchuk_result_t wii_nunchuk_read_async(wii_nunchuk_handle_t *handle)
{
    if (handle == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    if (handle->state != WII_NUNCHUK_STATE_READY)
    {
        return WII_NUNCHUK_ERROR_NOT_INITIALIZED;
    }

    if (!handle->transfer_complete)
    {
        return WII_NUNCHUK_ERROR_BUSY;
    }

    // Request data from Nunchuk
    return wii_nunchuk_request_data(handle);
}

wii_nunchuk_result_t wii_nunchuk_process_data(wii_nunchuk_handle_t *handle)
{
    if (handle == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    if (!handle->data_ready)
    {
        return WII_NUNCHUK_ERROR_NO_DATA;
    }

    // Validate and decode the received data
    if (wii_nunchuk_validate_data(handle->raw_data))
    {
        wii_nunchuk_decode_raw_data(handle);
        handle->data.data_valid = true;
        handle->data_ready = false;
        handle->last_read_time = HAL_GetTick();
        handle->error_count = 0;
        return WII_NUNCHUK_OK;
    }
    else
    {
        handle->data.data_valid = false;
        handle->data_ready = false; // Clear the flag even on error
        handle->error_count++;

        // Reset if too many errors
        if (handle->error_count >= WII_NUNCHUK_MAX_ERRORS)
        {
            handle->state = WII_NUNCHUK_STATE_ERROR;
        }

        return WII_NUNCHUK_ERROR_I2C_ERROR;
    }
}

wii_nunchuk_result_t wii_nunchuk_get_data(wii_nunchuk_handle_t *handle,
                                          wii_nunchuk_data_t *data)
{
    if (handle == NULL || data == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    if (handle->state != WII_NUNCHUK_STATE_READY &&
        handle->state != WII_NUNCHUK_STATE_READING)
    {
        return WII_NUNCHUK_ERROR_NOT_INITIALIZED;
    }

    // Copy the latest data
    memcpy(data, &handle->data, sizeof(wii_nunchuk_data_t));

    return WII_NUNCHUK_OK;
}

wii_nunchuk_state_t wii_nunchuk_get_state(wii_nunchuk_handle_t *handle)
{
    if (handle == NULL)
    {
        return WII_NUNCHUK_STATE_UNINITIALIZED;
    }

    return handle->state;
}

wii_nunchuk_result_t wii_nunchuk_reset(wii_nunchuk_handle_t *handle)
{
    if (handle == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    I2C_HandleTypeDef *hi2c_backup = handle->hi2c;
    wii_nunchuk_type_t type_backup = handle->type;

    return wii_nunchuk_init(handle, hi2c_backup, type_backup);
}

wii_nunchuk_result_t wii_nunchuk_update(wii_nunchuk_handle_t *handle)
{
    if (handle == NULL)
    {
        return WII_NUNCHUK_ERROR_INVALID_PARAM;
    }

    wii_nunchuk_result_t result = WII_NUNCHUK_OK;

    switch (handle->state)
    {
    case WII_NUNCHUK_STATE_INITIALIZING:
        // Check for initialization timeout
        if (HAL_GetTick() > handle->init_timeout)
        {
            handle->state = WII_NUNCHUK_STATE_ERROR;
            return WII_NUNCHUK_ERROR_I2C_TIMEOUT;
        }
        break;

    case WII_NUNCHUK_STATE_READY:
        // Nothing to do, ready for commands
        break;

    case WII_NUNCHUK_STATE_READING:
        // Check for read timeout
        if (HAL_GetTick() - handle->last_read_time > 1000)
        {
            handle->state = WII_NUNCHUK_STATE_READY;
            handle->transfer_complete = true;
            handle->error_count++;
        }
        break;

    case WII_NUNCHUK_STATE_ERROR:
        // Try to recover from error state after some time
        if (HAL_GetTick() - handle->last_read_time > 2000)
        {
            result = wii_nunchuk_reset(handle);
        }
        break;

    default:
        break;
    }

    return result;
}

void wii_nunchuk_i2c_rx_cplt_callback(wii_nunchuk_handle_t *handle)
{
    if (handle != NULL)
    {
        handle->transfer_complete = true;
        handle->data_ready = true;
        handle->state = WII_NUNCHUK_STATE_READY;
    }
}

void wii_nunchuk_i2c_tx_cplt_callback(wii_nunchuk_handle_t *handle)
{
    if (handle != NULL)
    {
        handle->transfer_complete = true;

        // If we were initializing, move to next step
        if (handle->state == WII_NUNCHUK_STATE_INITIALIZING)
        {
            handle->init_step++;

            // Start next initialization step or finish
            if ((handle->type == WII_NUNCHUK_BLACK && handle->init_step < 2) ||
                (handle->type == WII_NUNCHUK_WHITE && handle->init_step < 2))
            {
                // Continue with next init step
                wii_nunchuk_start_init_step(handle);
            }
            else
            {
                // Initialization complete
                handle->state = WII_NUNCHUK_STATE_READY;
                handle->last_read_time = HAL_GetTick();
            }
        }
        else if (handle->state == WII_NUNCHUK_STATE_READING)
        {
            // After sending read request, start receiving data
            HAL_StatusTypeDef hal_result;

            // Start interrupt-based read immediately
            hal_result = HAL_I2C_Master_Receive_IT(handle->hi2c,
                                                   WII_NUNCHUK_I2C_ADDR_8BIT,
                                                   handle->raw_data,
                                                   WII_NUNCHUK_DATA_SIZE);

            if (hal_result != HAL_OK)
            {
                handle->transfer_complete = true;
                handle->state = WII_NUNCHUK_STATE_READY;
                handle->error_count++;
            }
            else
            {
                handle->transfer_complete = false;
            }
        }
    }
}

void wii_nunchuk_i2c_error_callback(wii_nunchuk_handle_t *handle)
{
    if (handle != NULL)
    {
        handle->transfer_complete = true;
        handle->error_count++;

        if (handle->error_count >= WII_NUNCHUK_MAX_ERRORS)
        {
            handle->state = WII_NUNCHUK_STATE_ERROR;
        }
        else
        {
            if (handle->state == WII_NUNCHUK_STATE_INITIALIZING)
            {
                // Retry initialization step after error
                wii_nunchuk_start_init_step(handle);
            }
            else
            {
                handle->state = WII_NUNCHUK_STATE_READY;
            }
        }
    }
}

static wii_nunchuk_result_t wii_nunchuk_start_init_step(wii_nunchuk_handle_t *handle)
{
    HAL_StatusTypeDef hal_result;

    if (handle->type == WII_NUNCHUK_BLACK)
    {
        if (handle->init_step < 2)
        {
            hal_result = HAL_I2C_Master_Transmit_IT(handle->hi2c,
                                                    WII_NUNCHUK_I2C_ADDR_8BIT,
                                                    (uint8_t *)init_seq_black[handle->init_step],
                                                    2);
        }
        else
        {
            return WII_NUNCHUK_ERROR_INVALID_PARAM; // Should not happen
        }
    }
    else // WHITE Nunchuk
    {
        if (handle->init_step == 0)
        {
            hal_result = HAL_I2C_Master_Transmit_IT(handle->hi2c,
                                                    WII_NUNCHUK_I2C_ADDR_8BIT,
                                                    (uint8_t *)init_seq_white[0],
                                                    2);
        }
        else if (handle->init_step == 1)
        {
            uint8_t follow_up = 0x00;
            hal_result = HAL_I2C_Master_Transmit_IT(handle->hi2c,
                                                    WII_NUNCHUK_I2C_ADDR_8BIT,
                                                    &follow_up,
                                                    1);
        }
        else
        {
            return WII_NUNCHUK_ERROR_INVALID_PARAM; // Should not happen
        }
    }

    if (hal_result != HAL_OK)
    {
        handle->error_count++;
        if (handle->error_count >= WII_NUNCHUK_MAX_ERRORS)
        {
            handle->state = WII_NUNCHUK_STATE_ERROR;
        }
        return WII_NUNCHUK_ERROR_I2C_ERROR;
    }

    handle->transfer_complete = false;
    return WII_NUNCHUK_OK;
}

static wii_nunchuk_result_t wii_nunchuk_request_data(wii_nunchuk_handle_t *handle)
{
    HAL_StatusTypeDef hal_result;

    // Send data request command (0x00)
    uint8_t request_cmd = 0x00;
    hal_result = HAL_I2C_Master_Transmit_IT(handle->hi2c,
                                            WII_NUNCHUK_I2C_ADDR_8BIT,
                                            &request_cmd,
                                            1);

    if (hal_result != HAL_OK)
    {
        handle->error_count++;
        return WII_NUNCHUK_ERROR_I2C_ERROR;
    }

    handle->transfer_complete = false;
    handle->state = WII_NUNCHUK_STATE_READING;
    handle->last_read_time = HAL_GetTick();

    return WII_NUNCHUK_OK;
}

static void wii_nunchuk_decode_raw_data(wii_nunchuk_handle_t *handle)
{
    uint8_t decoded_data[WII_NUNCHUK_DATA_SIZE];

    // Decode the encrypted data
    for (int i = 0; i < WII_NUNCHUK_DATA_SIZE; i++)
    {
        decoded_data[i] = (handle->raw_data[i] ^ WII_NUNCHUK_DECODE_XOR) + WII_NUNCHUK_DECODE_ADD;
    }

    // Extract joystick data
    handle->data.joystick_x = decoded_data[0];
    handle->data.joystick_y = decoded_data[1];

    // Extract accelerometer data (10-bit resolution)
    handle->data.accel_x = (decoded_data[2] << 2) | ((decoded_data[5] >> 2) & 0x03);
    handle->data.accel_y = (decoded_data[3] << 2) | ((decoded_data[5] >> 4) & 0x03);
    handle->data.accel_z = (decoded_data[4] << 2) | ((decoded_data[5] >> 6) & 0x03);

    // Extract button states (inverted - 0 = pressed, 1 = released)
    handle->data.button_z = !(decoded_data[5] & 0x01);
    handle->data.button_c = !((decoded_data[5] >> 1) & 0x01);
}

static bool wii_nunchuk_validate_data(const uint8_t *data)
{
    // Check for all bytes being the same (error condition)
    bool all_same = true;
    uint8_t first_byte = data[0];

    for (int i = 1; i < WII_NUNCHUK_DATA_SIZE; i++)
    {
        if (data[i] != first_byte)
        {
            all_same = false;
            break;
        }
    }

    if (all_same)
    {
        return false; // All 0xFF, 0x00, etc. are invalid
    }

    // Additional check: ensure we have some variation in the data
    uint8_t unique_values = 0;
    uint8_t seen_values[256] = {0};

    for (int i = 0; i < WII_NUNCHUK_DATA_SIZE; i++)
    {
        if (!seen_values[data[i]])
        {
            seen_values[data[i]] = 1;
            unique_values++;
        }
    }

    // Should have at least 2 different byte values in valid nunchuk data
    return unique_values >= 2;
}
