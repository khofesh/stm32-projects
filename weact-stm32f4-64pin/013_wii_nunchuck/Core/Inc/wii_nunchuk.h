/*
 * wii_nunchuck.h
 *
 *  Created on: Aug 31, 2025
 *      Author: fahmad
 */

#ifndef INC_WII_NUNCHUK_H_
#define INC_WII_NUNCHUK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define WII_NUNCHUK_I2C_ADDR (0x52 << 1)

#define WII_NUNCHUK_I2C_ADDR_7BIT 0x52
#define WII_NUNCHUK_I2C_ADDR_8BIT (WII_NUNCHUK_I2C_ADDR_7BIT << 1) // 0xA4

/* Wii nunchuk data constants */
#define WII_NUNCHUK_DATA_SIZE 6
#define WII_NUNCHUK_DECODE_XOR 0x17
#define WII_NUNCHUK_DECODE_ADD 0x17

/* utility */
#define WII_NUNCHUK_JOYSTICK_CENTER 128
#define WII_NUNCHUK_ACCEL_CENTER 512
#define WII_NUNCHUK_JOYSTICK_DEADZONE 10

/* Wii nunchuk types */
typedef enum {
	WII_NUNCHUK_BLACK = 0, WII_NUNCHUK_WHITE = 1
} wii_nunchuk_type_t;

/* data structure */
typedef struct {
	uint8_t joystick_x; // joystick X-axis (0-255)
	uint8_t joystick_y; // joystick Y-axis (0-255)
	uint16_t accel_x;   // 10-bit
	uint16_t accel_y;
	uint16_t accel_z;
	bool button_c;
	bool button_z;
	bool data_valid;
} wii_nunchuk_data_t;

// states
typedef enum {
	WII_NUNCHUK_STATE_UNINITIALIZED = 0,
	WII_NUNCHUK_STATE_INITIALIZING,
	WII_NUNCHUK_STATE_READY,
	WII_NUNCHUK_STATE_READING,
	WII_NUNCHUK_STATE_ERROR
} wii_nunchuk_state_t;

/* driver struct */
typedef struct {
	I2C_HandleTypeDef *hi2c;                 // I2C handle pointer
	wii_nunchuk_type_t type;                 // Nunchuk type (black/white)
	wii_nunchuk_state_t state;               // Current driver state
	wii_nunchuk_data_t data;                 // Latest nunchuk data
	uint8_t raw_data[WII_NUNCHUK_DATA_SIZE]; // Raw I2C data buffer
	uint8_t init_step;                       // Initialization step counter
	uint32_t last_read_time;                 // Last successful read timestamp
	uint32_t error_count;                    // Error counter
	bool dma_transfer_complete;              // DMA transfer completion flag
	bool data_ready;                         // New data available flag
} wii_nunchuk_handle_t;

/* function return codes */
typedef enum {
	WII_NUNCHUK_OK = 0,
	WII_NUNCHUK_ERROR_INVALID_PARAM,
	WII_NUNCHUK_ERROR_I2C_TIMEOUT,
	WII_NUNCHUK_ERROR_I2C_ERROR,
	WII_NUNCHUK_ERROR_NOT_INITIALIZED,
	WII_NUNCHUK_ERROR_BUSY,
	WII_NUNCHUK_ERROR_NO_DATA
} wii_nunchuk_result_t;

wii_nunchuk_result_t wii_nunchuk_init(wii_nunchuk_handle_t *handle,
		I2C_HandleTypeDef *hi2c, wii_nunchuk_type_t type);
wii_nunchuk_result_t wii_nunchuk_read_async(wii_nunchuk_handle_t *handle);
wii_nunchuk_result_t wii_nunchuk_process_data(wii_nunchuk_handle_t *handle);
wii_nunchuk_result_t wii_nunchuk_get_data(wii_nunchuk_handle_t *handle,
		wii_nunchuk_data_t *data);
wii_nunchuk_state_t wii_nunchuk_get_state(wii_nunchuk_handle_t *handle);
wii_nunchuk_result_t wii_nunchuk_reset(wii_nunchuk_handle_t *handle);
void wii_nunchuk_i2c_rx_cplt_callback(wii_nunchuk_handle_t *handle);
void wii_nunchuk_i2c_tx_cplt_callback(wii_nunchuk_handle_t *handle);
void wii_nunchuk_i2c_error_callback(wii_nunchuk_handle_t *handle);
wii_nunchuk_result_t wii_nunchuk_update(wii_nunchuk_handle_t *handle);
wii_nunchuk_result_t wii_nunchuk_init_working(wii_nunchuk_handle_t *handle,
		I2C_HandleTypeDef *hi2c, wii_nunchuk_type_t type);
wii_nunchuk_result_t wii_nunchuk_read_sync_fixed(wii_nunchuk_handle_t *handle);

/* convert raw joystick to signed value (-128 to +127) */
#define WII_NUNCHUK_JOYSTICK_TO_SIGNED(val) ((int8_t)((val) - WII_NUNCHUK_JOYSTICK_CENTER))

/* Check if joystick is in deadzone */
#define WII_NUNCHUK_JOYSTICK_IN_DEADZONE(val) \
    (abs(WII_NUNCHUK_JOYSTICK_TO_SIGNED(val)) < WII_NUNCHUK_JOYSTICK_DEADZONE)

#ifdef __cplusplus
}
#endif

#endif /* INC_WII_NUNCHUK_H_ */
