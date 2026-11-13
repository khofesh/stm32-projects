#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "vec3.h"

/* Binary frames, SPEC §23: 84 bytes fixed, little-endian, CRC16-CCITT-FALSE
 * over bytes 2..81. */

#define FRAME_LEN 84u
#define FRAME_SYNC0 0xAAu
#define FRAME_SYNC1 0x55u
#define FRAME_TYPE_DATA 0x01u
#define FRAME_TYPE_DEBUG 0x02u
#define FRAME_PAYLOAD_LEN 78u

/* Status flags, SPEC §23.5. */
#define STATUS_GYRO_OK (1u << 0)
#define STATUS_ACCEL_OK (1u << 1)
#define STATUS_MAG_OK (1u << 2)
#define STATUS_BARO_OK (1u << 3)
#define STATUS_GYRO_CAL (1u << 4)
#define STATUS_ACCEL_CAL (1u << 5)
#define STATUS_MAG_CAL (1u << 6)
#define STATUS_BARO_REF (1u << 7)
#define STATUS_OVERRUN (1u << 8)
#define STATUS_CALIBRATING (1u << 9)
#define STATUS_IMU_ONLY (1u << 10)
#define STATUS_DEBUG_BUILD (1u << 15)

typedef struct
{
  uint16_t seq;
  uint32_t t_ms;
  float q[4];   /* w, x, y, z */
  vec3_t accel; /* m/s^2 */
  vec3_t gyro;  /* rad/s */
  vec3_t mag;   /* uT */
  float pressure_pa;
  float altitude_m;
  int16_t temperature_cdeg;
  uint16_t status;
  uint8_t cpu_load;
  uint8_t calib_state;
  uint16_t overruns;
  uint16_t frames_skipped;
} frame_data_t;

typedef struct
{
  uint16_t seq;
  uint32_t t_ms;
  char build_git_desc[32];
  uint16_t i2c1_errors;
  uint16_t i2c2_errors;
  uint16_t spi1_errors;
  uint16_t max_loop_us;
  uint16_t cpu_load_permille;
  uint16_t free_stack;
} frame_debug_t;

void frame_data_pack(uint8_t out[FRAME_LEN], const frame_data_t *f);
int32_t frame_data_unpack(const uint8_t in[FRAME_LEN], frame_data_t *f);
void frame_debug_pack(uint8_t out[FRAME_LEN], const frame_debug_t *f);
int32_t frame_debug_unpack(const uint8_t in[FRAME_LEN], frame_debug_t *f);

/* Pack and start the DMA transfer. A frame that cannot be sent (-EBUSY while
 * the link is busy, -EIO on a HAL failure) is dropped; data frames are counted
 * in telemetry_skipped(). */
int32_t telemetry_send_data(const frame_data_t *f);
int32_t telemetry_send_debug(const frame_debug_t *f);
uint16_t telemetry_skipped(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H */
