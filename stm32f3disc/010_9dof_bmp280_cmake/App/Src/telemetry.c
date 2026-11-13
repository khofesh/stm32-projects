#include "telemetry.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "bsp_uart.h"
#include "crc16.h"

#define OFF_SEQ 4u
#define OFF_TIME 6u
#define OFF_Q 10u
#define OFF_ACCEL 26u
#define OFF_GYRO 38u
#define OFF_MAG 50u
#define OFF_PRESSURE 62u
#define OFF_ALTITUDE 66u
#define OFF_TEMP 70u
#define OFF_STATUS 72u
#define OFF_CPU 74u
#define OFF_CALIB 75u
#define OFF_OVERRUNS 76u
#define OFF_SKIPPED 78u
#define OFF_RESERVED 80u
#define OFF_CRC 82u

#define CRC_COVER_LEN 80u /* bytes 2..81 */

#define BUILD_DESC_LEN 32u
#define DBG_OFF_DESC 10u
#define DBG_OFF_I2C1 42u
#define DBG_OFF_I2C2 44u
#define DBG_OFF_SPI1 46u
#define DBG_OFF_MAX_LOOP 48u
#define DBG_OFF_CPU 50u
#define DBG_OFF_STACK 52u

static uint8_t s_tx_data[FRAME_LEN];
static uint8_t s_tx_debug[FRAME_LEN];
static uint16_t s_skipped;

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)(v >> 24);
}

static void put_f32(uint8_t *p, float v)
{
  uint32_t u;

  memcpy(&u, &v, sizeof(u));
  put_u32(p, u);
}

static uint16_t get_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float get_f32(const uint8_t *p)
{
  uint32_t u = get_u32(p);
  float v;

  memcpy(&v, &u, sizeof(v));
  return v;
}

static void put_vec3(uint8_t *p, const vec3_t *v)
{
  put_f32(&p[0], v->x);
  put_f32(&p[4], v->y);
  put_f32(&p[8], v->z);
}

static void get_vec3(const uint8_t *p, vec3_t *v)
{
  v->x = get_f32(&p[0]);
  v->y = get_f32(&p[4]);
  v->z = get_f32(&p[8]);
}

static void header(uint8_t out[FRAME_LEN], uint8_t type, uint16_t seq, uint32_t t_ms)
{
  out[0] = FRAME_SYNC0;
  out[1] = FRAME_SYNC1;
  out[2] = type;
  out[3] = (uint8_t)FRAME_PAYLOAD_LEN;
  put_u16(&out[OFF_SEQ], seq);
  put_u32(&out[OFF_TIME], t_ms);
}

static void finish(uint8_t out[FRAME_LEN])
{
  put_u16(&out[OFF_CRC], crc16_ccitt(&out[2], CRC_COVER_LEN));
}

static int32_t check(const uint8_t in[FRAME_LEN], uint8_t type)
{
  if ((in[0] != FRAME_SYNC0) || (in[1] != FRAME_SYNC1) || (in[2] != type) ||
      (in[3] != (uint8_t)FRAME_PAYLOAD_LEN))
  {
    return -EBADMSG;
  }
  if (get_u16(&in[OFF_CRC]) != crc16_ccitt(&in[2], CRC_COVER_LEN))
  {
    return -EBADMSG;
  }
  return 0;
}

void frame_data_pack(uint8_t out[FRAME_LEN], const frame_data_t *f)
{
  header(out, FRAME_TYPE_DATA, f->seq, f->t_ms);

  for (uint8_t i = 0u; i < 4u; i++)
  {
    put_f32(&out[OFF_Q + (4u * i)], f->q[i]);
  }
  put_vec3(&out[OFF_ACCEL], &f->accel);
  put_vec3(&out[OFF_GYRO], &f->gyro);
  put_vec3(&out[OFF_MAG], &f->mag);
  put_f32(&out[OFF_PRESSURE], f->pressure_pa);
  put_f32(&out[OFF_ALTITUDE], f->altitude_m);
  put_u16(&out[OFF_TEMP], (uint16_t)f->temperature_cdeg);
  put_u16(&out[OFF_STATUS], f->status);
  out[OFF_CPU] = f->cpu_load;
  out[OFF_CALIB] = f->calib_state;
  put_u16(&out[OFF_OVERRUNS], f->overruns);
  put_u16(&out[OFF_SKIPPED], f->frames_skipped);
  put_u16(&out[OFF_RESERVED], 0u);
  finish(out);
}

int32_t frame_data_unpack(const uint8_t in[FRAME_LEN], frame_data_t *f)
{
  int32_t st;

  if (f == NULL)
  {
    return -EINVAL;
  }
  st = check(in, FRAME_TYPE_DATA);
  if (st != 0)
  {
    return st;
  }

  f->seq = get_u16(&in[OFF_SEQ]);
  f->t_ms = get_u32(&in[OFF_TIME]);
  for (uint8_t i = 0u; i < 4u; i++)
  {
    f->q[i] = get_f32(&in[OFF_Q + (4u * i)]);
  }
  get_vec3(&in[OFF_ACCEL], &f->accel);
  get_vec3(&in[OFF_GYRO], &f->gyro);
  get_vec3(&in[OFF_MAG], &f->mag);
  f->pressure_pa = get_f32(&in[OFF_PRESSURE]);
  f->altitude_m = get_f32(&in[OFF_ALTITUDE]);
  f->temperature_cdeg = (int16_t)get_u16(&in[OFF_TEMP]);
  f->status = get_u16(&in[OFF_STATUS]);
  f->cpu_load = in[OFF_CPU];
  f->calib_state = in[OFF_CALIB];
  f->overruns = get_u16(&in[OFF_OVERRUNS]);
  f->frames_skipped = get_u16(&in[OFF_SKIPPED]);
  return 0;
}

void frame_debug_pack(uint8_t out[FRAME_LEN], const frame_debug_t *f)
{
  header(out, FRAME_TYPE_DEBUG, f->seq, f->t_ms);

  memset(&out[DBG_OFF_DESC], 0, BUILD_DESC_LEN);
  for (uint8_t i = 0u; (i < BUILD_DESC_LEN) && (f->build_git_desc[i] != '\0'); i++)
  {
    out[DBG_OFF_DESC + i] = (uint8_t)f->build_git_desc[i];
  }
  put_u16(&out[DBG_OFF_I2C1], f->i2c1_errors);
  put_u16(&out[DBG_OFF_I2C2], f->i2c2_errors);
  put_u16(&out[DBG_OFF_SPI1], f->spi1_errors);
  put_u16(&out[DBG_OFF_MAX_LOOP], f->max_loop_us);
  put_u16(&out[DBG_OFF_CPU], f->cpu_load_permille);
  put_u16(&out[DBG_OFF_STACK], f->free_stack);
  memset(&out[DBG_OFF_STACK + 2u], 0, OFF_CRC - (DBG_OFF_STACK + 2u));
  finish(out);
}

int32_t frame_debug_unpack(const uint8_t in[FRAME_LEN], frame_debug_t *f)
{
  int32_t st;

  if (f == NULL)
  {
    return -EINVAL;
  }
  st = check(in, FRAME_TYPE_DEBUG);
  if (st != 0)
  {
    return st;
  }

  f->seq = get_u16(&in[OFF_SEQ]);
  f->t_ms = get_u32(&in[OFF_TIME]);
  for (uint8_t i = 0u; i < BUILD_DESC_LEN; i++)
  {
    f->build_git_desc[i] = (char)in[DBG_OFF_DESC + i];
  }
  f->i2c1_errors = get_u16(&in[DBG_OFF_I2C1]);
  f->i2c2_errors = get_u16(&in[DBG_OFF_I2C2]);
  f->spi1_errors = get_u16(&in[DBG_OFF_SPI1]);
  f->max_loop_us = get_u16(&in[DBG_OFF_MAX_LOOP]);
  f->cpu_load_permille = get_u16(&in[DBG_OFF_CPU]);
  f->free_stack = get_u16(&in[DBG_OFF_STACK]);
  return 0;
}

int32_t telemetry_send_data(const frame_data_t *f)
{
  frame_data_t copy;

  if (f == NULL)
  {
    return -EINVAL;
  }
  copy = *f;
  copy.frames_skipped = s_skipped;
  frame_data_pack(s_tx_data, &copy);
  if (bsp_uart_send_dma(s_tx_data, (uint16_t)FRAME_LEN) != 0)
  {
    s_skipped++;
    return -EBUSY;
  }
  return 0;
}

int32_t telemetry_send_debug(const frame_debug_t *f)
{
  if (f == NULL)
  {
    return -EINVAL;
  }
  frame_debug_pack(s_tx_debug, f);
  return bsp_uart_send_dma(s_tx_debug, (uint16_t)FRAME_LEN);
}

uint16_t telemetry_skipped(void)
{
  return s_skipped;
}
