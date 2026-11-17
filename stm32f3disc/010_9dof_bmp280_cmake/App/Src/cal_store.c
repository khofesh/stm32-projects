#include "cal_store.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "bsp_flash.h"
#include "crc16.h"

#define CAL_STORE_CRC_OFF (CAL_STORE_LEN - 2u)

#define CAL_STORE_FLAGS_OFF 72u
#define CAL_STORE_RSVD_OFF 74u

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

uint16_t cal_store_encode(const calib_params_t *p, uint8_t *buf, uint16_t cap)
{
  uint8_t *q;

  if ((p == NULL) || (buf == NULL) || (cap < CAL_STORE_LEN))
  {
    return 0u;
  }

  put_u32(&buf[0], CAL_STORE_MAGIC);
  put_u16(&buf[4], CAL_STORE_VERSION);
  put_u16(&buf[6], CAL_STORE_PARAM_LEN);

  q = &buf[8];
  put_f32(&q[0], p->gyro_bias.x);
  put_f32(&q[4], p->gyro_bias.y);
  put_f32(&q[8], p->gyro_bias.z);
  q += 12;

  put_f32(&q[0], p->accel_bias.x);
  put_f32(&q[4], p->accel_bias.y);
  put_f32(&q[8], p->accel_bias.z);
  q += 12;

  put_f32(&q[0], p->accel_scale.x);
  put_f32(&q[4], p->accel_scale.y);
  put_f32(&q[8], p->accel_scale.z);
  q += 12;

  put_f32(&q[0], p->mag_bias.x);
  put_f32(&q[4], p->mag_bias.y);
  put_f32(&q[8], p->mag_bias.z);
  q += 12;

  put_f32(&q[0], p->mag_scale.x);
  put_f32(&q[4], p->mag_scale.y);
  put_f32(&q[8], p->mag_scale.z);
  q += 12;

  put_f32(&q[0], p->baro_p0_pa);

  put_u16(&buf[CAL_STORE_FLAGS_OFF], p->flags);
  put_u16(&buf[CAL_STORE_RSVD_OFF], 0u);

  put_u16(&buf[CAL_STORE_CRC_OFF], crc16_ccitt(buf, CAL_STORE_CRC_OFF));
  return (uint16_t)CAL_STORE_LEN;
}

int32_t cal_store_decode(const uint8_t *buf, uint16_t len, calib_params_t *p)
{
  const uint8_t *q;
  calib_params_t out;

  if ((buf == NULL) || (p == NULL) || (len < CAL_STORE_LEN))
  {
    return -EINVAL;
  }
  if (get_u32(&buf[0]) != CAL_STORE_MAGIC)
  {
    return -EBADMSG;
  }
  if (get_u16(&buf[4]) != CAL_STORE_VERSION)
  {
    return -EBADMSG;
  }
  if (get_u16(&buf[6]) != CAL_STORE_PARAM_LEN)
  {
    return -EBADMSG;
  }
  if (get_u16(&buf[CAL_STORE_CRC_OFF]) != crc16_ccitt(buf, CAL_STORE_CRC_OFF))
  {
    return -EBADMSG;
  }

  q = &buf[8];
  out.gyro_bias.x = get_f32(&q[0]);
  out.gyro_bias.y = get_f32(&q[4]);
  out.gyro_bias.z = get_f32(&q[8]);
  q += 12;

  out.accel_bias.x = get_f32(&q[0]);
  out.accel_bias.y = get_f32(&q[4]);
  out.accel_bias.z = get_f32(&q[8]);
  q += 12;

  out.accel_scale.x = get_f32(&q[0]);
  out.accel_scale.y = get_f32(&q[4]);
  out.accel_scale.z = get_f32(&q[8]);
  q += 12;

  out.mag_bias.x = get_f32(&q[0]);
  out.mag_bias.y = get_f32(&q[4]);
  out.mag_bias.z = get_f32(&q[8]);
  q += 12;

  out.mag_scale.x = get_f32(&q[0]);
  out.mag_scale.y = get_f32(&q[4]);
  out.mag_scale.z = get_f32(&q[8]);
  q += 12;

  out.baro_p0_pa = get_f32(&q[0]);

  out.flags = get_u16(&buf[CAL_STORE_FLAGS_OFF]);
  if ((out.flags & (uint16_t)~CAL_FLAG_ALL) != 0u)
  {
    return -EBADMSG;
  }

  *p = out;
  return 0;
}

int32_t cal_store_load(calib_params_t *p)
{
  uint8_t buf[CAL_STORE_LEN];
  int32_t st;

  if (p == NULL)
  {
    return -EINVAL;
  }

  st = bsp_flash_read(BSP_FLASH_PAGE_ADDR, buf, (uint16_t)sizeof(buf));
  if (st == 0)
  {
    st = cal_store_decode(buf, (uint16_t)sizeof(buf), p);
  }
  if (st != 0)
  {
    calib_params_defaults(p);
  }
  return st;
}

int32_t cal_store_save(const calib_params_t *p, uint32_t *elapsed_ms)
{
  uint8_t buf[CAL_STORE_LEN];
  uint16_t n;

  n = cal_store_encode(p, buf, (uint16_t)sizeof(buf));
  if (n == 0u)
  {
    return -EINVAL;
  }
  return bsp_flash_write_page(BSP_FLASH_PAGE_ADDR, buf, n, elapsed_ms);
}
