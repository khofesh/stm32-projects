#include "frame_check.h"

#include <string.h>

#define CRC_COVER_LEN 80u /* bytes 2..81 */

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFu;

  for (uint32_t i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
      if ((crc & 0x8000u) != 0u)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

fc_result_t fc_check(const uint8_t f[FC_FRAME_LEN], uint8_t type)
{
  uint8_t any = 0u;

  for (uint8_t i = 0u; i < FC_FRAME_LEN; i++)
  {
    any |= f[i];
  }
  if (any == 0u)
  {
    return FC_ZERO;
  }
  if ((f[0] != FC_SYNC0) || (f[1] != FC_SYNC1) || (f[2] != type) || (f[3] != (uint8_t)FC_PAYLOAD_LEN))
  {
    return FC_BAD_HDR;
  }
  if (get_u16(&f[FC_OFF_CRC]) != crc16_ccitt(&f[2], CRC_COVER_LEN))
  {
    return FC_BAD_CRC;
  }
  return FC_OK;
}

uint16_t fc_seq(const uint8_t f[FC_FRAME_LEN])
{
  return get_u16(&f[FC_OFF_SEQ]);
}

void fc_seq_track(fc_seq_t *s, uint16_t seq, uint32_t *gaps, uint32_t *dups)
{
  uint16_t d;

  if (s->valid == 0u)
  {
    s->valid = 1u;
    s->last = seq;
    return;
  }
  d = (uint16_t)(seq - s->last);
  if (d == 0u)
  {
    (*dups)++;
    return;
  }
  *gaps += (uint32_t)(d - 1u);
  s->last = seq;
}

void fc_stats_pack(uint8_t out[FC_FRAME_LEN], uint16_t seq, uint32_t t_ms, const uint32_t *v, uint8_t n)
{
  if (n > FC_STATS_MAX)
  {
    n = FC_STATS_MAX;
  }
  memset(out, 0, FC_FRAME_LEN);
  out[0] = FC_SYNC0;
  out[1] = FC_SYNC1;
  out[2] = FC_TYPE_STATS;
  out[3] = (uint8_t)FC_PAYLOAD_LEN;
  put_u16(&out[FC_OFF_SEQ], seq);
  put_u32(&out[FC_OFF_TIME], t_ms);
  for (uint8_t i = 0u; i < n; i++)
  {
    put_u32(&out[FC_OFF_BODY + (4u * i)], v[i]);
  }
  put_u16(&out[FC_OFF_CRC], crc16_ccitt(&out[2], CRC_COVER_LEN));
}
