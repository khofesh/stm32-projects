#include "crc16.h"

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
