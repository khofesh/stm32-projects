#ifndef __BMP280_H
#define __BMP280_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  /* BMP280 on I2C2 at 0x77 (Adafruit breakout, SDO pulled high). */

#define BMP280_ADDR7 0x77u
#define BMP280_CHIP_ID_VALUE 0x58u

  int32_t bmp280_init(void);

  /* Absolute pressure in Pa and temperature in degC; -EAGAIN until the first
 * conversion has completed. */
  int32_t bmp280_read(float *pa, float *deg_c);

  /* Chip ID as read by the last init, 0 before that (T-02). */
  uint8_t bmp280_chip_id(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMP280_H */
