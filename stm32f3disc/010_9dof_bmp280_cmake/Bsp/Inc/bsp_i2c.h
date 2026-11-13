#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  /* Blocking I2C transfers on the CubeMX handles: bus 1 -> hi2c1, bus 2 -> hi2c2.
 * Register addressing is 8-bit with automatic increment on multi-byte reads. */

#define BSP_I2C_BUS_1 1u
#define BSP_I2C_BUS_2 2u

  int32_t bsp_i2c_read(uint8_t bus, uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);
  int32_t bsp_i2c_write(uint8_t bus, uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len);

  /* DeInit, clock out a stuck slave (9 SCL pulses + STOP), Init. Also run
 * automatically after any failure other than a NACK. */
  int32_t bsp_i2c_recover(uint8_t bus);

  /* Failed transfers on that bus since reset. */
  uint32_t bsp_i2c_errors(uint8_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H */
