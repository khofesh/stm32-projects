#ifndef __LSM303AGR_H
#define __LSM303AGR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "vec3.h"

  /* LSM303AGR on I2C1: accelerometer at 0x19, magnetometer at 0x1E. */

#define LSM303AGR_ACCEL_ADDR7 0x19u
#define LSM303AGR_MAG_ADDR7 0x1Eu

#define LSM303AGR_ACCEL_ID_VALUE 0x33u
#define LSM303AGR_MAG_ID_VALUE 0x40u

#define LSM303AGR_ACCEL_SENS_MG_PER_LSB 1.95f /* +-4 g, high resolution */
#define LSM303AGR_MAG_MGAUSS_PER_LSB 1.5f

#define LSM303AGR_STD_GRAVITY 9.80665f

  /* Both parts; the mag is configured even if the accel fails. Returns the
 * first error. */
  int32_t lsm303agr_init(void);
  int32_t lsm303agr_accel_init(void);
  int32_t lsm303agr_mag_init(void);

  /* Acceleration in m/s^2, sensor frame. */
  int32_t lsm303agr_read_accel(vec3_t *m_s2);

  /* Magnetic field in uT, sensor frame. */
  int32_t lsm303agr_read_mag(vec3_t *uT);

  /* WHO_AM_I values as read by the last init, 0 before that (T-02). */
  uint8_t lsm303agr_accel_id(void);
  uint8_t lsm303agr_mag_id(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM303AGR_H */
