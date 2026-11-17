#include "lsm303agr.h"

#include <errno.h>
#include <stddef.h>

#include "bsp_i2c.h"

/* Accelerometer registers. */
#define ACC_REG_WHO_AM_I 0x0Fu
#define ACC_REG_CTRL1 0x20u
#define ACC_REG_CTRL4 0x23u
#define ACC_REG_OUT_X_L 0x28u

/* CTRL_REG1_A: ODR=100 Hz, LPen=0 (HR selected in CTRL_REG4_A), XYZ enable. */
#define ACC_CTRL1_VALUE 0x57u
/* CTRL_REG4_A: BDU=1, BLE=0, FS=+-4 g, HR=1. */
#define ACC_CTRL4_VALUE 0x98u

/* Magnetometer registers. */
#define MAG_REG_WHO_AM_I 0x4Fu
#define MAG_REG_CFG_A 0x60u
#define MAG_REG_CFG_B 0x61u
#define MAG_REG_CFG_C 0x62u
#define MAG_REG_OUT_X_L 0x68u

/* CFG_REG_A_M: temperature compensation (required by the datasheet),
 * continuous mode, 100 Hz. */
#define MAG_CFG_A_VALUE 0x8Cu
/* CFG_REG_B_M: offset cancellation on. */
#define MAG_CFG_B_VALUE 0x02u
/* CFG_REG_C_M: BDU on. */
#define MAG_CFG_C_VALUE 0x10u

/* Accelerometer multi-byte reads need the sub-address MSB set; the
 * magnetometer auto-increments on its own. */
#define ACC_AUTO_INC 0x80u

#define READ_LEN 6u

#define ACC_SCALE (LSM303AGR_ACCEL_SENS_MG_PER_LSB * 1e-3f * LSM303AGR_STD_GRAVITY)
#define MAG_SCALE (LSM303AGR_MAG_MGAUSS_PER_LSB * 0.1f)

/* The accelerometer output registers are 16-bit left-justified: the 12-bit
 * high-resolution value sits in bits 15:4 (verified on MB1035-F303C-E02). */
#define ACC_SHIFT 4

static uint8_t s_accel_id;
static uint8_t s_mag_id;

static int32_t reg_write(uint8_t addr7, uint8_t reg, uint8_t value)
{
  return bsp_i2c_write(BSP_I2C_BUS_1, addr7, reg, &value, 1u);
}

static int16_t le16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t read_vec(uint8_t addr7, uint8_t reg, uint8_t shift, vec3_t *out, float scale)
{
  uint8_t raw[READ_LEN];

  if (out == NULL)
  {
    return -EINVAL;
  }
  if (bsp_i2c_read(BSP_I2C_BUS_1, addr7, reg, raw, READ_LEN) != 0)
  {
    return -EIO;
  }

  out->x = (float)((int32_t)le16(&raw[0]) >> shift) * scale;
  out->y = (float)((int32_t)le16(&raw[2]) >> shift) * scale;
  out->z = (float)((int32_t)le16(&raw[4]) >> shift) * scale;
  return 0;
}

int32_t lsm303agr_accel_init(void)
{
  uint8_t id = 0u;

  s_accel_id = 0u;
  if (bsp_i2c_read(BSP_I2C_BUS_1, LSM303AGR_ACCEL_ADDR7, ACC_REG_WHO_AM_I, &id, 1u) != 0)
  {
    return -EIO;
  }
  s_accel_id = id;
  if (id != LSM303AGR_ACCEL_ID_VALUE)
  {
    return -ENODEV;
  }
  if (reg_write(LSM303AGR_ACCEL_ADDR7, ACC_REG_CTRL1, ACC_CTRL1_VALUE) != 0)
  {
    return -EIO;
  }
  if (reg_write(LSM303AGR_ACCEL_ADDR7, ACC_REG_CTRL4, ACC_CTRL4_VALUE) != 0)
  {
    return -EIO;
  }
  return 0;
}

int32_t lsm303agr_mag_init(void)
{
  uint8_t id = 0u;

  s_mag_id = 0u;
  if (bsp_i2c_read(BSP_I2C_BUS_1, LSM303AGR_MAG_ADDR7, MAG_REG_WHO_AM_I, &id, 1u) != 0)
  {
    return -EIO;
  }
  s_mag_id = id;
  if (id != LSM303AGR_MAG_ID_VALUE)
  {
    return -ENODEV;
  }
  if (reg_write(LSM303AGR_MAG_ADDR7, MAG_REG_CFG_B, MAG_CFG_B_VALUE) != 0)
  {
    return -EIO;
  }
  if (reg_write(LSM303AGR_MAG_ADDR7, MAG_REG_CFG_C, MAG_CFG_C_VALUE) != 0)
  {
    return -EIO;
  }
  if (reg_write(LSM303AGR_MAG_ADDR7, MAG_REG_CFG_A, MAG_CFG_A_VALUE) != 0)
  {
    return -EIO;
  }
  return 0;
}

int32_t lsm303agr_init(void)
{
  int32_t acc = lsm303agr_accel_init();
  int32_t mag = lsm303agr_mag_init();

  return (acc != 0) ? acc : mag;
}

int32_t lsm303agr_read_accel(vec3_t *m_s2)
{
  return read_vec(LSM303AGR_ACCEL_ADDR7, (uint8_t)(ACC_REG_OUT_X_L | ACC_AUTO_INC), ACC_SHIFT, m_s2, ACC_SCALE);
}

int32_t lsm303agr_read_mag(vec3_t *uT)
{
  return read_vec(LSM303AGR_MAG_ADDR7, MAG_REG_OUT_X_L, 0, uT, MAG_SCALE);
}

uint8_t lsm303agr_accel_id(void)
{
  return s_accel_id;
}

uint8_t lsm303agr_mag_id(void)
{
  return s_mag_id;
}
