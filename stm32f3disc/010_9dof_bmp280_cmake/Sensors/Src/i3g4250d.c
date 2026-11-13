#include "i3g4250d.h"

#include <errno.h>
#include <stddef.h>

#include "bsp_spi.h"

/* CTRL_REG1: PD=1, BW=01 (25 Hz at 100 Hz ODR), XYZ enable. */
#define CTRL1_VALUE 0x1Fu
/* CTRL_REG4: BDU=1, BLE=0 (LSB at the lower address), FS=01 (+-500 dps). */
#define CTRL4_VALUE 0x90u

#define READ_LEN 6u

#define SCALE (I3G4250D_MDPS_PER_LSB * I3G4250D_DPS_TO_RAD_S)

static uint8_t s_id;

static int32_t write_reg(uint8_t reg, uint8_t value)
{
  return bsp_spi_write(reg, &value, 1u);
}

static int16_t le16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int32_t i3g4250d_init(void)
{
  uint8_t id = 0u;

  s_id = 0u;

  if (bsp_spi_read(I3G4250D_REG_WHO_AM_I, &id, 1u) != 0)
  {
    return -EIO;
  }
  s_id = id;
  if (id != I3G4250D_WHO_AM_I_VALUE)
  {
    return -ENODEV;
  }

  if (write_reg(I3G4250D_REG_CTRL1, CTRL1_VALUE) != 0)
  {
    return -EIO;
  }
  if (write_reg(I3G4250D_REG_CTRL4, CTRL4_VALUE) != 0)
  {
    return -EIO;
  }
  return 0;
}

int32_t i3g4250d_read(vec3_t *rad_s)
{
  uint8_t raw[READ_LEN];

  if (rad_s == NULL)
  {
    return -EINVAL;
  }
  if (bsp_spi_read(I3G4250D_REG_OUT_X_L, raw, READ_LEN) != 0)
  {
    return -EIO;
  }

  rad_s->x = (float)le16(&raw[0]) * SCALE;
  rad_s->y = (float)le16(&raw[2]) * SCALE;
  rad_s->z = (float)le16(&raw[4]) * SCALE;
  return 0;
}

uint8_t i3g4250d_id(void)
{
  return s_id;
}
