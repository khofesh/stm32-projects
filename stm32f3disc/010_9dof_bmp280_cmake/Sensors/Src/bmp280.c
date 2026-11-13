#include "bmp280.h"

#include <errno.h>
#include <stddef.h>

#include "bsp_i2c.h"
#include "bsp_time.h"

/* LibDriver exposes bmp280_init(bmp280_handle_t *), which collides with the
 * wrapper API; the vendored object is compiled with this rename. */
#define bmp280_init ld_bmp280_init
#include "driver_bmp280.h"
#undef bmp280_init

#define BMP280_REG_ID 0xD0u
/* ADC reset value: no conversion has completed yet (first ~45 ms in normal
 * mode) or the measurement is skipped. */
#define BMP280_ADC_NO_DATA 0x80000u

static bmp280_handle_t s_h;
static uint8_t s_id;

static uint8_t iic_init(void)
{
  return 0u;
}

static uint8_t iic_deinit(void)
{
  return 0u;
}

static uint8_t iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  /* LibDriver addresses the device with the R/W bit included. */
  return (bsp_i2c_read(BSP_I2C_BUS_2, (uint8_t)(addr >> 1), reg, buf, len) == 0) ? 0u : 1u;
}

// cppcheck-suppress constParameterCallback
static uint8_t iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (bsp_i2c_write(BSP_I2C_BUS_2, (uint8_t)(addr >> 1), reg, buf, len) == 0) ? 0u : 1u;
}

/* Unused SPI hooks; bmp280_init() requires them to be non-NULL. */
static uint8_t spi_init(void)
{
  return 1u;
}

static uint8_t spi_deinit(void)
{
  return 1u;
}

static uint8_t spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)reg;
  (void)buf;
  (void)len;
  return 1u;
}

static uint8_t spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)reg;
  (void)buf;
  (void)len;
  return 1u;
}

static void delay_ms(uint32_t ms)
{
  bsp_delay_ms(ms);
}

static void debug_print(const char *const fmt, ...)
{
  (void)fmt;
}

static int32_t config(void)
{
  if (bmp280_set_temperatue_oversampling(&s_h, BMP280_OVERSAMPLING_x2) != 0u)
  {
    return -EIO;
  }
  if (bmp280_set_pressure_oversampling(&s_h, BMP280_OVERSAMPLING_x16) != 0u)
  {
    return -EIO;
  }
  if (bmp280_set_filter(&s_h, BMP280_FILTER_COEFF_16) != 0u)
  {
    return -EIO;
  }
  if (bmp280_set_standby_time(&s_h, BMP280_STANDBY_TIME_0P5_MS) != 0u)
  {
    return -EIO;
  }
  if (bmp280_set_mode(&s_h, BMP280_MODE_NORMAL) != 0u)
  {
    return -EIO;
  }
  return 0;
}

int32_t bmp280_init(void)
{
  uint8_t id = 0u;

  s_id = 0u;

  if (bsp_i2c_read(BSP_I2C_BUS_2, BMP280_ADDR7, BMP280_REG_ID, &id, 1u) != 0)
  {
    return -EIO;
  }
  s_id = id;
  if (id != BMP280_CHIP_ID_VALUE)
  {
    return -ENODEV;
  }

  DRIVER_BMP280_LINK_INIT(&s_h, bmp280_handle_t);
  DRIVER_BMP280_LINK_IIC_INIT(&s_h, iic_init);
  DRIVER_BMP280_LINK_IIC_DEINIT(&s_h, iic_deinit);
  DRIVER_BMP280_LINK_IIC_READ(&s_h, iic_read);
  DRIVER_BMP280_LINK_IIC_WRITE(&s_h, iic_write);
  DRIVER_BMP280_LINK_SPI_INIT(&s_h, spi_init);
  DRIVER_BMP280_LINK_SPI_DEINIT(&s_h, spi_deinit);
  DRIVER_BMP280_LINK_SPI_READ(&s_h, spi_read);
  DRIVER_BMP280_LINK_SPI_WRITE(&s_h, spi_write);
  DRIVER_BMP280_LINK_DELAY_MS(&s_h, delay_ms);
  DRIVER_BMP280_LINK_DEBUG_PRINT(&s_h, debug_print);

  if (bmp280_set_interface(&s_h, BMP280_INTERFACE_IIC) != 0u)
  {
    return -EIO;
  }
  if (bmp280_set_addr_pin(&s_h, BMP280_ADDRESS_ADO_HIGH) != 0u)
  {
    return -EIO;
  }
  if (ld_bmp280_init(&s_h) != 0u)
  {
    return -EIO;
  }
  return config();
}

int32_t bmp280_read(float *pa, float *deg_c)
{
  uint32_t t_raw = 0u;
  uint32_t p_raw = 0u;
  float t = 0.0f;
  float p = 0.0f;

  if ((pa == NULL) || (deg_c == NULL))
  {
    return -EINVAL;
  }
  if (bmp280_read_temperature_pressure(&s_h, &t_raw, &t, &p_raw, &p) != 0u)
  {
    return -EIO;
  }
  if ((t_raw == BMP280_ADC_NO_DATA) || (p_raw == BMP280_ADC_NO_DATA))
  {
    return -EAGAIN;
  }

  *pa = p;
  *deg_c = t;
  return 0;
}

uint8_t bmp280_chip_id(void)
{
  return s_id;
}
