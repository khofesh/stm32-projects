#include "bmp280_regs_stub.h"

#include <stddef.h>
#include <string.h>

/* Emulates the BMP280 register file behind the LibDriver I2C hooks, so the
 * vendored compensation math can be exercised on the host with the datasheet
 * calibration values. */

#define REG_CAL_START 0x88u
#define REG_CAL_END 0xA1u
#define REG_ID 0xD0u
#define REG_RESET 0xE0u
#define REG_CTRL_MEAS 0xF4u

uint8_t stub_regs[256];

static uint8_t stub_iic_init(void)
{
  return 0u;
}

static uint8_t stub_iic_deinit(void)
{
  return 0u;
}

static uint8_t stub_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)addr;
  for (uint16_t i = 0u; i < len; i++)
  {
    buf[i] = stub_regs[(reg + i) & 0xFFu];
  }
  return 0u;
}

static uint8_t stub_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)addr;
  for (uint16_t i = 0u; i < len; i++)
  {
    /* The soft reset must not clear the emulated NVM calibration. */
    if (reg != REG_RESET)
    {
      stub_regs[(reg + i) & 0xFFu] = buf[i];
    }
  }
  return 0u;
}

static uint8_t stub_spi_xfer(uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)reg;
  (void)buf;
  (void)len;
  return 1u;
}

static void stub_delay_ms(uint32_t ms)
{
  (void)ms;
}

static void stub_debug_print(const char *const fmt, ...)
{
  (void)fmt;
}

void bmp280_stub_link(bmp280_handle_t *h)
{
  memset(h, 0, sizeof(*h));
  DRIVER_BMP280_LINK_IIC_INIT(h, stub_iic_init);
  DRIVER_BMP280_LINK_IIC_DEINIT(h, stub_iic_deinit);
  DRIVER_BMP280_LINK_IIC_READ(h, stub_iic_read);
  DRIVER_BMP280_LINK_IIC_WRITE(h, stub_iic_write);
  DRIVER_BMP280_LINK_SPI_INIT(h, stub_iic_init);
  DRIVER_BMP280_LINK_SPI_DEINIT(h, stub_iic_deinit);
  DRIVER_BMP280_LINK_SPI_READ(h, stub_spi_xfer);
  DRIVER_BMP280_LINK_SPI_WRITE(h, stub_spi_xfer);
  DRIVER_BMP280_LINK_DELAY_MS(h, stub_delay_ms);
  DRIVER_BMP280_LINK_DEBUG_PRINT(h, stub_debug_print);

  (void)bmp280_set_interface(h, BMP280_INTERFACE_IIC);
  (void)bmp280_set_addr_pin(h, BMP280_ADDRESS_ADO_HIGH);
}

void bmp280_stub_calibration(const uint16_t t1, const uint16_t t2, const uint16_t t3,
                             const uint16_t p1, const uint16_t p2, const uint16_t p3,
                             const uint16_t p4, const uint16_t p5, const uint16_t p6,
                             const uint16_t p7, const uint16_t p8, const uint16_t p9)
{
  const uint16_t cal[12] = {t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9};

  memset(stub_regs, 0, sizeof(stub_regs));
  stub_regs[REG_ID] = 0x58u;
  stub_regs[REG_CTRL_MEAS] = 0x03u; /* normal mode: read takes the direct path */

  for (uint8_t i = 0u; i < 12u; i++)
  {
    stub_regs[REG_CAL_START + (2u * i)] = (uint8_t)(cal[i] & 0xFFu);
    stub_regs[REG_CAL_START + (2u * i) + 1u] = (uint8_t)(cal[i] >> 8);
  }
  stub_regs[REG_CAL_END] = 0u;
}
