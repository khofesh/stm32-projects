#include "bsp_spi.h"

#include <errno.h>
#include <stddef.h>

#include "main.h"

extern SPI_HandleTypeDef hspi1;

#define SPI_TIMEOUT_MS 10u

#define SPI_READ 0x80u
#define SPI_INC 0x40u

#define CS_LOW() HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET)

static uint32_t s_errors;

static void spi_fail(void)
{
  s_errors++;
}

int32_t bsp_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t cmd;
  HAL_StatusTypeDef st;

  if ((buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  cmd = reg | SPI_READ;
  if (len > 1u)
  {
    cmd |= SPI_INC;
  }

  CS_LOW();
  st = HAL_SPI_Transmit(&hspi1, &cmd, 1u, SPI_TIMEOUT_MS);
  if (st == HAL_OK)
  {
    st = HAL_SPI_Receive(&hspi1, buf, len, SPI_TIMEOUT_MS);
  }
  CS_HIGH();

  if (st != HAL_OK)
  {
    spi_fail();
    return -EIO;
  }
  return 0;
}

int32_t bsp_spi_write(uint8_t reg, const uint8_t *buf, uint16_t len)
{
  uint8_t cmd = reg;
  HAL_StatusTypeDef st;

  if ((buf == NULL) || (len == 0u))
  {
    return -EINVAL;
  }
  if (len > 1u)
  {
    cmd |= SPI_INC;
  }

  CS_LOW();
  st = HAL_SPI_Transmit(&hspi1, &cmd, 1u, SPI_TIMEOUT_MS);
  if (st == HAL_OK)
  {
    /* HAL takes a non-const pointer but only reads from it. */
    st = HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, SPI_TIMEOUT_MS);
  }
  CS_HIGH();

  if (st != HAL_OK)
  {
    spi_fail();
    return -EIO;
  }
  return 0;
}

uint32_t bsp_spi_errors(void)
{
  return s_errors;
}
