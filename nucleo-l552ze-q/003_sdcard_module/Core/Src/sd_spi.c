/*
 * sd_spi.c
 *
 *  Created on: Dec 11, 2025
 *      Author: fahmad
 */

#include "sd_spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

uint8_t card_initialized = 0;

static uint8_t is_sdhc = 0;

static void sd_cs_low(void)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

static void sd_cs_high(void)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static uint8_t sd_spi_txrx(uint8_t data)
{
  uint8_t rx = 0xFF;
  (void)HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
  return rx;
}

static void sd_spi_rx(uint8_t *buf, uint16_t len)
{
  uint8_t tx = 0xFF;
  (void)HAL_SPI_TransmitReceive(&hspi1, &tx, buf, len, HAL_MAX_DELAY);
}

static void sd_spi_tx(const uint8_t *buf, uint16_t len)
{
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

static void sd_spi_clock_idle(uint16_t cycles)
{
  for (uint16_t i = 0; i < cycles; i++)
  {
    (void)sd_spi_txrx(0xFF);
  }
}

static uint8_t sd_wait_r1(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t r1;

  do
  {
    r1 = sd_spi_txrx(0xFF);
    if ((r1 & 0x80U) == 0U)
    {
      return r1;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return 0xFF;
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t frame[6];

  frame[0] = (uint8_t)(0x40U | cmd);
  frame[1] = (uint8_t)(arg >> 24);
  frame[2] = (uint8_t)(arg >> 16);
  frame[3] = (uint8_t)(arg >> 8);
  frame[4] = (uint8_t)(arg);
  frame[5] = crc;

  sd_cs_low();
  (void)sd_spi_txrx(0xFF);
  sd_spi_tx(frame, sizeof(frame));

  return sd_wait_r1(200);
}

static void sd_end_cmd(void)
{
  sd_cs_high();
  (void)sd_spi_txrx(0xFF);
}

static SD_Status sd_set_spi_prescaler(uint32_t prescaler)
{
  if (hspi1.Init.BaudRatePrescaler == prescaler)
  {
    return SD_OK;
  }

  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    return SD_ERROR;
  }

  hspi1.Init.BaudRatePrescaler = prescaler;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return SD_ERROR;
  }

  return SD_OK;
}

static SD_Status sd_wait_data_token(uint8_t token, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t b;

  do
  {
    b = sd_spi_txrx(0xFF);
    if (b == token)
    {
      return SD_OK;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return SD_ERROR;
}

uint8_t sd_is_sdhc(void)
{
  return is_sdhc;
}

SD_Status SD_SPI_Init(void)
{
  uint8_t r1;
  uint8_t r7[4];
  uint8_t ocr[4];

  card_initialized = 0;
  is_sdhc = 0;

  if (sd_set_spi_prescaler(SPI_BAUDRATEPRESCALER_256) != SD_OK)
  {
    return SD_ERROR;
  }

  sd_cs_high();
  sd_spi_clock_idle(20);

  r1 = sd_send_cmd(CMD0, 0, 0x95);
  sd_end_cmd();
  if (r1 != 0x01)
  {
    return SD_ERROR;
  }

  r1 = sd_send_cmd(CMD8, 0x000001AA, 0x87);
  if (r1 == 0x01)
  {
    sd_spi_rx(r7, sizeof(r7));
    sd_end_cmd();
    if (r7[2] != 0x01 || r7[3] != 0xAA)
    {
      return SD_ERROR;
    }

    uint32_t start = HAL_GetTick();
    do
    {
      r1 = sd_send_cmd(CMD55, 0, 0xFF);
      sd_end_cmd();
      if (r1 > 0x01)
      {
        return SD_ERROR;
      }

      r1 = sd_send_cmd(ACMD41, 0x40000000, 0xFF);
      sd_end_cmd();
      if (r1 == 0x00)
      {
        break;
      }
    } while ((HAL_GetTick() - start) < 1000);

    if (r1 != 0x00)
    {
      return SD_ERROR;
    }

    r1 = sd_send_cmd(CMD58, 0, 0xFF);
    if (r1 != 0x00)
    {
      sd_end_cmd();
      return SD_ERROR;
    }
    sd_spi_rx(ocr, sizeof(ocr));
    sd_end_cmd();

    if ((ocr[0] & 0x40U) != 0U)
    {
      is_sdhc = 1;
    }
  }
  else
  {
    sd_end_cmd();

    uint32_t start = HAL_GetTick();
    do
    {
      r1 = sd_send_cmd(CMD55, 0, 0xFF);
      sd_end_cmd();
      if (r1 > 0x01)
      {
        return SD_ERROR;
      }

      r1 = sd_send_cmd(ACMD41, 0, 0xFF);
      sd_end_cmd();
      if (r1 == 0x00)
      {
        break;
      }
    } while ((HAL_GetTick() - start) < 1000);

    if (r1 != 0x00)
    {
      return SD_ERROR;
    }
  }

  (void)sd_set_spi_prescaler(SPI_BAUDRATEPRESCALER_32);

  card_initialized = 1;
  return SD_OK;
}

static uint32_t sd_lba_to_addr(uint32_t sector)
{
  if (is_sdhc)
  {
    return sector;
  }
  return sector * 512U;
}

SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count)
{
  if (!card_initialized)
  {
    return SD_ERROR;
  }

  for (uint32_t i = 0; i < count; i++)
  {
    uint32_t addr = sd_lba_to_addr(sector + i);
    uint8_t r1 = sd_send_cmd(CMD17, addr, 0xFF);
    if (r1 != 0x00)
    {
      sd_end_cmd();
      return SD_ERROR;
    }

    if (sd_wait_data_token(0xFE, 200) != SD_OK)
    {
      sd_end_cmd();
      return SD_ERROR;
    }

    sd_spi_rx(&buff[i * 512U], 512);
    (void)sd_spi_txrx(0xFF);
    (void)sd_spi_txrx(0xFF);
    sd_end_cmd();
  }

  return SD_OK;
}

SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count)
{
  if (!card_initialized)
  {
    return SD_ERROR;
  }

  for (uint32_t i = 0; i < count; i++)
  {
    uint32_t addr = sd_lba_to_addr(sector + i);
    uint8_t r1 = sd_send_cmd(CMD24, addr, 0xFF);
    if (r1 != 0x00)
    {
      sd_end_cmd();
      return SD_ERROR;
    }

    (void)sd_spi_txrx(0xFF);
    (void)sd_spi_txrx(0xFE);
    sd_spi_tx(&buff[i * 512U], 512);
    (void)sd_spi_txrx(0xFF);
    (void)sd_spi_txrx(0xFF);

    uint8_t resp = sd_spi_txrx(0xFF);
    if ((resp & 0x1FU) != 0x05U)
    {
      sd_end_cmd();
      return SD_ERROR;
    }

    uint32_t start = HAL_GetTick();
    while (sd_spi_txrx(0xFF) == 0x00)
    {
      if ((HAL_GetTick() - start) > 500)
      {
        sd_end_cmd();
        return SD_ERROR;
      }
    }

    sd_end_cmd();
  }

  return SD_OK;
}

SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count)
{
  return SD_ReadBlocks(buff, sector, count);
}

SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count)
{
  return SD_WriteBlocks(buff, sector, count);
}

