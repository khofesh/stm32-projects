#include "bsp_flash.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "main.h"

static int32_t program_bytes(uint32_t addr, const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0u; i < len; i = (uint16_t)(i + 2u))
  {
    uint16_t half = (uint16_t)data[i];

    if ((uint16_t)(i + 1u) < len)
    {
      half |= (uint16_t)((uint16_t)data[i + 1u] << 8);
    }
    else
    {
      half |= 0xFF00u;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK)
    {
      return -EIO;
    }
  }
  return 0;
}

int32_t bsp_flash_read(uint32_t addr, void *buf, uint16_t len)
{
  if ((buf == NULL) || (addr < BSP_FLASH_PAGE_ADDR) || ((addr - BSP_FLASH_PAGE_ADDR) + len > BSP_FLASH_PAGE_SIZE))
  {
    return -EINVAL;
  }
  memcpy(buf, (const void *)addr, len);
  return 0;
}

/* DWT CYCCNT keeps counting through flash stalls, unlike the tick ISRs. */
static uint32_t cycles_start(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  return DWT->CYCCNT;
}

static uint32_t cycles_to_ms(uint32_t cycles)
{
  return cycles / (SystemCoreClock / 1000u);
}

static int32_t write_locked(uint32_t addr, const uint8_t *data, uint16_t len)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0u;
  int32_t st;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return -EIO;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = addr;
  erase.NbPages = 1u;
  if ((HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) || (page_error != 0xFFFFFFFFu))
  {
    st = -EIO;
  }
  else
  {
    st = program_bytes(addr, data, len);
  }

  if (HAL_FLASH_Lock() != HAL_OK)
  {
    st = -EIO;
  }
  return st;
}

int32_t bsp_flash_write_page(uint32_t addr, const void *data, uint16_t len, uint32_t *elapsed_ms)
{
  uint32_t c0;
  uint32_t t0;
  uint32_t ms;
  uint32_t ticked;
  int32_t st;

  if ((data == NULL) || (len == 0u) || (len > BSP_FLASH_PAGE_SIZE) || ((addr % BSP_FLASH_PAGE_SIZE) != 0u))
  {
    return -EINVAL;
  }

  t0 = HAL_GetTick();
  c0 = cycles_start();
  st = write_locked(addr, (const uint8_t *)data, len);
  ms = cycles_to_ms(DWT->CYCCNT - c0);

  ticked = HAL_GetTick() - t0;
  while (ticked < ms)
  {
    HAL_IncTick();
    ticked++;
  }

  if (elapsed_ms != NULL)
  {
    *elapsed_ms = ms;
  }
  return st;
}
