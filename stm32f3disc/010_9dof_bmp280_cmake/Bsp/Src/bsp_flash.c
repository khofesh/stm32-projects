#include "bsp_flash.h"

#include <errno.h>
#include <stddef.h>

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

int32_t bsp_flash_write_page(uint32_t addr, const void *data, uint16_t len)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0u;
  int32_t st = 0;

  if ((data == NULL) || (len == 0u) || (len > BSP_FLASH_PAGE_SIZE) ||
      ((addr % BSP_FLASH_PAGE_SIZE) != 0u))
  {
    return -EINVAL;
  }

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
    st = program_bytes(addr, (const uint8_t *)data, len);
  }

  (void)HAL_FLASH_Lock();
  return st;
}
