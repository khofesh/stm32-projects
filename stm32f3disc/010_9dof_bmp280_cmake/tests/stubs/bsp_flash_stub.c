#include "bsp_flash_stub.h"

#include <errno.h>
#include <string.h>

uint8_t stub_flash_data[BSP_FLASH_PAGE_SIZE];
uint16_t stub_flash_len;
uint32_t stub_flash_addr;
int stub_flash_result;
unsigned stub_flash_calls;
uint32_t stub_flash_elapsed_ms;

void stub_flash_reset(void)
{
  memset(stub_flash_data, 0xFF, sizeof(stub_flash_data));
  stub_flash_len = 0u;
  stub_flash_addr = 0u;
  stub_flash_result = 0;
  stub_flash_calls = 0u;
  stub_flash_elapsed_ms = 0u;
}

int32_t bsp_flash_read(uint32_t addr, void *buf, uint16_t len)
{
  if ((buf == NULL) || (addr < BSP_FLASH_PAGE_ADDR) ||
      ((addr - BSP_FLASH_PAGE_ADDR) + len > sizeof(stub_flash_data)))
  {
    return -EINVAL;
  }
  memcpy(buf, &stub_flash_data[addr - BSP_FLASH_PAGE_ADDR], len);
  return 0;
}

int32_t bsp_flash_write_page(uint32_t addr, const void *data, uint16_t len,
                             uint32_t *elapsed_ms)
{
  stub_flash_calls++;
  if (elapsed_ms != NULL)
  {
    *elapsed_ms = stub_flash_elapsed_ms;
  }
  stub_flash_addr = addr;
  stub_flash_len = len;

  if (stub_flash_result != 0)
  {
    return stub_flash_result;
  }
  if ((data == NULL) || (len > sizeof(stub_flash_data)))
  {
    return -EINVAL;
  }
  memcpy(stub_flash_data, data, len);
  return 0;
}
