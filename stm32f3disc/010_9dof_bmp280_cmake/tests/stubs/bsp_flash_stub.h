#ifndef __BSP_FLASH_STUB_H
#define __BSP_FLASH_STUB_H

#include <stdint.h>

#include "bsp_flash.h"

extern uint8_t stub_flash_data[BSP_FLASH_PAGE_SIZE];
extern uint16_t stub_flash_len;
extern uint32_t stub_flash_addr;
extern int stub_flash_result;
extern unsigned stub_flash_calls;
extern uint32_t stub_flash_elapsed_ms;

void stub_flash_reset(void);

#endif /* __BSP_FLASH_STUB_H */
