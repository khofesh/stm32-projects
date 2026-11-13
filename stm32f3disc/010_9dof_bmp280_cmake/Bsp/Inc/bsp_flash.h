#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Last 2 KB page of the STM32F303VCT6 (256 KB flash); CubeMX owns the linker
 * script and leaves this page out of the image (NFR-12). */
#define BSP_FLASH_PAGE_ADDR 0x0803F800u
#define BSP_FLASH_PAGE_SIZE 2048u

  /* Copy len bytes from the calibration page starting at addr into buf. */
  int32_t bsp_flash_read(uint32_t addr, void *buf, uint16_t len);

  /* Erase the page containing addr, then program len bytes. addr must be page
 * aligned; len must fit one page. The erased page reads 0xFF beyond len.
 *
 * The CPU stalls while the page erases (single bank), so timer interrupts
 * collapse into one. The HAL tick is corrected here; *elapsed_ms (may be
 * NULL) returns the wall-clock duration so callers can correct their own
 * tick counters. */
  int32_t bsp_flash_write_page(uint32_t addr, const void *data, uint16_t len, uint32_t *elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FLASH_H */
