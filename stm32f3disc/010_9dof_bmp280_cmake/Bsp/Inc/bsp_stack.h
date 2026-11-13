#ifndef __BSP_STACK_H
#define __BSP_STACK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  /* Stack high-water mark (debug frame, SPEC §23.4). bsp_stack_init() paints
 * the free RAM below the current stack pointer; bsp_stack_free_bytes()
 * returns the bytes that were never touched. Call the init once at boot,
 * before the deep call paths. */

  void bsp_stack_init(void);
  uint32_t bsp_stack_free_bytes(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STACK_H */
