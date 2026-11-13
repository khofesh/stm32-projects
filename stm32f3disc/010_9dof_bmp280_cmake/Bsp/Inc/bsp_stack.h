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
 * before the deep call paths. Only the 1 KB _Min_Stack_Size reservation
 * (_sstack.._estack) is painted: 0 means the stack reached or overflowed
 * _sstack, and the overflow depth is not measured. */

void bsp_stack_init(void);
uint32_t bsp_stack_free_bytes(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_STACK_H */
