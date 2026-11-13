#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* ms from the HAL tick (1 kHz SysTick), us from the tick + SysTick->VAL.
 * Both are monotonic modulo 2^32; the us counter wraps every ~71.6 min. */
void bsp_time_init(void);
uint32_t bsp_time_ms(void);
uint32_t bsp_time_us(void);

/* Init-time only: the scheduler must never block (SPEC §21.3). */
void bsp_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIME_H */
