#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ms from the HAL tick, us from DWT CYCCNT (both monotonic since boot). */
void     bsp_time_init(void);
uint32_t bsp_time_ms(void);
uint32_t bsp_time_us(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIME_H */
