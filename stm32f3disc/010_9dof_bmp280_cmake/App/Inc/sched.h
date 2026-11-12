#ifndef __SCHED_H
#define __SCHED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TIM6 runs at 1 kHz; the ISR only advances the tick counter. */
#define SCHED_TICK_HZ 1000u

int32_t  sched_start(void);
uint32_t sched_ticks(void);

#ifdef __cplusplus
}
#endif

#endif /* __SCHED_H */
