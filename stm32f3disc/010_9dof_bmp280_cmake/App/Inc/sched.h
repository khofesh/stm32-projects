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

/* Polled by the main loop. Returns 1 when *next is due and advances it by one
 * period, 2 when it is due and one or more slots were missed (the deadline is
 * realigned instead of bursting), 0 otherwise. */
uint8_t  sched_due(uint32_t *next, uint32_t now, uint32_t period_ms);

/* Add the ticks lost while the CPU was stalled (flash erase): elapsed_ms of
 * wall-clock time passed since the tick read 'since'. */
void     sched_compensate(uint32_t since, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SCHED_H */
