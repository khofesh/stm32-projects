#include "bsp_time.h"

#include "main.h"

static uint32_t s_cycles_per_us = 72u;      /* 72 MHz defaults, overwritten by init */
static uint32_t s_cycles_per_ms = 72000u;

void bsp_time_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  s_cycles_per_us = SystemCoreClock / 1000000u;
  if (s_cycles_per_us == 0u)
  {
    s_cycles_per_us = 1u;
  }
  s_cycles_per_ms = s_cycles_per_us * 1000u;
}

uint32_t bsp_time_ms(void)
{
  return HAL_GetTick();
}

uint32_t bsp_time_us(void)
{
  /* Remainder of the current HAL tick taken from the CPU cycle counter. */
  uint32_t cycles = DWT->CYCCNT % s_cycles_per_ms;
  return (HAL_GetTick() * 1000u) + (cycles / s_cycles_per_us);
}
