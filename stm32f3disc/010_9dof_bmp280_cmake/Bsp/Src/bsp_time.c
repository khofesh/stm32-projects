#include "bsp_time.h"

#include "main.h"

void bsp_time_init(void)
{
}

uint32_t bsp_time_ms(void)
{
  return HAL_GetTick();
}

void bsp_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

uint32_t bsp_time_us(void)
{
  uint32_t ms;
  uint32_t val;
  uint32_t pend;

  /* Retry if the SysTick ISR ran between the reads. */
  do
  {
    ms = HAL_GetTick();
    val = SysTick->VAL;
    pend = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
  } while (ms != HAL_GetTick());

  const uint32_t load = SysTick->LOAD;

  /* Counter reloaded but the ISR is masked: the tick is one ms behind. */
  if ((pend != 0u) && (val > (load / 2u)))
  {
    ms++;
  }

  /* Wraps modulo 2^32 continuously; compare with unsigned differences. */
  return (ms * 1000u) + (((load - val) * 1000u) / (load + 1u));
}
