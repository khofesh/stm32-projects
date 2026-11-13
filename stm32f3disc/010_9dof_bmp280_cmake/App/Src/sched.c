#include "sched.h"

#include <errno.h>

#include "main.h"

extern TIM_HandleTypeDef htim6;

static volatile uint32_t s_tick;
static uint32_t s_overruns;

int32_t sched_start(void)
{
  s_tick = 0u;
  s_overruns = 0u;
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    return -EIO;
  }
  return 0;
}

uint32_t sched_ticks(void)
{
  return s_tick;
}

uint8_t sched_due(uint32_t *next, uint32_t now, uint32_t period_ms)
{
  uint32_t behind;
  uint32_t missed;

  if ((next == NULL) || (period_ms == 0u))
  {
    return 0u;
  }
  if ((int32_t)(now - *next) < 0)
  {
    return 0u;
  }

  behind = now - *next;
  missed = behind / period_ms;
  if (missed != 0u)
  {
    s_overruns += missed;
  }
  *next += (missed + 1u) * period_ms;
  return (missed == 0u) ? 1u : 2u;
}

uint32_t sched_overruns(void)
{
  return s_overruns;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    s_tick++;
  }
}
