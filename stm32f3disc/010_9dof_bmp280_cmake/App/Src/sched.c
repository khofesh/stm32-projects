#include "sched.h"

#include <errno.h>

#include "main.h"

extern TIM_HandleTypeDef htim6;

static volatile uint32_t s_tick;

int32_t sched_start(void)
{
  s_tick = 0u;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    s_tick++;
  }
}
