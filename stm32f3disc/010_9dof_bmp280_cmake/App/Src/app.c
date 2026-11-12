#include "app.h"

#include "bsp_time.h"
#include "bsp_uart.h"
#include "build_info.h"
#include "main.h"
#include "sched.h"

#define HEARTBEAT_HALF_TICKS (SCHED_TICK_HZ / 2u) /* LD3 toggles every 500 ms -> 1 Hz */

static const char s_banner[] = "\r\n010_9dof_bmp280_cmake " BUILD_GIT_DESC "\r\n";

void app_init(void)
{
  bsp_time_init();

  if (bsp_uart_init() != 0)
  {
    Error_Handler();
  }
  if (sched_start() != 0)
  {
    Error_Handler();
  }
  if (bsp_uart_send_dma((const uint8_t *)s_banner,
                        (uint16_t)(sizeof(s_banner) - 1u)) != 0)
  {
    Error_Handler();
  }
}

void app_run(void)
{
  uint32_t last = sched_ticks();

  for (;;)
  {
    uint32_t now = sched_ticks();

    if ((uint32_t)(now - last) >= HEARTBEAT_HALF_TICKS)
    {
      last = now;
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
}
