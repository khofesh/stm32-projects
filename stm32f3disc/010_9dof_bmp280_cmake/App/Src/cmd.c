#include "cmd.h"

#include "bsp_uart.h"
#include "calibration.h"
#include "main.h"

static void dispatch(uint8_t c)
{
  switch (c)
  {
    case 'G':
      calibration_start_gyro();
      break;
    case 'A':
      calibration_start_accel();
      break;
    case 'M':
      calibration_toggle_mag();
      break;
    case 'B':
      calibration_start_baro();
      break;
    case 'S':
      calibration_request_save();
      break;
    case 'D':
      calibration_reset_defaults();
      break;
    case 'R':
      NVIC_SystemReset();
      break;
    default:
      break;
  }
}

uint8_t cmd_poll(void)
{
  uint8_t c;
  uint8_t got = 0u;

  while (bsp_uart_rx_pop(&c) == 0)
  {
    dispatch(c);
    got = 1u;
  }
  return got;
}
