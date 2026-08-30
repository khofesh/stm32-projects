#include "stm32h5xx_hal.h"
#include "platform_os.h"

void platform_os_delay(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}
