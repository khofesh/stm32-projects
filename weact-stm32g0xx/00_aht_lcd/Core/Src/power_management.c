/**
 * @file power_management.c
 * @brief Power management implementation for battery conservation
 */

#include "power_management.h"

/* Forward declaration */
void SystemClock_Config_AfterStop(void);

/* Private variables */
static RTC_HandleTypeDef *pRtc = NULL;
static PowerMode_t currentMode = POWER_MODE_NORMAL;

/**
 * @brief Initialize power management
 * @param hrtc Pointer to RTC handle
 */
void Power_Init(RTC_HandleTypeDef *hrtc)
{
    pRtc = hrtc;
    currentMode = POWER_MODE_NORMAL;
    
    /* Enable PWR clock */
    __HAL_RCC_PWR_CLK_ENABLE();
}

/**
 * @brief Set power mode
 * @param mode Power mode to set
 */
void Power_SetMode(PowerMode_t mode)
{
    currentMode = mode;
}

/**
 * @brief Get current power mode
 * @return Current power mode
 */
PowerMode_t Power_GetMode(void)
{
    return currentMode;
}

/**
 * @brief Get update interval based on current power mode
 * @return Update interval in milliseconds
 */
uint32_t Power_GetUpdateInterval(void)
{
    switch (currentMode)
    {
        case POWER_MODE_LOW_POWER:
            return UPDATE_INTERVAL_LOW_POWER_MS;
        case POWER_MODE_ULTRA_LOW:
            return UPDATE_INTERVAL_ULTRA_LOW_MS;
        case POWER_MODE_NORMAL:
        default:
            return UPDATE_INTERVAL_NORMAL_MS;
    }
}

/**
 * @brief Get LCD contrast based on current power mode
 * @return LCD contrast value
 */
uint8_t Power_GetLCDContrast(void)
{
    switch (currentMode)
    {
        case POWER_MODE_LOW_POWER:
        case POWER_MODE_ULTRA_LOW:
            return LCD_CONTRAST_LOW_POWER;
        case POWER_MODE_NORMAL:
        default:
            return LCD_CONTRAST_NORMAL;
    }
}

/**
 * @brief Check if LCD should be turned off between updates
 * @return 1 if LCD should be off, 0 otherwise
 */
uint8_t Power_ShouldTurnOffLCD(void)
{
    return (currentMode == POWER_MODE_ULTRA_LOW) ? 1 : 0;
}

/**
 * @brief Configure RTC wakeup timer
 * @param wakeup_time_ms Wakeup time in milliseconds
 */
void Power_ConfigureRTCWakeup(uint32_t wakeup_time_ms)
{
    if (pRtc == NULL) return;
    
    /* Disable wakeup timer first */
    HAL_RTCEx_DeactivateWakeUpTimer(pRtc);
    
    /* Calculate wakeup counter value
     * RTC wakeup clock = LSE/16 = 32768/16 = 2048 Hz
     * For 1 second: counter = 2048
     * For N ms: counter = (N * 2048) / 1000
     */
    uint32_t wakeup_counter = (wakeup_time_ms * 2048) / 1000;
    
    /* Limit to 16-bit max */
    if (wakeup_counter > 0xFFFF)
    {
        wakeup_counter = 0xFFFF;
    }
    
    /* Configure wakeup timer */
    HAL_RTCEx_SetWakeUpTimer_IT(pRtc, wakeup_counter, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
 * @brief Disable RTC wakeup timer
 */
void Power_DisableRTCWakeup(void)
{
    if (pRtc == NULL) return;
    HAL_RTCEx_DeactivateWakeUpTimer(pRtc);
}

/**
 * @brief Enter Stop mode with RTC wakeup
 * @param sleep_ms Sleep duration in milliseconds
 * @note This is the most power-efficient sleep mode while retaining RAM
 */
void Power_EnterStopMode(uint32_t sleep_ms)
{
    if (pRtc == NULL)
    {
        /* Fallback to regular delay if RTC not available */
        HAL_Delay(sleep_ms);
        return;
    }
    
    /* Configure RTC wakeup */
    Power_ConfigureRTCWakeup(sleep_ms);
    
    /* Clear wakeup flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
    
    /* Suspend SysTick to prevent wakeup */
    HAL_SuspendTick();
    
    /* Enter Stop mode 1 (lowest power with RAM retention) */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* After wakeup: Restore system clock */
    SystemClock_Config_AfterStop();
    
    /* Resume SysTick */
    HAL_ResumeTick();
    
    /* Disable wakeup timer */
    Power_DisableRTCWakeup();
}

/**
 * @brief Enter Sleep mode (lighter sleep, faster wakeup)
 * @param sleep_ms Sleep duration in milliseconds
 */
void Power_EnterSleepMode(uint32_t sleep_ms)
{
    if (pRtc == NULL)
    {
        HAL_Delay(sleep_ms);
        return;
    }
    
    /* Configure RTC wakeup */
    Power_ConfigureRTCWakeup(sleep_ms);
    
    /* Enter Sleep mode */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    
    /* Disable wakeup timer */
    Power_DisableRTCWakeup();
}

/**
 * @brief Disable unused peripherals to save power
 */
void Power_DisableUnusedPeripherals(void)
{
    /* Disable DMA clock if not using DMA actively */
    /* Note: Keep enabled if using DMA for UART */
    
    /* Disable unused GPIO ports */
    /* Keep GPIOA, GPIOB, GPIOC for I2C and UART */
}

/**
 * @brief Enable peripherals needed for sensor reading
 */
void Power_EnablePeripheralsForReading(void)
{
    /* Enable I2C clock */
    __HAL_RCC_I2C1_CLK_ENABLE();
}

/**
 * @brief Disable peripherals after sensor reading to save power
 */
void Power_DisablePeripheralsAfterReading(void)
{
    /* In ultra-low power mode, disable I2C between readings */
    if (currentMode == POWER_MODE_ULTRA_LOW)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
    }
}

/**
 * @brief Restore system clock after Stop mode
 * @note This function must match your SystemClock_Config
 */
__weak void SystemClock_Config_AfterStop(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Re-enable HSI and PLL after Stop mode */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        return;
    }

    /* Select PLL as system clock source */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        return;
    }
}
