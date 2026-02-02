/**
 * @file power_management.h
 * @brief Power management functions for battery conservation
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "stm32g0xx_hal.h"

/* Power mode configuration */
typedef enum {
    POWER_MODE_NORMAL,      /* Full performance, LCD always on */
    POWER_MODE_LOW_POWER,   /* Reduced updates, LCD dimmed */
    POWER_MODE_ULTRA_LOW    /* Minimal updates, LCD off between readings */
} PowerMode_t;

/* Configuration */
#define UPDATE_INTERVAL_NORMAL_MS       2000    /* 2 seconds */
#define UPDATE_INTERVAL_LOW_POWER_MS    10000   /* 10 seconds */
#define UPDATE_INTERVAL_ULTRA_LOW_MS    30000   /* 30 seconds */

#define LCD_CONTRAST_NORMAL             0xCF    /* Full brightness */
#define LCD_CONTRAST_LOW_POWER          0x40    /* Dimmed */

/* Function prototypes */
void Power_Init(RTC_HandleTypeDef *hrtc);
void Power_SetMode(PowerMode_t mode);
PowerMode_t Power_GetMode(void);
uint32_t Power_GetUpdateInterval(void);
uint8_t Power_GetLCDContrast(void);
uint8_t Power_ShouldTurnOffLCD(void);

/* Sleep functions */
void Power_EnterStopMode(uint32_t sleep_ms);
void Power_EnterSleepMode(uint32_t sleep_ms);

/* RTC wakeup configuration */
void Power_ConfigureRTCWakeup(uint32_t wakeup_time_ms);
void Power_DisableRTCWakeup(void);

/* Peripheral power management */
void Power_DisableUnusedPeripherals(void);
void Power_EnablePeripheralsForReading(void);
void Power_DisablePeripheralsAfterReading(void);

#endif /* POWER_MANAGEMENT_H */
