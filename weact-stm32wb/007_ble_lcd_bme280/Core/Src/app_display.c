/**
 ******************************************************************************
 * @file    app_display.c
 * @brief   Display policy: when the OLED is awake and what it shows
 *
 * The panel is off by default. A user interaction lights it for
 * OLED_TIMEOUT_MS, each further interaction restarts the timeout, and the
 * frame is only recomposed when a value that is actually printed changed.
 *
 * Pushing a frame is spread over eight sequencer passes, one page each, so a
 * repaint never holds the I2C bus long enough to starve the BLE stack.
 ******************************************************************************
 */

#include "app_display.h"
#include "app_common.h"
#include "app_config.h"
#include "app_env.h"
#include "display_app.h"
#include "main.h"
#include "stm32_seq.h"

#define APP_DISPLAY_MS_TO_TICKS(ms)  ((uint32_t)(((uint64_t)(ms) * 1000U) / (uint64_t)CFG_TS_TICK_VAL))

static uint8_t           s_timeout_timer_id;
static uint8_t           s_awake;
static bme280_app_data_t s_latest;
static bme280_app_data_t s_shown;
static uint8_t           s_have_latest;
static uint8_t           s_have_shown;

/**
 * @brief  does the new reading differ in any digit the panel actually prints
 * @note   the rows show 0.01 degC, 0.1 hPa and 0.1 %RH, so comparing the raw
 *         fixed point values would repaint for changes nobody can see
 */
static uint8_t app_display_needs_repaint(const bme280_app_data_t *data)
{
    if (s_have_shown == 0)
    {
        return 1;
    }

    if (data->temperature != s_shown.temperature)
    {
        return 1;
    }

    if ((data->pressure / 10U) != (s_shown.pressure / 10U))
    {
        return 1;
    }

    if (((data->humidity * 10U) / 1024U) != ((s_shown.humidity * 10U) / 1024U))
    {
        return 1;
    }

    return 0;
}

static void app_display_render(void)
{
    if ((s_awake == 0) || (s_have_latest == 0))
    {
        return;
    }

    if (app_display_needs_repaint(&s_latest) == 0)
    {
        return;
    }

    if (DISPLAY_APP_ShowMeasurement(&s_latest) != 0)
    {
        return;
    }

    s_shown      = s_latest;
    s_have_shown = 1;

    UTIL_SEQ_SetTask(1U << CFG_TASK_DISPLAY_PUSH_ID, CFG_SCH_PRIO_0);
}

/**
 * @brief  push one page, reschedule while pages remain
 */
static void app_display_push_task(void)
{
    DISPLAY_APP_Process();

    if (DISPLAY_APP_HasPendingFrame() != 0)
    {
        UTIL_SEQ_SetTask(1U << CFG_TASK_DISPLAY_PUSH_ID, CFG_SCH_PRIO_0);
    }
}

static void app_display_timeout_task(void)
{
    if (s_awake == 0)
    {
        return;
    }

    (void)DISPLAY_APP_PowerOff();
    s_awake      = 0;
    /* the panel keeps no state across a power cycle, so force a full repaint
       on the next wake */
    s_have_shown = 0;

    APP_ENV_OnDisplayTimeout();
}

static void app_display_timeout_cb(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_DISPLAY_TIMEOUT_ID, CFG_SCH_PRIO_0);
}

void APP_DISPLAY_Init(void)
{
    s_awake       = 0;
    s_have_latest = 0;
    s_have_shown  = 0;

    UTIL_SEQ_RegTask(1U << CFG_TASK_DISPLAY_PUSH_ID,    UTIL_SEQ_RFU, app_display_push_task);
    UTIL_SEQ_RegTask(1U << CFG_TASK_DISPLAY_TIMEOUT_ID, UTIL_SEQ_RFU, app_display_timeout_task);

    (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_timeout_timer_id, hw_ts_SingleShot, app_display_timeout_cb);

    /* default state is off, the splash from DISPLAY_APP_Init() has served its
       purpose by the time we get here */
    (void)DISPLAY_APP_PowerOff();
}

void APP_DISPLAY_OnReading(const bme280_app_data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    s_latest      = *data;
    s_have_latest = 1;

    app_display_render();
}

void APP_DISPLAY_Wake(void)
{
    if (s_awake == 0)
    {
        if (DISPLAY_APP_PowerOn() != 0)
        {
            return;
        }
        s_awake = 1;
    }

    app_display_render();

    HW_TS_Stop(s_timeout_timer_id);
    HW_TS_Start(s_timeout_timer_id, APP_DISPLAY_MS_TO_TICKS(OLED_TIMEOUT_MS));
}

uint8_t APP_DISPLAY_IsAwake(void)
{
    return s_awake;
}
