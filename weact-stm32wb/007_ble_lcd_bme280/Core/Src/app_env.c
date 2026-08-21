/**
 ******************************************************************************
 * @file    app_env.c
 * @brief   Environmental sampling policy and STABLE/ACTIVE/INTERACTIVE states
 *
 * One measurement cycle looks like:
 *
 *   sample timer (RTC) -> ENV_MEASURE task -> forced conversion started
 *                      -> conversion timer -> ENV_READ task -> compensate,
 *                         classify, hand to the display and the BLE policy
 *
 * Both timers come from the STM32WB timer server, which runs off the RTC and
 * therefore keeps counting in STOP2. Nothing here polls HAL_GetTick(), and
 * nothing here blocks: between the two timer expiries the application is free
 * to sleep.
 ******************************************************************************
 */

#include "app_env.h"
#include "app_common.h"
#include "app_ble_policy.h"
#include "app_config.h"
#include "app_display.h"
#include "bme280_app.h"
#include "main.h"
#include "stm32_seq.h"

/* the timer server counts RTC ticks, CFG_TS_TICK_VAL is one tick in us */
#define APP_ENV_MS_TO_TICKS(ms)  ((uint32_t)(((uint64_t)(ms) * 1000U) / (uint64_t)CFG_TS_TICK_VAL))

/* a dead sensor must not turn into a 1 Hz stream of failing I2C transactions */
#define APP_ENV_ERROR_BACKOFF_MS  SENSOR_STABLE_INTERVAL_MS

static uint8_t           s_sample_timer_id;
static uint8_t           s_conv_timer_id;

static AppState          s_state;
static AppState          s_state_before_interactive;
static bme280_app_data_t s_current;
static bme280_app_data_t s_previous;
static uint8_t           s_have_reading;
static uint32_t          s_stable_samples;   /**< consecutive unchanged ACTIVE samples */
static uint8_t           s_measuring;        /**< a conversion is in flight */

#if (APP_POWER_TRACE_ENABLED != 0)
#define POWER_TRACE_HIGH()  HAL_GPIO_WritePin(POWER_TRACE_GPIO_Port, POWER_TRACE_Pin, GPIO_PIN_SET)
#define POWER_TRACE_LOW()   HAL_GPIO_WritePin(POWER_TRACE_GPIO_Port, POWER_TRACE_Pin, GPIO_PIN_RESET)
#else
#define POWER_TRACE_HIGH()  do { } while (0)
#define POWER_TRACE_LOW()   do { } while (0)
#endif

/**
 * @brief  sampling interval implied by the current state
 */
static uint32_t app_env_sample_interval_ms(void)
{
    return (s_state == APP_STATE_STABLE) ? SENSOR_STABLE_INTERVAL_MS
                                         : SENSOR_ACTIVE_INTERVAL_MS;
}

static void app_env_schedule_next(uint32_t delay_ms)
{
    uint32_t ticks = APP_ENV_MS_TO_TICKS(delay_ms);

    /* the timer server rejects a zero timeout, and a sub tick delay is a zero
       tick delay after the division above */
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    HW_TS_Stop(s_sample_timer_id);
    HW_TS_Start(s_sample_timer_id, ticks);
}

static void app_env_set_state(AppState next)
{
    if (next == s_state)
    {
        return;
    }

    s_state = next;

    /* the BLE policy only reacts to transitions, so it is told here rather
       than on every sample */
    APP_BLE_POLICY_OnStateChanged(s_state);
}

/**
 * @brief  absolute deltas against the previous reading
 * @return 1 when the change is meaningful, 0 otherwise
 * @note   pressure is measured and published but deliberately does not trigger
 *         ACTIVE, weather drift would keep the device awake for nothing
 */
static uint8_t app_env_has_changed(const bme280_app_data_t *cur,
                                   const bme280_app_data_t *prev)
{
    int32_t  temp_delta;
    uint32_t hum_delta;

    temp_delta = cur->temperature - prev->temperature;
    if (temp_delta < 0)
    {
        temp_delta = -temp_delta;
    }

    if (temp_delta >= TEMP_CHANGE_THRESHOLD_C_X100)
    {
        return 1;
    }

    if (cur->humidity >= prev->humidity)
    {
        hum_delta = cur->humidity - prev->humidity;
    }
    else
    {
        hum_delta = prev->humidity - cur->humidity;
    }

    return (hum_delta >= HUMIDITY_CHANGE_THRESHOLD_X1024) ? 1U : 0U;
}

/**
 * @brief  classify the new reading and move between STABLE and ACTIVE
 * @note   INTERACTIVE is not left here, the display timeout owns that
 */
static void app_env_classify(void)
{
    uint8_t changed;

    if (s_have_reading == 0)
    {
        s_previous       = s_current;
        s_have_reading   = 1;
        s_stable_samples = 0;
        return;
    }

    changed = app_env_has_changed(&s_current, &s_previous);
    s_previous = s_current;

    if (changed != 0)
    {
        s_stable_samples = 0;
        if (s_state == APP_STATE_STABLE)
        {
            app_env_set_state(APP_STATE_ACTIVE);
        }
        else if (s_state == APP_STATE_INTERACTIVE)
        {
            /* remember where to land when the display goes back to sleep */
            s_state_before_interactive = APP_STATE_ACTIVE;
        }
        else
        {
            /* already ACTIVE, the streak counter above is the whole update */
        }

        return;
    }

    /* ACTIVE samples once a second, so counting unchanged samples is an exact
       and wraparound free stand in for a stability stopwatch */
    if (s_stable_samples < ACTIVE_STABLE_SAMPLE_COUNT)
    {
        s_stable_samples++;
    }

    if (s_stable_samples >= ACTIVE_STABLE_SAMPLE_COUNT)
    {
        if (s_state == APP_STATE_ACTIVE)
        {
            app_env_set_state(APP_STATE_STABLE);
        }
        else if (s_state == APP_STATE_INTERACTIVE)
        {
            s_state_before_interactive = APP_STATE_STABLE;
        }
        else
        {
            /* already STABLE */
        }
    }
}

/* --------------------------------------------------------------------------
 * Tasks and timer callbacks. The callbacks run in the RTC interrupt, so they
 * only set a task; all sensor and bus work happens in the sequencer context.
 * ------------------------------------------------------------------------ */

static void app_env_measure_task(void)
{
    POWER_TRACE_HIGH();

    if (BME280_APP_StartMeasurement() != 0)
    {
        POWER_TRACE_LOW();
        s_measuring = 0;
        /* retry on a slow schedule instead of hammering a sensor that is not
           answering */
        app_env_schedule_next(APP_ENV_ERROR_BACKOFF_MS);
        return;
    }

    s_measuring = 1;
    HW_TS_Stop(s_conv_timer_id);
    HW_TS_Start(s_conv_timer_id, APP_ENV_MS_TO_TICKS(BME280_APP_GetMeasurementDelayMs()));
}

static void app_env_read_task(void)
{
    uint8_t ok;

    ok = (BME280_APP_ReadMeasurement(&s_current) == 0) ? 1U : 0U;
    s_measuring = 0;

    POWER_TRACE_LOW();

    if (ok == 0)
    {
        app_env_schedule_next(APP_ENV_ERROR_BACKOFF_MS);
        return;
    }

    app_env_classify();

    APP_DISPLAY_OnReading(&s_current);
    APP_BLE_POLICY_OnReading(&s_current);

    app_env_schedule_next(app_env_sample_interval_ms());
}

static void app_env_interact_task(void)
{
    if (s_state != APP_STATE_INTERACTIVE)
    {
        s_state_before_interactive = s_state;
        app_env_set_state(APP_STATE_INTERACTIVE);
    }

    APP_DISPLAY_Wake();

    /* in STABLE the reading can be up to SENSOR_STABLE_INTERVAL_MS old, so it
       is refreshed before the user reads it. In ACTIVE the 1 s cadence already
       guarantees fresh data */
    if ((s_measuring == 0) &&
        ((s_have_reading == 0) || (app_env_sample_interval_ms() > SENSOR_DATA_STALE_MS)))
    {
        HW_TS_Stop(s_sample_timer_id);
        UTIL_SEQ_SetTask(1U << CFG_TASK_ENV_MEASURE_ID, CFG_SCH_PRIO_0);
    }
}

static void app_env_sample_timer_cb(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_ENV_MEASURE_ID, CFG_SCH_PRIO_0);
}

static void app_env_conv_timer_cb(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_ENV_READ_ID, CFG_SCH_PRIO_0);
}

/* --------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */

void APP_ENV_Init(void)
{
    s_state                    = APP_STATE_STABLE;
    s_state_before_interactive = APP_STATE_STABLE;
    s_have_reading             = 0;
    s_stable_samples           = 0;
    s_measuring                = 0;

    UTIL_SEQ_RegTask(1U << CFG_TASK_ENV_MEASURE_ID,  UTIL_SEQ_RFU, app_env_measure_task);
    UTIL_SEQ_RegTask(1U << CFG_TASK_ENV_READ_ID,     UTIL_SEQ_RFU, app_env_read_task);
    UTIL_SEQ_RegTask(1U << CFG_TASK_ENV_INTERACT_ID, UTIL_SEQ_RFU, app_env_interact_task);

    (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_sample_timer_id, hw_ts_SingleShot, app_env_sample_timer_cb);
    (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_conv_timer_id,   hw_ts_SingleShot, app_env_conv_timer_cb);

    APP_DISPLAY_Init();
    APP_BLE_POLICY_Init();

    /* first measurement as soon as the sequencer runs */
    UTIL_SEQ_SetTask(1U << CFG_TASK_ENV_MEASURE_ID, CFG_SCH_PRIO_0);
}

AppState APP_ENV_GetState(void)
{
    return s_state;
}

uint8_t APP_ENV_GetReading(bme280_app_data_t *out)
{
    if ((out == NULL) || (s_have_reading == 0))
    {
        return 1;
    }

    *out = s_current;

    return 0;
}

void APP_ENV_OnUserInteraction(void)
{
    UTIL_SEQ_SetTask(1U << CFG_TASK_ENV_INTERACT_ID, CFG_SCH_PRIO_0);
}

void APP_ENV_OnDisplayTimeout(void)
{
    if (s_state != APP_STATE_INTERACTIVE)
    {
        return;
    }

    app_env_set_state(s_state_before_interactive);
    app_env_schedule_next(app_env_sample_interval_ms());
}
