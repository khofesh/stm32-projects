/**
 ******************************************************************************
 * @file    app_ble_policy.c
 * @brief   BLE publication policy
 *
 * Three separate jobs, all funnelled through one sequencer task so that no
 * ACI command is ever issued from an interrupt or from the sensor path:
 *
 *   - notify the environmental characteristic, but only past a threshold
 *   - keep the advertising manufacturer data carrying the latest T/RH so a
 *     scanner can read the room without connecting
 *   - ask for a faster connection interval when the device gets busy, and a
 *     relaxed one when it settles, once per transition
 ******************************************************************************
 */

#include "app_ble_policy.h"
#include "app_common.h"
#include "app_config.h"
#include "ble.h"
#include "custom_stm.h"
#include "main.h"
#include "stm32_seq.h"

/* the advertising payload built here replaces the generated a_AdvData, which
   has no room for sensor values. Flags are added by aci_gap_set_discoverable()
   and are not part of this buffer, matching the generated code */
#define APP_BLE_ADV_NAME_LEN   5U

/* an AD structure occupies one length byte + one type byte + its payload, so
   the footprint is always payload + 2. Sizing these from the *value* of the
   length byte instead is off by one per field. */
#define APP_BLE_AD_FIELD(payload_len)  ((payload_len) + 2U)

#define APP_BLE_ADV_TXPOWER_LEN   1U   /**< one signed dBm byte */
#define APP_BLE_ADV_MANUF_LEN     6U   /**< company id + int16 temp + uint16 rh */

#define APP_BLE_ADV_TOTAL_LEN  (APP_BLE_AD_FIELD(APP_BLE_ADV_TXPOWER_LEN) + \
                                APP_BLE_AD_FIELD(APP_BLE_ADV_NAME_LEN) + \
                                APP_BLE_AD_FIELD(APP_BLE_ADV_MANUF_LEN))

static uint8_t           s_adv_data[APP_BLE_ADV_TOTAL_LEN];
static uint8_t           s_adv_len;

_Static_assert(APP_BLE_ADV_TOTAL_LEN <= 28,
               "advertising payload must fit in 31 bytes minus the 3 byte flags field");

/* ---------------------------------------------------------------------------
 * Notification LED
 *
 * Lit when the stack accepts a notification, cleared by a timer server
 * single shot. The pin state is retained in STOP2, so the blink survives a
 * sleep and costs no wakeup beyond the one the timer server already owns.
 * ------------------------------------------------------------------------- */

#if (LED_NOTIFY_BLINK == 1)

#if (LED_ACTIVE_HIGH == 1)
#define LED_ON_STATE   GPIO_PIN_SET
#define LED_OFF_STATE  GPIO_PIN_RESET
#else
#define LED_ON_STATE   GPIO_PIN_RESET
#define LED_OFF_STATE  GPIO_PIN_SET
#endif

/* CFG_TS_TICK_VAL is the timer server tick in microseconds */
#define LED_BLINK_TICKS  (((uint32_t)LED_NOTIFY_BLINK_MS * 1000U) / CFG_TS_TICK_VAL)

static uint8_t           s_led_ts_id;

static void app_ble_policy_led_off_cb(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, LED_OFF_STATE);
}

/* HW_TS_Start() restarts a running timer, so back to back notifications
   extend the blink instead of stacking callbacks */
static void app_ble_policy_led_blink(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, LED_ON_STATE);
    HW_TS_Start(s_led_ts_id, LED_BLINK_TICKS);
}

static void app_ble_policy_led_init(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, LED_OFF_STATE);
    (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &s_led_ts_id, hw_ts_SingleShot,
                       app_ble_policy_led_off_cb);
}

#else

#define app_ble_policy_led_blink()  ((void)0)
#define app_ble_policy_led_init()   ((void)0)

#endif /* LED_NOTIFY_BLINK */

static uint16_t          s_conn_handle;
static uint8_t           s_connected;
static uint8_t           s_notify_enabled;

static bme280_app_data_t s_latest;
static uint8_t           s_have_latest;

/* what the client was last told, deliberately distinct from the previous
   sensor reading kept by app_env */
static bme280_app_data_t s_reported;
static uint8_t           s_have_reported;

static uint8_t           s_adv_dirty;
static uint8_t           s_conn_update_pending;
static uint8_t           s_conn_fast;         /**< a fast interval is in force */

static void app_ble_policy_pack(const bme280_app_data_t *data, uint8_t *buf)
{
    uint32_t t = (uint32_t)data->temperature;

    buf[0]  = (uint8_t)(t & 0xFFU);
    buf[1]  = (uint8_t)((t >> 8) & 0xFFU);
    buf[2]  = (uint8_t)((t >> 16) & 0xFFU);
    buf[3]  = (uint8_t)((t >> 24) & 0xFFU);

    buf[4]  = (uint8_t)(data->pressure & 0xFFU);
    buf[5]  = (uint8_t)((data->pressure >> 8) & 0xFFU);
    buf[6]  = (uint8_t)((data->pressure >> 16) & 0xFFU);
    buf[7]  = (uint8_t)((data->pressure >> 24) & 0xFFU);

    buf[8]  = (uint8_t)(data->humidity & 0xFFU);
    buf[9]  = (uint8_t)((data->humidity >> 8) & 0xFFU);
    buf[10] = (uint8_t)((data->humidity >> 16) & 0xFFU);
    buf[11] = (uint8_t)((data->humidity >> 24) & 0xFFU);
}

/**
 * @brief  is this reading worth spending a connection event on
 */
static uint8_t app_ble_policy_worth_notifying(const bme280_app_data_t *data)
{
    int32_t  temp_delta;
    uint32_t hum_delta;
    uint32_t press_delta;

    if (s_have_reported == 0)
    {
        return 1;
    }

    temp_delta = data->temperature - s_reported.temperature;
    if (temp_delta < 0)
    {
        temp_delta = -temp_delta;
    }
    if (temp_delta >= BLE_TEMP_NOTIFY_DELTA_C_X100)
    {
        return 1;
    }

    hum_delta = (data->humidity >= s_reported.humidity)
                    ? (data->humidity - s_reported.humidity)
                    : (s_reported.humidity - data->humidity);
    if (hum_delta >= BLE_HUMIDITY_NOTIFY_DELTA_X1024)
    {
        return 1;
    }

    press_delta = (data->pressure >= s_reported.pressure)
                      ? (data->pressure - s_reported.pressure)
                      : (s_reported.pressure - data->pressure);

    return (press_delta >= BLE_PRESSURE_NOTIFY_DELTA_PA) ? 1U : 0U;
}

/**
 * @brief  rebuild the advertising payload around the latest reading
 * @note   temperature is int16 in 0.01 degC, humidity uint16 in 0.01 %RH
 */
static void app_ble_policy_build_adv(void)
{
    int32_t  t;
    uint32_t h;
    uint8_t  i = 0;

    s_adv_data[i++] = (uint8_t)(1U + APP_BLE_ADV_TXPOWER_LEN);
    s_adv_data[i++] = AD_TYPE_TX_POWER_LEVEL;
    s_adv_data[i++] = 0;

    s_adv_data[i++] = (uint8_t)(1U + APP_BLE_ADV_NAME_LEN);
    s_adv_data[i++] = AD_TYPE_COMPLETE_LOCAL_NAME;
    s_adv_data[i++] = 'W';
    s_adv_data[i++] = 'e';
    s_adv_data[i++] = 'E';
    s_adv_data[i++] = 'n';
    s_adv_data[i++] = 'v';

    t = (s_have_latest != 0) ? s_latest.temperature : 0;
    h = (s_have_latest != 0) ? ((s_latest.humidity * 100U) / 1024U) : 0U;

    s_adv_data[i++] = (uint8_t)(1U + APP_BLE_ADV_MANUF_LEN);
    s_adv_data[i++] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
    s_adv_data[i++] = (uint8_t)(BLE_ADV_COMPANY_ID & 0xFFU);
    s_adv_data[i++] = (uint8_t)((BLE_ADV_COMPANY_ID >> 8) & 0xFFU);
    s_adv_data[i++] = (uint8_t)((uint16_t)t & 0xFFU);
    s_adv_data[i++] = (uint8_t)(((uint16_t)t >> 8) & 0xFFU);
    s_adv_data[i++] = (uint8_t)(h & 0xFFU);
    s_adv_data[i++] = (uint8_t)((h >> 8) & 0xFFU);

    /* a short payload would leave the last AD structure claiming more bytes
       than follow it, which scanners drop, so keep the two in step */
    if (i != (uint8_t)sizeof(s_adv_data))
    {
        s_adv_len = 0U;
        return;
    }

    s_adv_len = i;
}

static void app_ble_policy_publish_task(void)
{
    uint8_t payload[BLE_ENV_PAYLOAD_LEN];

    /* A parameter update issued while the client is still walking the
       attribute table stretches every discovery round trip to a full relaxed
       interval, and a central that times discovery out just drops the link.
       Hold the request until the client has subscribed, which is the signal
       that it got through discovery */
    if ((s_conn_update_pending != 0) && (s_notify_enabled != 0))
    {
        s_conn_update_pending = 0;

        if (s_connected != 0)
        {
            (void)aci_l2cap_connection_parameter_update_req(
                      s_conn_handle,
                      (s_conn_fast != 0) ? BLE_CONN_INTERVAL_FAST_MIN
                                         : BLE_CONN_INTERVAL_RELAXED_MIN,
                      (s_conn_fast != 0) ? BLE_CONN_INTERVAL_FAST_MAX
                                         : BLE_CONN_INTERVAL_RELAXED_MAX,
                      BLE_CONN_LATENCY,
                      BLE_CONN_SUPERVISION_TIMEOUT);
        }
    }

    if (s_have_latest == 0)
    {
        return;
    }

    if ((s_connected != 0) && (s_notify_enabled != 0))
    {
        if (app_ble_policy_worth_notifying(&s_latest) != 0)
        {
            app_ble_policy_pack(&s_latest, payload);

            if (Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_BME_C, payload,
                                                          (uint8_t)BLE_ENV_PAYLOAD_LEN)
                    == BLE_STATUS_SUCCESS)
            {
                s_reported      = s_latest;
                s_have_reported = 1;

                app_ble_policy_led_blink();
            }
        }
    }

    if ((s_connected == 0) && (s_adv_dirty != 0))
    {
        app_ble_policy_build_adv();

        /* the very first sample can land before the stack is advertising, so
           the dirty flag is only cleared on success and the next sample retries */
        if ((s_adv_len != 0U) &&
            (aci_gap_update_adv_data(s_adv_len, s_adv_data) == BLE_STATUS_SUCCESS))
        {
            s_adv_dirty = 0;
        }
    }
}

void APP_BLE_POLICY_Init(void)
{
    s_connected           = 0;
    s_notify_enabled      = 0;
    s_have_latest         = 0;
    s_have_reported       = 0;
    s_adv_dirty           = 0;
    s_conn_update_pending = 0;
    s_conn_fast           = 0;

    app_ble_policy_led_init();

    UTIL_SEQ_RegTask(1U << CFG_TASK_BLE_PUBLISH_ID, UTIL_SEQ_RFU, app_ble_policy_publish_task);
}

void APP_BLE_POLICY_OnReading(const bme280_app_data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    s_latest      = *data;
    s_have_latest = 1;
    s_adv_dirty   = 1;

    UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_PUBLISH_ID, CFG_SCH_PRIO_0);
}

void APP_BLE_POLICY_OnConnected(uint16_t connection_handle)
{
    s_conn_handle   = connection_handle;
    s_connected     = 1;
    /* a new client has been told nothing yet */
    s_have_reported = 0;
}

void APP_BLE_POLICY_OnDisconnected(void)
{
    s_connected           = 0;
    s_notify_enabled      = 0;
    s_have_reported       = 0;
    s_conn_fast           = 0;
    s_conn_update_pending = 0;
    s_adv_dirty           = 1;

    UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_PUBLISH_ID, CFG_SCH_PRIO_0);
}

void APP_BLE_POLICY_OnNotifyEnabled(uint8_t enabled)
{
    s_notify_enabled = enabled;

    if (enabled != 0)
    {
        /* discovery is over, so the policy interval can be asserted now */
        s_conn_update_pending = 1;

        /* give a fresh subscriber the current value straight away */
        s_have_reported = 0;
        UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_PUBLISH_ID, CFG_SCH_PRIO_0);
    }
}

void APP_BLE_POLICY_OnStateChanged(AppState state)
{
    uint8_t want_fast = ((state == APP_STATE_ACTIVE) || (state == APP_STATE_INTERACTIVE)) ? 1U : 0U;

    if ((s_connected == 0) || (want_fast == s_conn_fast))
    {
        s_conn_fast = want_fast;
        return;
    }

    s_conn_fast           = want_fast;
    s_conn_update_pending = 1;

    UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_PUBLISH_ID, CFG_SCH_PRIO_0);
}
