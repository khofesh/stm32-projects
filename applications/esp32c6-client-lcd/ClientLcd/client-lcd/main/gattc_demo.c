/*
 * ESP-IDF BLE GATT Client for SEN55 sensor data from STM32WB55
 * Based on ESP-IDF gattc_demo example
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define GATTC_TAG "SEN55_CLIENT"

// Bluetooth address macros for compatibility
#ifndef ESP_BD_ADDR_STR
#define ESP_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif
#ifndef ESP_BD_ADDR_HEX
#define ESP_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#endif

// SEN55 Service and Characteristic UUIDs (128-bit)
#define SERVICE_UUID        "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"
#define SEN55_CHAR_UUID     "0000fe42-8e22-4541-9d4c-21edae82ed19"

// UUID128 in byte array format (little-endian)
static uint8_t service_uuid128[ESP_UUID_LEN_128] = {
    0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
    0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00
};

static uint8_t char_uuid128[ESP_UUID_LEN_128] = {
    0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
    0x41, 0x45, 0x22, 0x8e, 0x42, 0xfe, 0x00, 0x00
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

static const char remote_device_name[] = "sen55CST";
static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// Service UUID filter (128-bit)
static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {
        .uuid128 = {0},
    },
};

// Characteristic UUID filter (128-bit)
static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {
        .uuid128 = {0},
    },
};

// CCCD UUID (0x2902)
static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

// SEN55 data structure
typedef struct
{
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm4_0;
    uint16_t pm10;
    int16_t temperature;
    int16_t humidity;
    int16_t vocIndex;
    int16_t noxIndex;
} sen55_data_t;

// Parse and print SEN55 data
static void print_sen55_data(uint8_t *data, uint16_t len)
{
    if (len < 16)
    {
        ESP_LOGW(GATTC_TAG, "Data too short: %d bytes", len);
        return;
    }

    sen55_data_t sensor_data;

    sensor_data.pm1_0 = data[0] | (data[1] << 8);
    sensor_data.pm2_5 = data[2] | (data[3] << 8);
    sensor_data.pm4_0 = data[4] | (data[5] << 8);
    sensor_data.pm10 = data[6] | (data[7] << 8);
    sensor_data.temperature = (int16_t)(data[8] | (data[9] << 8));
    sensor_data.humidity = (int16_t)(data[10] | (data[11] << 8));
    sensor_data.vocIndex = (int16_t)(data[12] | (data[13] << 8));
    sensor_data.noxIndex = (int16_t)(data[14] | (data[15] << 8));

    ESP_LOGI(GATTC_TAG, "\n=== SEN55 Data ===");
    ESP_LOGI(GATTC_TAG, "PM1.0: %.1f µg/m³", sensor_data.pm1_0 / 10.0);
    ESP_LOGI(GATTC_TAG, "PM2.5: %.1f µg/m³", sensor_data.pm2_5 / 10.0);
    ESP_LOGI(GATTC_TAG, "PM4.0: %.1f µg/m³", sensor_data.pm4_0 / 10.0);
    ESP_LOGI(GATTC_TAG, "PM10:  %.1f µg/m³", sensor_data.pm10 / 10.0);

    if (sensor_data.temperature != 0x7fff)
    {
        ESP_LOGI(GATTC_TAG, "Temp:  %.1f °C", sensor_data.temperature / 200.0);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "Temp:  N/A");
    }

    if (sensor_data.humidity != 0x7fff)
    {
        ESP_LOGI(GATTC_TAG, "Humid: %.1f %%", sensor_data.humidity / 100.0);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "Humid: N/A");
    }

    if (sensor_data.vocIndex != 0x7fff)
    {
        ESP_LOGI(GATTC_TAG, "VOC:   %.0f", sensor_data.vocIndex / 10.0);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "VOC:   N/A");
    }

    if (sensor_data.noxIndex != 0x7fff)
    {
        ESP_LOGI(GATTC_TAG, "NOx:   %.0f", sensor_data.noxIndex / 10.0);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "NOx:   N/A");
    }
    ESP_LOGI(GATTC_TAG, "==================\n");
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d",
                 param->reg.status, param->reg.app_id, gattc_if);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(GATTC_TAG, "Set scan params error, error code = %x", scan_ret);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote " ESP_BD_ADDR_STR "",
                 p_data->connect.conn_id, ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret)
        {
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        ESP_LOGI(GATTC_TAG, "Searching for SEN55 service UUID128: %s", SERVICE_UUID);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d",
                 p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "Start handle %d, end handle %d, current handle value %d",
                 p_data->search_res.start_handle, p_data->search_res.end_handle,
                 p_data->search_res.srvc_id.inst_id);

        // Log the UUID we found
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16)
        {
            ESP_LOGI(GATTC_TAG, "Found service UUID16: 0x%04x",
                     p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        else if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128)
        {
            ESP_LOGI(GATTC_TAG, "Found service UUID128:");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->search_res.srvc_id.uuid.uuid.uuid128, ESP_UUID_LEN_128);
        }

        // Check if this is our target service (UUID128)
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 &&
            memcmp(p_data->search_res.srvc_id.uuid.uuid.uuid128, service_uuid128, ESP_UUID_LEN_128) == 0)
        {
            ESP_LOGI(GATTC_TAG, "*** TARGET SERVICE FOUND! UUID128: %s ***", SERVICE_UUID);
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }

        ESP_LOGI(GATTC_TAG, "Service search complete");

        if (!get_server)
        {
            ESP_LOGE(GATTC_TAG, "Service UUID %s NOT FOUND! Disconnecting...", SERVICE_UUID);
            esp_ble_gattc_close(gattc_if, p_data->search_cmpl.conn_id);
            break;
        }

        if (get_server)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                gattc_if,
                p_data->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                INVALID_HANDLE,
                &count);

            if (status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            ESP_LOGI(GATTC_TAG, "Found %d characteristics", count);

            if (count > 0)
            {
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }

                // Try to get characteristic by UUID
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    remote_filter_char_uuid,
                    char_elem_result,
                    &count);

                if (status != ESP_GATT_OK)
                {
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error, trying to get all chars");

                    // If specific UUID fails, get all characteristics
                    status = esp_ble_gattc_get_all_char(
                        gattc_if,
                        p_data->search_cmpl.conn_id,
                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                        char_elem_result,
                        &count,
                        0);

                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_all_char error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    // Print all characteristics found
                    ESP_LOGI(GATTC_TAG, "Listing all %d characteristics:", count);
                    for (int i = 0; i < count; i++)
                    {
                        ESP_LOGI(GATTC_TAG, "  Char %d: handle=%d, uuid_len=%d, properties=0x%x",
                                 i, char_elem_result[i].char_handle,
                                 char_elem_result[i].uuid.len,
                                 char_elem_result[i].properties);
                        if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16)
                        {
                            ESP_LOGI(GATTC_TAG, "    UUID16: 0x%04x", char_elem_result[i].uuid.uuid.uuid16);
                        }
                    }
                }

                // Use first characteristic that supports notify
                for (int i = 0; i < count; i++)
                {
                    if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                        ESP_LOGI(GATTC_TAG, "Registering for notify on char handle %d", char_elem_result[i].char_handle);
                        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                          char_elem_result[i].char_handle);
                        break;
                    }
                }

                free(char_elem_result);
                char_elem_result = NULL;
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "No characteristics found");
            }
        }
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Notification register successfully");

            uint16_t count = 0;
            uint16_t notify_en = 1;

            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(
                gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                ESP_GATT_DB_DESCRIPTOR,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                &count);

            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0)
            {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }

                ret_status = esp_ble_gattc_get_descr_by_char_handle(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    p_data->reg_for_notify.handle,
                    notify_descr_uuid,
                    descr_elem_result,
                    &count);

                if (ret_status != ESP_GATT_OK)
                {
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    free(descr_elem_result);
                    descr_elem_result = NULL;
                    break;
                }

                if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 &&
                    descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                {
                    ESP_LOGI(GATTC_TAG, "Writing to CCCD (0x2902) to enable notifications");
                    ret_status = esp_ble_gattc_write_char_descr(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                        descr_elem_result[0].handle,
                        sizeof(notify_en),
                        (uint8_t *)&notify_en,
                        ESP_GATT_WRITE_TYPE_RSP,
                        ESP_GATT_AUTH_REQ_NONE);
                }

                if (ret_status != ESP_GATT_OK)
                {
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }

                free(descr_elem_result);
                descr_elem_result = NULL;
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "Descriptor not found");
            }
        }
        break;

    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "*** Notification received ***");
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "*** Indication received ***");
        }

        // Print raw data
        ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

        // Parse and print SEN55 data
        print_sen55_data(p_data->notify.value, p_data->notify.value_len);
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "CCCD write successful - notifications enabled!");
        ESP_LOGI(GATTC_TAG, "Waiting for data from STM32WB55...\n");
        break;

    case ESP_GATTC_SRVC_CHG_EVT:
        ESP_LOGI(GATTC_TAG, "Service change");
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "Disconnected, reason 0x%02x", p_data->disconnect.reason);
        ESP_LOGI(GATTC_TAG, "Restarting scan...");
        esp_ble_gap_start_scanning(30);
        break;

    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Scan params set, starting scan for 30 seconds...");
        esp_ble_gap_start_scanning(30);
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning started, looking for '%s'", remote_device_name);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data_by_type(
                scan_result->scan_rst.ble_adv,
                scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                ESP_BLE_AD_TYPE_NAME_CMPL,
                &adv_name_len);

            if (adv_name != NULL)
            {
                if (strlen(remote_device_name) == adv_name_len &&
                    strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0)
                {
                    ESP_LOGI(GATTC_TAG, "*** Found device '%s' ***", remote_device_name);
                    ESP_LOGI(GATTC_TAG, "Address: " ESP_BD_ADDR_STR "", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));

                    if (connect == false)
                    {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "Connecting to device...");
                        esp_ble_gap_stop_scanning();

                        esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
                        memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                        creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                        creat_conn_params.is_direct = true;
                        creat_conn_params.is_aux = false;
                        creat_conn_params.phy_mask = 0x0;

                        esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, &creat_conn_params);
                    }
                }
            }
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "Scan complete");
            break;

        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning stopped");
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++)
    {
        if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if)
        {
            if (gl_profile_tab[idx].gattc_cb)
            {
                gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(GATTC_TAG, "\n=== ESP-IDF SEN55 BLE Client ===");
    ESP_LOGI(GATTC_TAG, "Looking for device: %s", remote_device_name);
    ESP_LOGI(GATTC_TAG, "Service UUID: %s", SERVICE_UUID);
    ESP_LOGI(GATTC_TAG, "Char UUID: %s\n", SEN55_CHAR_UUID);

    // Initialize UUID128 structures
    memcpy(remote_filter_service_uuid.uuid.uuid128, service_uuid128, ESP_UUID_LEN_128);
    memcpy(remote_filter_char_uuid.uuid.uuid128, char_uuid128, ESP_UUID_LEN_128);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Enable controller failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Init bluetooth failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Enable bluetooth failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Gap register failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Gattc register failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "Gattc app register failed, error code = %x", ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "Set local MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(GATTC_TAG, "Initialization complete!");
}