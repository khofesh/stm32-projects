/*
 * Low-Power ESP-IDF BLE GATT Client for SEN55 sensor data from STM32WB55
 * Wakes up periodically, connects, reads data, displays on LCD, then sleeps
 *
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
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7735.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "simple_font.h"

#define LCD_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000) // 40MHz
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

// Pin assignments for FireBeetle 2 ESP32-C6 GDI Interface
#define PIN_NUM_MOSI 22     // SPI MOSI (SDA)
#define PIN_NUM_CLK 23      // SPI SCK (SCL)
#define PIN_NUM_CS 1        // SPI CS (Chip Select)
#define PIN_NUM_DC 8        // DC (Data/Command)
#define PIN_NUM_RST 14      // RST (Reset)
#define PIN_NUM_BK_LIGHT 15 // Backlight (BL)

// LCD resolution
#define LCD_H_RES 128
#define LCD_V_RES 160

#define GATTC_TAG "SEN55_LP"

// Sleep configuration
#define SLEEP_TIME_SEC 60 // Sleep for 5 minutes (300 seconds)
#define SLEEP_TIME_US (SLEEP_TIME_SEC * 1000000ULL)

// Connection timeout
#define CONNECT_TIMEOUT_MS 15000 // 15 seconds to connect and read

// Event bits for synchronization
#define DATA_RECEIVED_BIT BIT0
#define DISCONNECT_BIT BIT1

static EventGroupHandle_t event_group;

// Bluetooth address macros
#ifndef ESP_BD_ADDR_STR
#define ESP_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif
#ifndef ESP_BD_ADDR_HEX
#define ESP_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#endif

// SEN55 Service and Characteristic UUIDs (128-bit)
static uint8_t service_uuid128[ESP_UUID_LEN_128] = {
    0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
    0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00};

static uint8_t char_uuid128[ESP_UUID_LEN_128] = {
    0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
    0x41, 0x45, 0x22, 0x8e, 0x42, 0xfe, 0x00, 0x00};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

static const char remote_device_name[] = "sen55CST";
static bool connect = false;
static bool get_server = false;
static bool data_received = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
// static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0}},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0}},
};

// static esp_bt_uuid_t notify_descr_uuid = {
//     .len = ESP_UUID_LEN_16,
//     .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
// };

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

// Store latest sensor data
static sen55_data_t latest_sensor_data = {0};

// RGB565 colors
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F

static esp_lcd_panel_handle_t panel_handle = NULL;
static uint16_t line_buffer[LCD_H_RES];

static void lcd_fill_screen(uint16_t color)
{
    for (int i = 0; i < LCD_H_RES; i++)
    {
        line_buffer[i] = color;
    }
    for (int y = 0; y < LCD_V_RES; y++)
    {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + 1, line_buffer);
    }
}

static void lcd_draw_rect(int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0)
        return;

    for (int i = 0; i < width && i < LCD_H_RES; i++)
    {
        line_buffer[i] = color;
    }
    for (int row = 0; row < height; row++)
    {
        esp_lcd_panel_draw_bitmap(panel_handle, x, y + row, x + width, y + row + 1, line_buffer);
    }
}

static void lcd_draw_char(int x, int y, char c, uint16_t color)
{
    const uint8_t *font_data = get_char_font(c);
    for (int col = 0; col < 6; col++)
    {
        uint8_t column_data = font_data[col];
        for (int row = 0; row < 8; row++)
        {
            if (column_data & (1 << row))
            {
                lcd_draw_rect(x + col, y + row, 1, 1, color);
            }
        }
    }
}

static void lcd_draw_text(int x, int y, const char *text, uint16_t color)
{
    int cursor_x = x;
    while (*text)
    {
        lcd_draw_char(cursor_x, y, *text, color);
        cursor_x += 7;
        text++;
    }
}

static void lcd_display_sen55_data(sen55_data_t *data, bool sleeping)
{
    lcd_fill_screen(COLOR_BLACK);

    char buf[32];
    int y = 2;

    // Title
    lcd_draw_text(2, y, "SEN55 Monitor", COLOR_GREEN);
    y += 12;

    // PM2.5
    lcd_draw_text(2, y, "PM2.5", COLOR_CYAN);
    float pm25 = data->pm2_5 / 10.0;
    snprintf(buf, sizeof(buf), "%.1fug/m3", pm25);
    lcd_draw_text(50, y, buf, COLOR_WHITE);
    y += 10;

    // PM10
    lcd_draw_text(2, y, "PM10", COLOR_CYAN);
    float pm10 = data->pm10 / 10.0;
    snprintf(buf, sizeof(buf), "%.1fug/m3", pm10);
    lcd_draw_text(50, y, buf, COLOR_WHITE);
    y += 10;

    // Temperature
    lcd_draw_text(2, y, "Temp", COLOR_RED);
    if (data->temperature != 0x7fff)
    {
        float temp = data->temperature / 200.0;
        snprintf(buf, sizeof(buf), "%.1fC", temp);
        lcd_draw_text(50, y, buf, COLOR_WHITE);
    }
    else
    {
        lcd_draw_text(50, y, "N/A", COLOR_WHITE);
    }
    y += 10;

    // Humidity
    lcd_draw_text(2, y, "Humid", COLOR_BLUE);
    if (data->humidity != 0x7fff)
    {
        float humid = data->humidity / 100.0;
        snprintf(buf, sizeof(buf), "%.0f%%", humid);
        lcd_draw_text(50, y, buf, COLOR_WHITE);
    }
    else
    {
        lcd_draw_text(50, y, "N/A", COLOR_WHITE);
    }
    y += 10;

    // VOC
    lcd_draw_text(2, y, "VOC", COLOR_YELLOW);
    if (data->vocIndex != 0x7fff)
    {
        float voc = data->vocIndex / 10.0;
        snprintf(buf, sizeof(buf), "%.0f", voc);
        lcd_draw_text(50, y, buf, COLOR_WHITE);
    }
    else
    {
        lcd_draw_text(50, y, "N/A", COLOR_WHITE);
    }
    y += 10;

    // NOx
    lcd_draw_text(2, y, "NOx", COLOR_MAGENTA);
    if (data->noxIndex != 0x7fff)
    {
        float nox = data->noxIndex / 10.0;
        snprintf(buf, sizeof(buf), "%.0f", nox);
        lcd_draw_text(50, y, buf, COLOR_WHITE);
    }
    else
    {
        lcd_draw_text(50, y, "N/A", COLOR_WHITE);
    }
    y += 12;

    // Status at bottom
    if (sleeping)
    {
        snprintf(buf, sizeof(buf), "Sleep %ds", SLEEP_TIME_SEC);
        lcd_draw_text(2, 145, buf, COLOR_MAGENTA);
    }
    else
    {
        lcd_draw_text(2, 145, "Reading...", COLOR_GREEN);
    }
}

static void parse_sen55_data(uint8_t *data, uint16_t len)
{
    if (len < 16)
    {
        ESP_LOGW(GATTC_TAG, "Data too short: %d bytes", len);
        return;
    }

    latest_sensor_data.pm1_0 = data[0] | (data[1] << 8);
    latest_sensor_data.pm2_5 = data[2] | (data[3] << 8);
    latest_sensor_data.pm4_0 = data[4] | (data[5] << 8);
    latest_sensor_data.pm10 = data[6] | (data[7] << 8);
    latest_sensor_data.temperature = (int16_t)(data[8] | (data[9] << 8));
    latest_sensor_data.humidity = (int16_t)(data[10] | (data[11] << 8));
    latest_sensor_data.vocIndex = (int16_t)(data[12] | (data[13] << 8));
    latest_sensor_data.noxIndex = (int16_t)(data[14] | (data[15] << 8));

    ESP_LOGI(GATTC_TAG, "PM2.5: %.1f, PM10: %.1f, Temp: %.1f°C, Humid: %.1f%%",
             latest_sensor_data.pm2_5 / 10.0,
             latest_sensor_data.pm10 / 10.0,
             latest_sensor_data.temperature / 200.0,
             latest_sensor_data.humidity / 100.0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client registered");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Connected to " ESP_BD_ADDR_STR "", ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Open failed");
            xEventGroupSetBits(event_group, DISCONNECT_BIT);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Connection opened, MTU %u", p_data->open.mtu);
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "Service discovery complete");
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU configured: %d", param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128)
        {
            ESP_LOGI(GATTC_TAG, "Service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (get_server)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                gattc_if, p_data->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                INVALID_HANDLE, &count);

            if (status == ESP_GATT_OK && count > 0)
            {
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (char_elem_result)
                {
                    status = esp_ble_gattc_get_char_by_uuid(
                        gattc_if, p_data->search_cmpl.conn_id,
                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                        remote_filter_char_uuid, char_elem_result, &count);

                    if (status == ESP_GATT_OK && count > 0)
                    {
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;

                        // Instead of registering for notify, just READ the value directly
                        ESP_LOGI(GATTC_TAG, "Reading characteristic...");
                        esp_ble_gattc_read_char(gattc_if, p_data->search_cmpl.conn_id,
                                                char_elem_result[0].char_handle, ESP_GATT_AUTH_REQ_NONE);
                    }
                    free(char_elem_result);
                    char_elem_result = NULL;
                }
            }
        }
        break;

    case ESP_GATTC_READ_CHAR_EVT:
        if (p_data->read.status == ESP_GATT_OK)
        {
            ESP_LOGI(GATTC_TAG, "Read successful, %d bytes", p_data->read.value_len);
            ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->read.value, p_data->read.value_len);

            parse_sen55_data(p_data->read.value, p_data->read.value_len);
            data_received = true;
            xEventGroupSetBits(event_group, DATA_RECEIVED_BIT);

            // Disconnect immediately after reading
            esp_ble_gattc_close(gattc_if, p_data->read.conn_id);
        }
        else
        {
            ESP_LOGE(GATTC_TAG, "Read failed, status %d", p_data->read.status);
            xEventGroupSetBits(event_group, DISCONNECT_BIT);
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Disconnected");
        connect = false;
        get_server = false;
        xEventGroupSetBits(event_group, DISCONNECT_BIT);
        break;

    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Starting scan...");
        esp_ble_gap_start_scanning(10); // 10 second scan
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(GATTC_TAG, "Scanning for '%s'", remote_device_name);
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data_by_type(
                scan_result->scan_rst.ble_adv,
                scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

            if (adv_name && strlen(remote_device_name) == adv_name_len &&
                strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0)
            {
                ESP_LOGI(GATTC_TAG, "Found device!");

                if (!connect)
                {
                    connect = true;
                    esp_ble_gap_stop_scanning();

                    esp_ble_gatt_creat_conn_params_t conn_params = {0};
                    memcpy(&conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                    conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                    conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                    conn_params.is_direct = true;

                    esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, &conn_params);
                }
            }
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Scan stopped");
        break;

    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT && param->reg.status == ESP_GATT_OK)
    {
        gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
    }

    for (int idx = 0; idx < PROFILE_NUM; idx++)
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

static void init_lcd(void)
{
    ESP_LOGI(GATTC_TAG, "Initializing LCD...");

    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    gpio_config(&bk_gpio_config);
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle);

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    ESP_LOGI(GATTC_TAG, "LCD initialized");
}

static void enter_deep_sleep(void)
{
    ESP_LOGI(GATTC_TAG, "Entering deep sleep for %d seconds", SLEEP_TIME_SEC);

    // Display sleep message
    lcd_display_sen55_data(&latest_sensor_data, true);
    vTaskDelay(pdMS_TO_TICKS(2000)); // Show for 2 seconds

    // Turn off backlight
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    // Configure wake up timer
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);

    // Enter deep sleep
    esp_deep_sleep_start();
}

void app_main(void)
{
    uint32_t boot_count = 0;
    esp_reset_reason_t reset_reason = esp_reset_reason();

    if (reset_reason == ESP_RST_DEEPSLEEP)
    {
        boot_count++;
        ESP_LOGI(GATTC_TAG, "\n=== Wake from deep sleep (boot #%lu) ===", boot_count);
    }
    else
    {
        ESP_LOGI(GATTC_TAG, "\n=== First boot - Low Power SEN55 Client ===");
        ESP_LOGI(GATTC_TAG, "Sleep interval: %d seconds", SLEEP_TIME_SEC);
    }

    // Initialize LCD
    init_lcd();
    lcd_fill_screen(COLOR_BLACK);
    lcd_draw_text(10, 70, "Starting...", COLOR_GREEN);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create event group for synchronization
    event_group = xEventGroupCreate();

    // Initialize Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Initialize UUIDs
    memcpy(remote_filter_service_uuid.uuid.uuid128, service_uuid128, ESP_UUID_LEN_128);
    memcpy(remote_filter_char_uuid.uuid.uuid128, char_uuid128, ESP_UUID_LEN_128);

    // Register callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_A_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));

    // Display "Scanning" on LCD
    lcd_fill_screen(COLOR_BLACK);
    lcd_draw_text(10, 70, "Scanning...", COLOR_YELLOW);

    // Wait for data or timeout
    EventBits_t bits = xEventGroupWaitBits(
        event_group,
        DATA_RECEIVED_BIT | DISCONNECT_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

    if (bits & DATA_RECEIVED_BIT)
    {
        ESP_LOGI(GATTC_TAG, "Data received successfully!");

        // Display data on LCD
        lcd_display_sen55_data(&latest_sensor_data, false);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Show for 3 seconds
    }
    else
    {
        ESP_LOGW(GATTC_TAG, "Timeout or connection failed");
        lcd_fill_screen(COLOR_BLACK);
        lcd_draw_text(10, 70, "Failed!", COLOR_RED);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Cleanup Bluetooth to save power
    ESP_LOGI(GATTC_TAG, "Shutting down Bluetooth...");
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    // Enter deep sleep
    enter_deep_sleep();
}