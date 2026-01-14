/**
 * @file sscma_stm32l5.c
 * @brief SSCMA driver implementation for STM32L5
 *
 * Ported from Seeed_Arduino_SSCMA library
 * Original: Copyright (c) 2022 Seeed Technology Inc.
 * STM32L5 Port: 2024
 */

#include "sscma_stm32l5.h"
#include "sscma_parser.h"

/* ============================================================================
 * Private Macros
 * ============================================================================ */

#define SPI_CS_LOW(h)   do { if((h)->cs_port) HAL_GPIO_WritePin((h)->cs_port, (h)->cs_pin, GPIO_PIN_RESET); } while(0)
#define SPI_CS_HIGH(h)  do { if((h)->cs_port) HAL_GPIO_WritePin((h)->cs_port, (h)->cs_pin, GPIO_PIN_SET); } while(0)

#define MIN(a, b)       ((a) < (b) ? (a) : (b))

/* ============================================================================
 * AT Command Strings
 * ============================================================================ */

static const char CMD_AT_ID[]           = "ID?";
static const char CMD_AT_NAME[]         = "NAME?";
static const char CMD_AT_VERSION[]      = "VER?";
static const char CMD_AT_STATS[]        = "STAT";
static const char CMD_AT_BREAK[]        = "BREAK";
static const char CMD_AT_RESET[]        = "RST";
static const char CMD_AT_WIFI[]         = "WIFI";
static const char CMD_AT_WIFI_VER[]     = "WIFIVER";
static const char CMD_AT_WIFI_STA[]     = "WIFISTA";
static const char CMD_AT_WIFI_IN4[]     = "WIFIIN4";
static const char CMD_AT_WIFI_IN6[]     = "WIFIIN6";
static const char CMD_AT_MQTTSERVER[]   = "MQTTSERVER";
static const char CMD_AT_MQTTPUBSUB[]   = "MQTTPUBSUB";
static const char CMD_AT_MQTTSERVER_STA[] = "MQTTSERVERSTA";
static const char CMD_AT_INVOKE[]       = "INVOKE";
static const char CMD_AT_SAMPLE[]       = "SAMPLE";
static const char CMD_AT_INFO[]         = "INFO";
static const char CMD_AT_TSCORE[]       = "TSCORE";
static const char CMD_AT_TIOU[]         = "TIOU";
static const char CMD_AT_ALGOS[]        = "ALGOS";
static const char CMD_AT_MODELS[]       = "MODELS";
static const char CMD_AT_MODEL[]        = "MODEL";
static const char CMD_AT_SENSORS[]      = "SENSORS";
static const char CMD_AT_ACTION[]       = "ACTION";

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

/* I2C functions */
static int i2c_write(sscma_handle_t *handle, const char *data, int length);
static int i2c_read(sscma_handle_t *handle, char *data, int length);
static int i2c_available(sscma_handle_t *handle);
static void i2c_cmd(sscma_handle_t *handle, uint8_t feature, uint8_t cmd,
                    uint16_t len, uint8_t *data);

/* UART functions */
static int uart_write(sscma_handle_t *handle, const char *data, int length);
static int uart_read(sscma_handle_t *handle, char *data, int length);
static int uart_available(sscma_handle_t *handle);

/* SPI functions */
static int spi_write(sscma_handle_t *handle, const char *data, int length);
static int spi_read(sscma_handle_t *handle, char *data, int length);
static int spi_available(sscma_handle_t *handle);
static void spi_cmd(sscma_handle_t *handle, uint8_t feature, uint8_t cmd,
                    uint16_t len, uint8_t *data);

/* Response handling */
static sscma_err_t wait_response(sscma_handle_t *handle, int type, const char *cmd,
                                 uint32_t timeout);
static void parse_event(sscma_handle_t *handle, const char *json, size_t len);
static void parse_invoke_results(sscma_handle_t *handle, const char *json, size_t len);

/* Utility */
static char* strnstr_local(const char *haystack, const char *needle, size_t n);
static void delay_ms(uint32_t ms);

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

void sscma_init_handle(sscma_handle_t *handle)
{
    if (!handle) return;

    memset(handle, 0, sizeof(sscma_handle_t));
    handle->iface_type = SSCMA_IFACE_NONE;
    handle->i2c_addr = SSCMA_I2C_ADDRESS;
    handle->wait_delay_ms = 2;
    handle->timeout_ms = 1000;
}

sscma_err_t sscma_begin_i2c(sscma_handle_t *handle, I2C_HandleTypeDef *hi2c,
                            uint16_t address)
{
    if (!handle || !hi2c) return SSCMA_EINVAL;

    handle->iface_type = SSCMA_IFACE_I2C;
    handle->hi2c = hi2c;
    handle->i2c_addr = address;
    handle->huart = NULL;
    handle->hspi = NULL;

    /* Allocate buffers */
    if (!sscma_set_rx_buffer(handle, SSCMA_MAX_RX_SIZE)) {
        return SSCMA_ENOMEM;
    }
    if (!sscma_set_tx_buffer(handle, SSCMA_MAX_TX_SIZE)) {
        return SSCMA_ENOMEM;
    }

    /* Hardware reset if pin configured */
    if (handle->rst_port) {
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_RESET);
        delay_ms(50);
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_SET);
        delay_ms(500);
    }

    /* Verify device communication */
    if (sscma_get_id(handle, false) == NULL) {
        return SSCMA_EIO;
    }
    if (sscma_get_name(handle, false) == NULL) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

sscma_err_t sscma_begin_uart(sscma_handle_t *handle, UART_HandleTypeDef *huart)
{
    if (!handle || !huart) return SSCMA_EINVAL;

    handle->iface_type = SSCMA_IFACE_UART;
    handle->huart = huart;
    handle->hi2c = NULL;
    handle->hspi = NULL;

    /* Allocate buffers */
    if (!sscma_set_rx_buffer(handle, SSCMA_MAX_RX_SIZE)) {
        return SSCMA_ENOMEM;
    }
    if (!sscma_set_tx_buffer(handle, SSCMA_MAX_TX_SIZE)) {
        return SSCMA_ENOMEM;
    }

    /* Hardware reset if pin configured */
    if (handle->rst_port) {
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_RESET);
        delay_ms(50);
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_SET);
        delay_ms(500);
    }

    /* Verify device communication */
    if (sscma_get_id(handle, false) == NULL) {
        return SSCMA_EIO;
    }
    if (sscma_get_name(handle, false) == NULL) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

sscma_err_t sscma_begin_spi(sscma_handle_t *handle, SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port, uint16_t cs_pin,
                            GPIO_TypeDef *sync_port, uint16_t sync_pin)
{
    if (!handle || !hspi) return SSCMA_EINVAL;

    handle->iface_type = SSCMA_IFACE_SPI;
    handle->hspi = hspi;
    handle->cs_port = cs_port;
    handle->cs_pin = cs_pin;
    handle->sync_port = sync_port;
    handle->sync_pin = sync_pin;
    handle->hi2c = NULL;
    handle->huart = NULL;

    /* Set CS high initially */
    if (cs_port) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }

    /* Allocate buffers */
    if (!sscma_set_rx_buffer(handle, SSCMA_MAX_RX_SIZE)) {
        return SSCMA_ENOMEM;
    }
    if (!sscma_set_tx_buffer(handle, SSCMA_MAX_TX_SIZE)) {
        return SSCMA_ENOMEM;
    }

    /* Hardware reset if pin configured */
    if (handle->rst_port) {
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_RESET);
        delay_ms(50);
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_SET);
        delay_ms(500);
    }

    /* Send reset command */
    spi_cmd(handle, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_RESET, 0, NULL);

    /* Verify device communication */
    if (sscma_get_id(handle, false) == NULL) {
        return SSCMA_EIO;
    }
    if (sscma_get_name(handle, false) == NULL) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

void sscma_set_reset_pin(sscma_handle_t *handle, GPIO_TypeDef *rst_port,
                         uint16_t rst_pin)
{
    if (!handle) return;
    handle->rst_port = rst_port;
    handle->rst_pin = rst_pin;
}

void sscma_deinit(sscma_handle_t *handle)
{
    if (!handle) return;

    if (handle->rx_buf) {
        free(handle->rx_buf);
        handle->rx_buf = NULL;
    }
    if (handle->tx_buf) {
        free(handle->tx_buf);
        handle->tx_buf = NULL;
    }
    if (handle->image_buf) {
        free(handle->image_buf);
        handle->image_buf = NULL;
    }

    handle->iface_type = SSCMA_IFACE_NONE;
}

/* ============================================================================
 * Core Communication Functions
 * ============================================================================ */

int sscma_write(sscma_handle_t *handle, const char *data, int length)
{
    if (!handle || !data || length <= 0) return -1;

    switch (handle->iface_type) {
        case SSCMA_IFACE_I2C:
            return i2c_write(handle, data, length);
        case SSCMA_IFACE_UART:
            return uart_write(handle, data, length);
        case SSCMA_IFACE_SPI:
            return spi_write(handle, data, length);
        default:
            return -1;
    }
}

int sscma_read(sscma_handle_t *handle, char *data, int length)
{
    if (!handle || !data || length <= 0) return -1;

    switch (handle->iface_type) {
        case SSCMA_IFACE_I2C:
            return i2c_read(handle, data, length);
        case SSCMA_IFACE_UART:
            return uart_read(handle, data, length);
        case SSCMA_IFACE_SPI:
            return spi_read(handle, data, length);
        default:
            return -1;
    }
}

int sscma_available(sscma_handle_t *handle)
{
    if (!handle) return 0;

    switch (handle->iface_type) {
        case SSCMA_IFACE_I2C:
            return i2c_available(handle);
        case SSCMA_IFACE_UART:
            return uart_available(handle);
        case SSCMA_IFACE_SPI:
            return spi_available(handle);
        default:
            return 0;
    }
}

void sscma_reset(sscma_handle_t *handle)
{
    if (!handle) return;

    if (handle->rst_port) {
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_RESET);
        delay_ms(50);
        HAL_GPIO_WritePin(handle->rst_port, handle->rst_pin, GPIO_PIN_SET);
        delay_ms(500);
    }

    /* Clear buffers */
    handle->rx_end = 0;
    memset(&handle->results, 0, sizeof(sscma_results_t));
}

/* ============================================================================
 * Inference Functions
 * ============================================================================ */

sscma_err_t sscma_invoke(sscma_handle_t *handle, int times, bool filter, bool show)
{
    char cmd[64];

    if (!handle) return SSCMA_EINVAL;

    /* Check if buffer is large enough for image data */
    if (show && handle->rx_buf_size < 16 * 1024) {
        return SSCMA_ENOTSUP;
    }

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s=%d,%d,%d" SSCMA_CMD_SUFFIX,
             CMD_AT_INVOKE, times, !filter ? 1 : 0, filter ? 1 : 0);

    printf("[TX] cmd='%s'\n", cmd);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_INVOKE,
                      handle->timeout_ms) == SSCMA_OK) {

    	sscma_err_t result = wait_response(handle, SSCMA_CMD_TYPE_EVENT, CMD_AT_INVOKE,
                handle->timeout_ms * 10);  /* Increased timeout for inference */
        if (result == SSCMA_OK) {
            return SSCMA_OK;
        }
        return result;  /* Return actual error instead of generic timeout */
    }

    return SSCMA_ETIMEDOUT;
}

const sscma_results_t* sscma_get_results(sscma_handle_t *handle)
{
    if (!handle) return NULL;
    return &handle->results;
}

void sscma_fetch(sscma_handle_t *handle)
{
    if (!handle) return;

    int len = sscma_available(handle);
    if (len == 0) return;

    /* Ensure we don't overflow buffer */
    if (len + handle->rx_end > handle->rx_buf_size) {
        len = handle->rx_buf_size - handle->rx_end;
        if (len <= 0) {
            handle->rx_end = 0;
            return;
        }
    }

    handle->rx_end += sscma_read(handle, handle->rx_buf + handle->rx_end, len);
    handle->rx_buf[handle->rx_end] = '\0';

    /* Process complete responses */
    char *suffix;
    while ((suffix = strnstr_local(handle->rx_buf, SSCMA_RESPONSE_SUFFIX,
                                   handle->rx_end)) != NULL) {
        char *prefix = strnstr_local(handle->rx_buf, SSCMA_RESPONSE_PREFIX,
                                     suffix - handle->rx_buf);
        if (prefix) {
            /* Extract JSON payload */
            len = suffix - prefix + SSCMA_RESPONSE_SUFFIX_LEN;
            char *payload = (char *)malloc(len + 1);

            if (payload) {
                memcpy(payload, prefix, len);
                payload[len] = '\0';

                /* Invoke callback if set */
                if (handle->response_cb) {
                    handle->response_cb(payload, len, handle->cb_user_data);
                }

                /* Parse the response */
                parse_event(handle, payload + 1, len - 2);  /* Skip \r and \n */

                free(payload);
            }

            /* Remove processed data from buffer */
            memmove(handle->rx_buf, suffix + SSCMA_RESPONSE_SUFFIX_LEN,
                    handle->rx_end - (suffix - handle->rx_buf) - SSCMA_RESPONSE_SUFFIX_LEN);
            handle->rx_end -= suffix - handle->rx_buf + SSCMA_RESPONSE_SUFFIX_LEN;
            handle->rx_buf[handle->rx_end] = '\0';
        } else {
            /* Discard incomplete data */
            memmove(handle->rx_buf, suffix + SSCMA_RESPONSE_PREFIX_LEN,
                    handle->rx_end - (suffix - handle->rx_buf) - SSCMA_RESPONSE_PREFIX_LEN);
            handle->rx_end -= suffix - handle->rx_buf + SSCMA_RESPONSE_PREFIX_LEN;
            handle->rx_buf[handle->rx_end] = '\0';
        }
    }
}

void sscma_set_response_callback(sscma_handle_t *handle, sscma_response_cb_t callback,
                                 void *user_data)
{
    if (!handle) return;
    handle->response_cb = callback;
    handle->cb_user_data = user_data;
}

/* ============================================================================
 * Device Information Functions
 * ============================================================================ */

const char* sscma_get_id(sscma_handle_t *handle, bool use_cache)
{
    char cmd[64];

    if (!handle) return NULL;

    if (use_cache && handle->id[0] != '\0') {
        return handle->id;
    }

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s" SSCMA_CMD_SUFFIX, CMD_AT_ID);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_ID,
                      handle->timeout_ms) == SSCMA_OK) {
        return handle->id;
    }

    return NULL;
}

const char* sscma_get_name(sscma_handle_t *handle, bool use_cache)
{
    char cmd[64];

    if (!handle) return NULL;

    if (use_cache && handle->name[0] != '\0') {
        return handle->name;
    }

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s" SSCMA_CMD_SUFFIX, CMD_AT_NAME);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_NAME,
                      handle->timeout_ms * 3) == SSCMA_OK) {
        return handle->name;
    }

    return NULL;
}

const char* sscma_get_info(sscma_handle_t *handle, bool use_cache)
{
    char cmd[64];

    if (!handle) return NULL;

    if (use_cache && handle->info[0] != '\0') {
        return handle->info;
    }

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s?" SSCMA_CMD_SUFFIX, CMD_AT_INFO);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_INFO,
                      handle->timeout_ms * 3) == SSCMA_OK) {
        return handle->info;
    }

    return NULL;
}

/* ============================================================================
 * WiFi Functions
 * ============================================================================ */

sscma_err_t sscma_wifi_get_config(sscma_handle_t *handle, sscma_wifi_t *wifi)
{
    char cmd[64];

    if (!handle || !wifi) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s?" SSCMA_CMD_SUFFIX, CMD_AT_WIFI);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, "WIFI?",
                      handle->timeout_ms) == SSCMA_OK) {
        /* WiFi config parsed in wait_response via parse_event */
        return SSCMA_OK;
    }

    return SSCMA_ETIMEDOUT;
}

sscma_err_t sscma_wifi_set_status(sscma_handle_t *handle, sscma_wifi_status_t *status)
{
    char cmd[128];

    if (!handle || !status) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s=%d" SSCMA_CMD_SUFFIX,
             CMD_AT_WIFI_STA, status->status);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_WIFI_STA,
                      handle->timeout_ms) == SSCMA_OK) {
        snprintf(cmd, sizeof(cmd),
                 SSCMA_CMD_PREFIX "%s=\"%s\",\"%s\",\"%s\"" SSCMA_CMD_SUFFIX,
                 CMD_AT_WIFI_IN4, status->ipv4, status->netmask, status->gateway);
        sscma_write(handle, cmd, strlen(cmd));

        if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_WIFI_IN4,
                          handle->timeout_ms) == SSCMA_OK) {
            return SSCMA_OK;
        }
    }

    return SSCMA_ETIMEDOUT;
}

/* ============================================================================
 * MQTT Functions
 * ============================================================================ */

sscma_err_t sscma_mqtt_get_config(sscma_handle_t *handle, sscma_mqtt_t *mqtt)
{
    char cmd[64];

    if (!handle || !mqtt) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s?" SSCMA_CMD_SUFFIX, CMD_AT_MQTTSERVER);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, "MQTTSERVER?",
                      handle->timeout_ms) == SSCMA_OK) {
        return SSCMA_OK;
    }

    return SSCMA_ETIMEDOUT;
}

sscma_err_t sscma_mqtt_set_status(sscma_handle_t *handle, sscma_mqtt_status_t *status)
{
    char cmd[64];

    if (!handle || !status) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s=%d" SSCMA_CMD_SUFFIX,
             CMD_AT_MQTTSERVER_STA, status->status);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_MQTTSERVER_STA,
                      handle->timeout_ms) == SSCMA_OK) {
        return SSCMA_OK;
    }

    return SSCMA_ETIMEDOUT;
}

/* ============================================================================
 * Action Functions
 * ============================================================================ */

sscma_err_t sscma_clean_actions(sscma_handle_t *handle)
{
    char cmd[64];

    if (!handle) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s=\"\"" SSCMA_CMD_SUFFIX, CMD_AT_ACTION);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_ACTION,
                      handle->timeout_ms) == SSCMA_OK) {
        return SSCMA_OK;
    }

    return SSCMA_ETIMEDOUT;
}

sscma_err_t sscma_save_jpeg(sscma_handle_t *handle)
{
    char cmd[64];

    if (!handle) return SSCMA_EINVAL;

    snprintf(cmd, sizeof(cmd), SSCMA_CMD_PREFIX "%s=\"save_jpeg()\"" SSCMA_CMD_SUFFIX,
             CMD_AT_ACTION);
    sscma_write(handle, cmd, strlen(cmd));

    if (wait_response(handle, SSCMA_CMD_TYPE_RESPONSE, CMD_AT_ACTION,
                      handle->timeout_ms) == SSCMA_OK) {
        return SSCMA_OK;
    }

    return SSCMA_ETIMEDOUT;
}

/* ============================================================================
 * Buffer Management
 * ============================================================================ */

bool sscma_set_rx_buffer(sscma_handle_t *handle, uint32_t size)
{
    if (!handle || size == 0) return false;

    if (handle->rx_buf_size == 0) {
        handle->rx_buf = (char *)malloc(size);
    } else {
        handle->rx_buf = (char *)realloc(handle->rx_buf, size);
    }

    if (handle->rx_buf) {
        handle->rx_end = 0;
        handle->rx_buf_size = size;
        return true;
    }

    return false;
}

bool sscma_set_tx_buffer(sscma_handle_t *handle, uint32_t size)
{
    if (!handle || size == 0) return false;

    if (handle->tx_buf_size == 0) {
        handle->tx_buf = (char *)malloc(size);
    } else {
        handle->tx_buf = (char *)realloc(handle->tx_buf, size);
    }

    if (handle->tx_buf) {
        handle->tx_buf_size = size;
        return true;
    }

    return false;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

const char* sscma_get_last_image(sscma_handle_t *handle, uint32_t *length)
{
    if (!handle) return NULL;

    if (length) {
        *length = handle->image_len;
    }

    return handle->image_buf;
}

const char* sscma_err_to_str(sscma_err_t err)
{
    switch (err) {
        case SSCMA_OK:          return "OK";
        case SSCMA_AGAIN:       return "Try again";
        case SSCMA_ELOG:        return "Log error";
        case SSCMA_ETIMEDOUT:   return "Timeout";
        case SSCMA_EIO:         return "I/O error";
        case SSCMA_EINVAL:      return "Invalid argument";
        case SSCMA_ENOMEM:      return "Out of memory";
        case SSCMA_EBUSY:       return "Device busy";
        case SSCMA_ENOTSUP:     return "Not supported";
        case SSCMA_EPERM:       return "Permission denied";
        case SSCMA_EUNKNOWN:    return "Unknown error";
        default:                return "Invalid error code";
    }
}

/* ============================================================================
 * I2C Implementation
 * ============================================================================ */

static void i2c_cmd(sscma_handle_t *handle, uint8_t feature, uint8_t cmd,
                    uint16_t len, uint8_t *data)
{
    uint8_t header[6];

    delay_ms(handle->wait_delay_ms);

    header[0] = feature;
    header[1] = cmd;
    header[2] = (len >> 8) & 0xFF;
    header[3] = len & 0xFF;
    header[4] = 0;  /* Checksum placeholder */
    header[5] = 0;

    HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, header,
                            SSCMA_HEADER_LEN, HAL_MAX_DELAY);

    if (data && len > 0) {
        HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, data,
                                len, HAL_MAX_DELAY);
    }

    /* Send checksum */
    HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, &header[4],
                            SSCMA_CHECKSUM_LEN, HAL_MAX_DELAY);
}

static int i2c_available(sscma_handle_t *handle)
{
    uint8_t buf[6];
    uint8_t resp[2] = {0};
    HAL_StatusTypeDef status;

    delay_ms(handle->wait_delay_ms);

    buf[0] = FEATURE_TRANSPORT;
    buf[1] = FEATURE_TRANSPORT_CMD_AVAILABLE;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;

    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, buf,
                                6, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        delay_ms(handle->wait_delay_ms);
        status = HAL_I2C_Master_Receive(handle->hi2c, handle->i2c_addr << 1, resp,
                               2, HAL_MAX_DELAY);
    }

    int available = (resp[0] << 8) | resp[1];
    /* Debug: only print when data is available or on error */
    if (available > 0 || status != HAL_OK) {
        printf("[I2C] available=%d, status=%d\n", available, status);
    }
    return available;
}

static int i2c_read(sscma_handle_t *handle, char *data, int length)
{
    uint16_t packets = length / SSCMA_MAX_PL_LEN;
    uint8_t remain = length % SSCMA_MAX_PL_LEN;
    uint8_t buf[6];

    for (uint16_t i = 0; i < packets; i++) {
        delay_ms(handle->wait_delay_ms);

        buf[0] = FEATURE_TRANSPORT;
        buf[1] = FEATURE_TRANSPORT_CMD_READ;
        buf[2] = (SSCMA_MAX_PL_LEN >> 8) & 0xFF;
        buf[3] = SSCMA_MAX_PL_LEN & 0xFF;
        buf[4] = 0;
        buf[5] = 0;

        if (HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, buf,
                                    6, HAL_MAX_DELAY) == HAL_OK) {
            delay_ms(handle->wait_delay_ms);
            HAL_I2C_Master_Receive(handle->hi2c, handle->i2c_addr << 1,
                                   (uint8_t *)(data + i * SSCMA_MAX_PL_LEN),
                                   SSCMA_MAX_PL_LEN, HAL_MAX_DELAY);
        }
    }

    if (remain) {
        delay_ms(handle->wait_delay_ms);

        buf[0] = FEATURE_TRANSPORT;
        buf[1] = FEATURE_TRANSPORT_CMD_READ;
        buf[2] = (remain >> 8) & 0xFF;
        buf[3] = remain & 0xFF;
        buf[4] = 0;
        buf[5] = 0;

        if (HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, buf,
                                    6, HAL_MAX_DELAY) == HAL_OK) {
            delay_ms(handle->wait_delay_ms);
            HAL_I2C_Master_Receive(handle->hi2c, handle->i2c_addr << 1,
                                   (uint8_t *)(data + packets * SSCMA_MAX_PL_LEN),
                                   remain, HAL_MAX_DELAY);
        }
    }

    return length;
}

static int i2c_write(sscma_handle_t *handle, const char *data, int length)
{
    uint16_t packets = length / SSCMA_MAX_PL_LEN;
    uint16_t remain = length % SSCMA_MAX_PL_LEN;
    uint8_t buf[6 + SSCMA_MAX_PL_LEN];

    for (uint16_t i = 0; i < packets; i++) {
        delay_ms(handle->wait_delay_ms);

        buf[0] = FEATURE_TRANSPORT;
        buf[1] = FEATURE_TRANSPORT_CMD_WRITE;
        buf[2] = (SSCMA_MAX_PL_LEN >> 8) & 0xFF;
        buf[3] = SSCMA_MAX_PL_LEN & 0xFF;
        memcpy(&buf[4], data + i * SSCMA_MAX_PL_LEN, SSCMA_MAX_PL_LEN);
        buf[4 + SSCMA_MAX_PL_LEN] = 0;
        buf[5 + SSCMA_MAX_PL_LEN] = 0;

        HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, buf,
                                6 + SSCMA_MAX_PL_LEN, HAL_MAX_DELAY);
    }

    if (remain) {
        delay_ms(handle->wait_delay_ms);

        buf[0] = FEATURE_TRANSPORT;
        buf[1] = FEATURE_TRANSPORT_CMD_WRITE;
        buf[2] = (remain >> 8) & 0xFF;
        buf[3] = remain & 0xFF;
        memcpy(&buf[4], data + packets * SSCMA_MAX_PL_LEN, remain);
        buf[4 + remain] = 0;
        buf[5 + remain] = 0;

        HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_addr << 1, buf,
                                6 + remain, HAL_MAX_DELAY);
    }

    return length;
}

/* ============================================================================
 * UART Implementation
 * ============================================================================ */

static int uart_write(sscma_handle_t *handle, const char *data, int length)
{
    if (HAL_UART_Transmit(handle->huart, (uint8_t *)data, length,
                          HAL_MAX_DELAY) == HAL_OK) {
        return length;
    }
    return -1;
}

static int uart_read(sscma_handle_t *handle, char *data, int length)
{
    /* Note: For production use, implement interrupt-based or DMA reception */
    /* This is a simple polling implementation */
    if (HAL_UART_Receive(handle->huart, (uint8_t *)data, length,
                         100) == HAL_OK) {
        return length;
    }
    return 0;
}

static int uart_available(sscma_handle_t *handle)
{
    /* Check UART RX buffer */
    /* Note: For a proper implementation, use interrupt-based reception
     * with a ring buffer. This is a simplified version. */
    if (__HAL_UART_GET_FLAG(handle->huart, UART_FLAG_RXNE)) {
        return 1;
    }
    return 0;
}

/* ============================================================================
 * SPI Implementation
 * ============================================================================ */

static void spi_cmd(sscma_handle_t *handle, uint8_t feature, uint8_t cmd,
                    uint16_t len, uint8_t *data)
{
    delay_ms(handle->wait_delay_ms);

    handle->tx_buf[0] = feature;
    handle->tx_buf[1] = cmd;
    handle->tx_buf[2] = (len >> 8) & 0xFF;
    handle->tx_buf[3] = len & 0xFF;

    if (data && len > 0) {
        memcpy(&handle->tx_buf[4], data, len);
        handle->tx_buf[4 + len] = 0xFF;
        handle->tx_buf[5 + len] = 0xFF;
    } else {
        handle->tx_buf[4] = 0xFF;
        handle->tx_buf[5] = 0xFF;
    }

    SPI_CS_LOW(handle);
    HAL_SPI_Transmit(handle->hspi, (uint8_t *)handle->tx_buf, SSCMA_PACKET_SIZE,
                     HAL_MAX_DELAY);
    SPI_CS_HIGH(handle);

    delay_ms(handle->wait_delay_ms);
}

static int spi_available(sscma_handle_t *handle)
{
    uint8_t rx_buf[2];
    uint32_t size;

    /* Check sync pin if available */
    if (handle->sync_port) {
        if (HAL_GPIO_ReadPin(handle->sync_port, handle->sync_pin) == GPIO_PIN_RESET) {
            return 0;
        }
    }

    spi_cmd(handle, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_AVAILABLE, 0, NULL);

    SPI_CS_LOW(handle);
    HAL_SPI_Receive(handle->hspi, rx_buf, 2, HAL_MAX_DELAY);
    SPI_CS_HIGH(handle);

    size = (rx_buf[0] << 8) | rx_buf[1];

    delay_ms(handle->wait_delay_ms);

    return size;
}

static int spi_read(sscma_handle_t *handle, char *data, int length)
{
    int recv_len = 0;
    int pl_len;

    while (recv_len < length) {
        /* Check sync pin if available */
        if (handle->sync_port) {
            if (HAL_GPIO_ReadPin(handle->sync_port, handle->sync_pin) == GPIO_PIN_RESET) {
                return recv_len;
            }
        }

        pl_len = length - recv_len;
        pl_len = MIN(pl_len, SSCMA_MAX_SPI_PL_LEN);

        spi_cmd(handle, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_READ, pl_len, NULL);

        SPI_CS_LOW(handle);
        HAL_SPI_Receive(handle->hspi, (uint8_t *)(data + recv_len), pl_len,
                        HAL_MAX_DELAY);
        SPI_CS_HIGH(handle);

        recv_len += pl_len;
    }

    return recv_len;
}

static int spi_write(sscma_handle_t *handle, const char *data, int length)
{
    uint16_t packets = length / SSCMA_MAX_PL_LEN;
    uint16_t remain = length % SSCMA_MAX_PL_LEN;

    for (uint16_t i = 0; i < packets; i++) {
        spi_cmd(handle, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_WRITE,
                SSCMA_MAX_PL_LEN, (uint8_t *)(data + i * SSCMA_MAX_PL_LEN));
    }

    if (remain) {
        spi_cmd(handle, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_WRITE,
                remain, (uint8_t *)(data + packets * SSCMA_MAX_PL_LEN));
    }

    return length;
}

/* ============================================================================
 * Response Handling
 * ============================================================================ */

static sscma_err_t wait_response(sscma_handle_t *handle, int type, const char *cmd,
                                 uint32_t timeout)
{
    sscma_err_t ret = SSCMA_OK;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_print = 0;

    printf("[WAIT] type=%d, cmd=%s, timeout=%lu\n", type, cmd, timeout);

    while ((HAL_GetTick() - start_tick) <= timeout) {
        int len = sscma_available(handle);
        if (len == 0) {
            /* Print status every 1 second */
            if ((HAL_GetTick() - last_print) >= 1000) {
                printf("[WAIT] polling... elapsed=%lu ms\n", HAL_GetTick() - start_tick);
                last_print = HAL_GetTick();
            }
            delay_ms(10);  /* Small delay to avoid overwhelming I2C bus */
            continue;
        }

        /* Ensure we don't overflow buffer */
        if (len + handle->rx_end > handle->rx_buf_size) {
            len = handle->rx_buf_size - handle->rx_end;
            if (len <= 0) {
                handle->rx_end = 0;
                continue;
            }
        }

        handle->rx_end += sscma_read(handle, handle->rx_buf + handle->rx_end, len);
        handle->rx_buf[handle->rx_end] = '\0';

        printf("[RX] len=%d, buffer='%.*s'\n", len, (int)handle->rx_end, handle->rx_buf);

        /* Process complete responses */
        char *suffix;
        while ((suffix = strnstr_local(handle->rx_buf, SSCMA_RESPONSE_SUFFIX,
                                       handle->rx_end)) != NULL) {
            char *prefix = strnstr_local(handle->rx_buf, SSCMA_RESPONSE_PREFIX,
                                         suffix - handle->rx_buf);
            if (prefix) {
                /* Extract JSON payload */
                len = suffix - prefix + SSCMA_RESPONSE_SUFFIX_LEN;
                char *payload = (char *)malloc(len + 1);

                if (!payload) continue;

                memcpy(payload, prefix + 1, len - 1);  /* Skip \r */
                payload[len - 1] = '\0';

                /* Remove processed data from buffer */
                memmove(handle->rx_buf, suffix + SSCMA_RESPONSE_SUFFIX_LEN,
                        handle->rx_end - (suffix - handle->rx_buf) - SSCMA_RESPONSE_SUFFIX_LEN);
                handle->rx_end -= suffix - handle->rx_buf + SSCMA_RESPONSE_SUFFIX_LEN;

                /* Parse JSON response */
                int resp_type = -1;
                char resp_name[32] = {0};
                int resp_code = 0;

                sscma_parse_response(payload, strlen(payload), &resp_type, resp_name,
                                     sizeof(resp_name), &resp_code);

                if (resp_type == SSCMA_CMD_TYPE_EVENT) {
                    parse_invoke_results(handle, payload, strlen(payload));
                }

                ret = (sscma_err_t)resp_code;

                /* Check if this is the response we're waiting for */
                if (resp_type == type && strncmp(resp_name, cmd, strlen(cmd)) == 0) {
                    /* Parse additional data based on command */
                    if (strcmp(cmd, CMD_AT_ID) == 0) {
                        sscma_parse_get_string(payload, strlen(payload), "data",
                                               handle->id, sizeof(handle->id));
                    } else if (strcmp(cmd, CMD_AT_NAME) == 0) {
                        sscma_parse_get_string(payload, strlen(payload), "data",
                                               handle->name, sizeof(handle->name));
                    } else if (strcmp(cmd, CMD_AT_INFO) == 0) {
                        sscma_parse_get_data_string(payload, strlen(payload), "info",
                                                    handle->info, sizeof(handle->info));
                    }

                    free(payload);
                    return ret;
                }

                free(payload);
            } else {
                /* Discard incomplete data */
                memmove(handle->rx_buf, suffix + SSCMA_RESPONSE_PREFIX_LEN,
                        handle->rx_end - (suffix - handle->rx_buf) - SSCMA_RESPONSE_PREFIX_LEN);
                handle->rx_end -= suffix - handle->rx_buf + SSCMA_RESPONSE_PREFIX_LEN;
                handle->rx_buf[handle->rx_end] = '\0';
            }
        }
    }

    return SSCMA_ETIMEDOUT;
}

static void parse_event(sscma_handle_t *handle, const char *json, size_t len)
{
    int type = -1;
    char name[32] = {0};
    int code = 0;

    sscma_parse_response(json, len, &type, name, sizeof(name), &code);

    if (type == SSCMA_CMD_TYPE_EVENT) {
        if (strstr(name, CMD_AT_INVOKE) != NULL) {
            parse_invoke_results(handle, json, strlen(json));
        }
    }
}

static void parse_invoke_results(sscma_handle_t *handle, const char *json, size_t len)
{
    /* Parse performance data */
    sscma_parse_get_perf(json, len,
                         &handle->results.perf.preprocess,
                         &handle->results.perf.inference,
                         &handle->results.perf.postprocess);

    /* Parse boxes */
    int num = sscma_parse_get_boxes(json, len, handle->results.boxes, SSCMA_MAX_BOXES);
    handle->results.num_boxes = (num > 0) ? (uint8_t)num : 0;

    /* Parse classes */
    num = sscma_parse_get_classes(json, len, handle->results.classes, SSCMA_MAX_CLASSES);
    handle->results.num_classes = (num > 0) ? (uint8_t)num : 0;

    /* Parse points */
    num = sscma_parse_get_points(json, len, handle->results.points, SSCMA_MAX_POINTS);
    handle->results.num_points = (num > 0) ? (uint8_t)num : 0;

    /* Parse keypoints */
    num = sscma_parse_get_keypoints(json, len, handle->results.keypoints,
                                     SSCMA_MAX_KEYPOINTS, SSCMA_MAX_KEYPOINTS_POINTS);
    handle->results.num_keypoints = (num > 0) ? (uint8_t)num : 0;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

static char* strnstr_local(const char *haystack, const char *needle, size_t n)
{
    if (!needle || !haystack || n == 0) {
        return NULL;
    }

    size_t needle_len = strlen(needle);

    if (needle_len == 0) {
        return (char *)haystack;
    }

    for (size_t i = 0; i <= n - needle_len && haystack[i] != '\0'; i++) {
        if (haystack[i] == needle[0]) {
            size_t j = 1;
            while (j < needle_len && haystack[i + j] == needle[j]) {
                j++;
            }
            if (j == needle_len) {
                return (char *)&haystack[i];
            }
        }
    }

    return NULL;
}

static void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
