/**
 * @file sscma_stm32l5.h
 * @brief SSCMA (Seeed SenseCraft Model Assistant) driver for STM32L5
 *
 * Ported from Seeed_Arduino_SSCMA library
 * Original: Copyright (c) 2022 Seeed Technology Inc.
 * STM32L5 Port: 2024
 *
 * Supports communication with Grove Vision AI modules via I2C, UART, or SPI
 */

#ifndef SSCMA_STM32L5_H
#define SSCMA_STM32L5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ============================================================================
 * Configuration
 * ============================================================================ */

#ifndef SSCMA_MAX_RX_SIZE
#define SSCMA_MAX_RX_SIZE           (8 * 1024)  /* RX buffer size */
#endif

#ifndef SSCMA_MAX_TX_SIZE
#define SSCMA_MAX_TX_SIZE           (4 * 1024)  /* TX buffer size */
#endif

#ifndef SSCMA_MAX_BOXES
#define SSCMA_MAX_BOXES             16          /* Max detection boxes */
#endif

#ifndef SSCMA_MAX_CLASSES
#define SSCMA_MAX_CLASSES           16          /* Max classification results */
#endif

#ifndef SSCMA_MAX_POINTS
#define SSCMA_MAX_POINTS            32          /* Max point detections */
#endif

#ifndef SSCMA_MAX_KEYPOINTS
#define SSCMA_MAX_KEYPOINTS         8           /* Max keypoint detections */
#endif

#ifndef SSCMA_MAX_KEYPOINTS_POINTS
#define SSCMA_MAX_KEYPOINTS_POINTS  17          /* Max points per keypoint */
#endif

/* ============================================================================
 * Protocol Constants
 * ============================================================================ */

#define SSCMA_I2C_ADDRESS           0x62

#define SSCMA_HEADER_LEN            4
#define SSCMA_MAX_PL_LEN            250
#define SSCMA_MAX_SPI_PL_LEN        4095
#define SSCMA_CHECKSUM_LEN          2
#define SSCMA_PACKET_SIZE           (SSCMA_HEADER_LEN + SSCMA_MAX_PL_LEN + SSCMA_CHECKSUM_LEN)

/* Feature transport commands */
#define FEATURE_TRANSPORT           0x10
#define FEATURE_TRANSPORT_CMD_READ      0x01
#define FEATURE_TRANSPORT_CMD_WRITE     0x02
#define FEATURE_TRANSPORT_CMD_AVAILABLE 0x03
#define FEATURE_TRANSPORT_CMD_START     0x04
#define FEATURE_TRANSPORT_CMD_STOP      0x05
#define FEATURE_TRANSPORT_CMD_RESET     0x06

/* Response delimiters */
#define SSCMA_RESPONSE_PREFIX       "\r{"
#define SSCMA_RESPONSE_SUFFIX       "}\n"
#define SSCMA_RESPONSE_PREFIX_LEN   2
#define SSCMA_RESPONSE_SUFFIX_LEN   2

/* Command format */
#define SSCMA_CMD_PREFIX            "AT+"
#define SSCMA_CMD_SUFFIX            "\r\n"

/* Response types */
#define SSCMA_CMD_TYPE_RESPONSE     0
#define SSCMA_CMD_TYPE_EVENT        1
#define SSCMA_CMD_TYPE_LOG          2

/* ============================================================================
 * Error Codes
 * ============================================================================ */

typedef enum {
    SSCMA_OK            = 0,
    SSCMA_AGAIN         = 1,
    SSCMA_ELOG          = 2,
    SSCMA_ETIMEDOUT     = 3,
    SSCMA_EIO           = 4,
    SSCMA_EINVAL        = 5,
    SSCMA_ENOMEM        = 6,
    SSCMA_EBUSY         = 7,
    SSCMA_ENOTSUP       = 8,
    SSCMA_EPERM         = 9,
    SSCMA_EUNKNOWN      = 10
} sscma_err_t;

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Bounding box detection result
 */
typedef struct sscma_box_s {
    uint16_t x;         /**< X coordinate */
    uint16_t y;         /**< Y coordinate */
    uint16_t w;         /**< Width */
    uint16_t h;         /**< Height */
    uint8_t  score;     /**< Confidence score (0-100) */
    uint8_t  target;    /**< Target class ID */
} sscma_box_t;

/**
 * @brief Classification result
 */
typedef struct sscma_class_s {
    uint8_t target;     /**< Class ID */
    uint8_t score;      /**< Confidence score (0-100) */
} sscma_class_t;

/**
 * @brief Point detection result
 */
typedef struct sscma_point_s {
    uint16_t x;         /**< X coordinate */
    uint16_t y;         /**< Y coordinate */
    uint16_t z;         /**< Z coordinate (optional) */
    uint8_t  score;     /**< Confidence score */
    uint8_t  target;    /**< Target ID */
} sscma_point_t;

/**
 * @brief Keypoint detection result (pose estimation)
 */
typedef struct sscma_keypoints_s {
    sscma_box_t   box;                                  /**< Bounding box */
    sscma_point_t points[SSCMA_MAX_KEYPOINTS_POINTS];   /**< Keypoints */
    uint8_t       num_points;                           /**< Number of valid points */
} sscma_keypoints_t;

/**
 * @brief Performance metrics
 */
typedef struct {
    uint16_t preprocess;    /**< Preprocessing time (ms) */
    uint16_t inference;     /**< Inference time (ms) */
    uint16_t postprocess;   /**< Postprocessing time (ms) */
} sscma_perf_t;

/**
 * @brief WiFi configuration
 */
typedef struct {
    int  status;
    int  security;
    char ssid[64];
    char password[64];
} sscma_wifi_t;

/**
 * @brief WiFi connection status
 */
typedef struct {
    int  status;
    char ipv4[16];
    char ipv6[64];
    char netmask[16];
    char gateway[16];
} sscma_wifi_status_t;

/**
 * @brief MQTT configuration
 */
typedef struct {
    int      status;
    uint16_t port;
    bool     use_ssl;
    char     server[128];
    char     username[128];
    char     password[128];
    char     client_id[128];
} sscma_mqtt_t;

/**
 * @brief MQTT connection status
 */
typedef struct {
    int status;
} sscma_mqtt_status_t;

/**
 * @brief Inference results container
 */
typedef struct {
    sscma_box_t       boxes[SSCMA_MAX_BOXES];
    uint8_t           num_boxes;

    sscma_class_t     classes[SSCMA_MAX_CLASSES];
    uint8_t           num_classes;

    sscma_point_t     points[SSCMA_MAX_POINTS];
    uint8_t           num_points;

    sscma_keypoints_t keypoints[SSCMA_MAX_KEYPOINTS];
    uint8_t           num_keypoints;

    sscma_perf_t      perf;
} sscma_results_t;

/**
 * @brief Interface type enumeration
 */
typedef enum {
    SSCMA_IFACE_NONE = 0,
    SSCMA_IFACE_I2C,
    SSCMA_IFACE_UART,
    SSCMA_IFACE_SPI
} sscma_iface_type_t;

/**
 * @brief Response callback type
 */
typedef void (*sscma_response_cb_t)(const char *resp, size_t len, void *user_data);

/**
 * @brief SSCMA device handle
 */
typedef struct {
    /* Interface selection */
    sscma_iface_type_t iface_type;

    /* I2C interface */
    I2C_HandleTypeDef *hi2c;
    uint16_t           i2c_addr;

    /* UART interface */
    UART_HandleTypeDef *huart;

    /* SPI interface */
    SPI_HandleTypeDef  *hspi;
    GPIO_TypeDef       *cs_port;
    uint16_t            cs_pin;
    GPIO_TypeDef       *sync_port;
    uint16_t            sync_pin;

    /* Reset pin (optional) */
    GPIO_TypeDef       *rst_port;
    uint16_t            rst_pin;

    /* Communication parameters */
    uint32_t wait_delay_ms;
    uint32_t timeout_ms;

    /* Buffers */
    char    *rx_buf;
    uint32_t rx_buf_size;
    uint32_t rx_end;

    char    *tx_buf;
    uint32_t tx_buf_size;

    /* Device info (cached) */
    char id[32];
    char name[32];
    char info[256];

    /* Latest results */
    sscma_results_t results;

    /* Image data (Base64 encoded, if available) */
    char    *image_buf;
    uint32_t image_len;

    /* User callback */
    sscma_response_cb_t response_cb;
    void               *cb_user_data;

} sscma_handle_t;

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

/**
 * @brief Initialize SSCMA handle with default values
 * @param handle Pointer to SSCMA handle
 */
void sscma_init_handle(sscma_handle_t *handle);

/**
 * @brief Initialize SSCMA with I2C interface
 * @param handle Pointer to SSCMA handle
 * @param hi2c Pointer to I2C handle
 * @param address I2C address (default: SSCMA_I2C_ADDRESS)
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_begin_i2c(sscma_handle_t *handle, I2C_HandleTypeDef *hi2c,
                            uint16_t address);

/**
 * @brief Initialize SSCMA with UART interface
 * @param handle Pointer to SSCMA handle
 * @param huart Pointer to UART handle
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_begin_uart(sscma_handle_t *handle, UART_HandleTypeDef *huart);

/**
 * @brief Initialize SSCMA with SPI interface
 * @param handle Pointer to SSCMA handle
 * @param hspi Pointer to SPI handle
 * @param cs_port CS GPIO port
 * @param cs_pin CS GPIO pin
 * @param sync_port SYNC GPIO port (NULL if not used)
 * @param sync_pin SYNC GPIO pin
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_begin_spi(sscma_handle_t *handle, SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_port, uint16_t cs_pin,
                            GPIO_TypeDef *sync_port, uint16_t sync_pin);

/**
 * @brief Configure reset pin
 * @param handle Pointer to SSCMA handle
 * @param rst_port Reset GPIO port
 * @param rst_pin Reset GPIO pin
 */
void sscma_set_reset_pin(sscma_handle_t *handle, GPIO_TypeDef *rst_port,
                         uint16_t rst_pin);

/**
 * @brief Deinitialize and free resources
 * @param handle Pointer to SSCMA handle
 */
void sscma_deinit(sscma_handle_t *handle);

/* ============================================================================
 * Core Communication Functions
 * ============================================================================ */

/**
 * @brief Write data to device
 * @param handle Pointer to SSCMA handle
 * @param data Data buffer
 * @param length Data length
 * @return Number of bytes written, or negative error
 */
int sscma_write(sscma_handle_t *handle, const char *data, int length);

/**
 * @brief Read data from device
 * @param handle Pointer to SSCMA handle
 * @param data Data buffer
 * @param length Maximum bytes to read
 * @return Number of bytes read, or negative error
 */
int sscma_read(sscma_handle_t *handle, char *data, int length);

/**
 * @brief Check available data
 * @param handle Pointer to SSCMA handle
 * @return Number of bytes available
 */
int sscma_available(sscma_handle_t *handle);

/**
 * @brief Reset the device
 * @param handle Pointer to SSCMA handle
 */
void sscma_reset(sscma_handle_t *handle);

/* ============================================================================
 * Inference Functions
 * ============================================================================ */

/**
 * @brief Run inference
 * @param handle Pointer to SSCMA handle
 * @param times Number of inference iterations (use -1 for continuous)
 * @param filter Enable result filtering
 * @param show Enable image capture in response
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_invoke(sscma_handle_t *handle, int times, bool filter, bool show);

/**
 * @brief Get last inference results
 * @param handle Pointer to SSCMA handle
 * @return Pointer to results structure
 */
const sscma_results_t* sscma_get_results(sscma_handle_t *handle);

/**
 * @brief Fetch and process responses (non-blocking)
 * @param handle Pointer to SSCMA handle
 *
 * Call this periodically to process incoming data.
 * Results will be updated in handle->results
 */
void sscma_fetch(sscma_handle_t *handle);

/**
 * @brief Set response callback
 * @param handle Pointer to SSCMA handle
 * @param callback Callback function
 * @param user_data User data passed to callback
 */
void sscma_set_response_callback(sscma_handle_t *handle, sscma_response_cb_t callback,
                                 void *user_data);

/* ============================================================================
 * Device Information Functions
 * ============================================================================ */

/**
 * @brief Get device ID
 * @param handle Pointer to SSCMA handle
 * @param use_cache Use cached value if available
 * @return Pointer to ID string, or NULL on error
 */
const char* sscma_get_id(sscma_handle_t *handle, bool use_cache);

/**
 * @brief Get device name
 * @param handle Pointer to SSCMA handle
 * @param use_cache Use cached value if available
 * @return Pointer to name string, or NULL on error
 */
const char* sscma_get_name(sscma_handle_t *handle, bool use_cache);

/**
 * @brief Get device info
 * @param handle Pointer to SSCMA handle
 * @param use_cache Use cached value if available
 * @return Pointer to info string, or NULL on error
 */
const char* sscma_get_info(sscma_handle_t *handle, bool use_cache);

/* ============================================================================
 * WiFi Functions (for WiFi-enabled modules)
 * ============================================================================ */

/**
 * @brief Get WiFi configuration
 * @param handle Pointer to SSCMA handle
 * @param wifi Pointer to WiFi config structure
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_wifi_get_config(sscma_handle_t *handle, sscma_wifi_t *wifi);

/**
 * @brief Set WiFi status
 * @param handle Pointer to SSCMA handle
 * @param status Pointer to WiFi status structure
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_wifi_set_status(sscma_handle_t *handle, sscma_wifi_status_t *status);

/* ============================================================================
 * MQTT Functions (for MQTT-enabled modules)
 * ============================================================================ */

/**
 * @brief Get MQTT configuration
 * @param handle Pointer to SSCMA handle
 * @param mqtt Pointer to MQTT config structure
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_mqtt_get_config(sscma_handle_t *handle, sscma_mqtt_t *mqtt);

/**
 * @brief Set MQTT status
 * @param handle Pointer to SSCMA handle
 * @param status Pointer to MQTT status structure
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_mqtt_set_status(sscma_handle_t *handle, sscma_mqtt_status_t *status);

/* ============================================================================
 * Action Functions
 * ============================================================================ */

/**
 * @brief Clear all actions
 * @param handle Pointer to SSCMA handle
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_clean_actions(sscma_handle_t *handle);

/**
 * @brief Save JPEG image
 * @param handle Pointer to SSCMA handle
 * @return SSCMA_OK on success
 */
sscma_err_t sscma_save_jpeg(sscma_handle_t *handle);

/* ============================================================================
 * Buffer Management
 * ============================================================================ */

/**
 * @brief Set RX buffer size
 * @param handle Pointer to SSCMA handle
 * @param size Buffer size in bytes
 * @return true on success
 */
bool sscma_set_rx_buffer(sscma_handle_t *handle, uint32_t size);

/**
 * @brief Set TX buffer size
 * @param handle Pointer to SSCMA handle
 * @param size Buffer size in bytes
 * @return true on success
 */
bool sscma_set_tx_buffer(sscma_handle_t *handle, uint32_t size);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Get last captured image (Base64 encoded)
 * @param handle Pointer to SSCMA handle
 * @param length Pointer to store image length
 * @return Pointer to image data, or NULL if not available
 */
const char* sscma_get_last_image(sscma_handle_t *handle, uint32_t *length);

/**
 * @brief Error code to string
 * @param err Error code
 * @return Error description string
 */
const char* sscma_err_to_str(sscma_err_t err);

#ifdef __cplusplus
}
#endif

#endif /* SSCMA_STM32L5_H */
