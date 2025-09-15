/**
 * https://controllerstech.com/stm32-esp8266-wifi-ip/
 */

#ifndef INC_ESP32_AT_STM32_H_
#define INC_ESP32_AT_STM32_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

#define ESP_UART huart1

#define ENABLE_USER_LOG 1
#define ENABLE_DEBUG_LOG 1

// DMA Buffer sizes for UART4
#define ESP_DMA_RX_BUFFER_SIZE 2048
#define ESP_DMA_TX_BUFFER_SIZE 512

extern UART_HandleTypeDef ESP_UART;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// DMA buffers
extern uint8_t esp_dma_rx_buffer[ESP_DMA_RX_BUFFER_SIZE];
extern uint8_t esp_dma_tx_buffer[ESP_DMA_TX_BUFFER_SIZE];
extern volatile uint16_t esp_dma_rx_head;
extern volatile uint16_t esp_dma_rx_tail;

#if ENABLE_USER_LOG
#define USER_LOG(fmt, ...) printf("[USER] " fmt "\r\n", ##__VA_ARGS__)
#else
#define USER_LOG(fmt, ...)
#endif

#if ENABLE_DEBUG_LOG
#define DEBUG_LOG(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...)
#endif

typedef enum
{
	ESP8266_OK = 0,
	ESP8266_ERROR,
	ESP8266_TIMEOUT,
	ESP8266_NO_RESPONSE,
	ESP8266_NOT_CONNECTED,
	ESP8266_CONNECTED_NO_IP,
	ESP8266_CONNECTED_IP
} ESP8266_Status;

typedef enum
{
	ESP8266_DISCONNECTED = 0,
	ESP8266_CONNECTED
} ESP8266_ConnectionState;

extern ESP8266_ConnectionState ESP_ConnState;

// DMA-specific
void ESP_DMA_Init(void);
void ESP_DMA_StartReceive(void);
uint16_t ESP_DMA_GetReceivedData(uint8_t *buffer, uint16_t max_len);
ESP8266_Status ESP_DMA_SendCommand(const char *cmd, const char *ack, uint32_t timeout);

// MQTT Configuration
#define MQTT_MAX_TOPIC_LEN 128
#define MQTT_MAX_MESSAGE_LEN 512
#define MQTT_MAX_CLIENT_ID_LEN 64

typedef enum
{
    MQTT_DISCONNECTED = 0,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_ERROR,
    MQTT_CONNECTION_REFUSED_PROTOCOL,
    MQTT_CONNECTION_REFUSED_IDENTIFIER,
    MQTT_CONNECTION_REFUSED_SERVER,
    MQTT_CONNECTION_REFUSED_CREDENTIALS,
    MQTT_CONNECTION_REFUSED_UNAUTHORIZED
} MQTT_ConnectionState;

typedef enum
{
    MQTT_QOS_0 = 0,  // At most once
    MQTT_QOS_1 = 1,  // At least once
    MQTT_QOS_2 = 2   // Exactly once
} MQTT_QoS;

typedef struct
{
    char broker[64];
    uint16_t port;
    char client_id[MQTT_MAX_CLIENT_ID_LEN];
    char username[32];
    char password[32];
    uint16_t keepalive;
    uint8_t clean_session;
} MQTT_Config;

typedef struct
{
    char topic[MQTT_MAX_TOPIC_LEN];
    char message[MQTT_MAX_MESSAGE_LEN];
    uint16_t message_len;
    MQTT_QoS qos;
    uint8_t retain;
    uint16_t packet_id;
} MQTT_Message;

extern MQTT_ConnectionState mqtt_state;
extern MQTT_Config mqtt_config;

// Basic ESP8266 functions
ESP8266_Status ESP_Init(void);
ESP8266_Status ESP_DetectBaudRate(void);
ESP8266_Status ESP_TestBasicUART(void);
ESP8266_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len);
ESP8266_ConnectionState ESP_GetConnectionState(void);

// MQTT functions
ESP8266_Status ESP_MQTT_Init(const char *broker, uint16_t port, const char *client_id);
ESP8266_Status ESP_MQTT_SetAuth(const char *username, const char *password);
ESP8266_Status ESP_MQTT_Connect(void);
ESP8266_Status ESP_MQTT_Disconnect(void);
ESP8266_Status ESP_MQTT_Subscribe(const char *topic, MQTT_QoS qos);
ESP8266_Status ESP_MQTT_Unsubscribe(const char *topic);
ESP8266_Status ESP_MQTT_Publish(const char *topic, const char *message, MQTT_QoS qos, uint8_t retain);
ESP8266_Status ESP_MQTT_CheckMessages(MQTT_Message *msg);
MQTT_ConnectionState ESP_MQTT_GetState(void);

// MQTT spec compliance functions
ESP8266_Status ESP_MQTT_ValidateTopic(const char *topic, uint8_t is_subscription);
ESP8266_Status ESP_MQTT_ValidateClientId(const char *client_id);
ESP8266_Status ESP_MQTT_Ping(void);
ESP8266_Status ESP_MQTT_CheckConnection(void);

#endif /* INC_ESP32_AT_STM32_H_ */
