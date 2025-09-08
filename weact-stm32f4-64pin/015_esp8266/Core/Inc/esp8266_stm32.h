/**
 * https://controllerstech.com/stm32-esp8266-wifi-ip/
 */

#ifndef INC_ESP8266_STM32_H_
#define INC_ESP8266_STM32_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

#define ESP_UART huart4

#define ENABLE_USER_LOG 1
#define ENABLE_DEBUG_LOG 1

// DMA Buffer sizes for UART4
#define ESP_DMA_RX_BUFFER_SIZE 2048
#define ESP_DMA_TX_BUFFER_SIZE 512

extern UART_HandleTypeDef ESP_UART;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

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

ESP8266_Status ESP_Init(void);
ESP8266_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len);
ESP8266_ConnectionState ESP_GetConnectionState(void);

#endif /* INC_ESP8266_STM32_H_ */
