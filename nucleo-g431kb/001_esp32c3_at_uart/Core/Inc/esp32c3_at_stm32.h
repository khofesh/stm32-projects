/*
 * esp32c3_at_stm32.h
 *
 *  Created on: Sep 24, 2025
 *      Author: controllerstech.com
 *
 *
 */

#ifndef INC_ESP32C3_AT_STM32_H_
#define INC_ESP32C3_AT_STM32_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

#define ESP_UART 	huart1

// log
#define ENABLE_USER_LOG   1
#define ENABLE_DEBUG_LOG  1

extern UART_HandleTypeDef ESP_UART;

/* ------------ LOG MACROS ------------- */
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
/* ------------------------------------- */

typedef enum {
    ESP32C3_OK = 0,
    ESP32C3_ERROR,
    ESP32C3_TIMEOUT,
    ESP32C3_NO_RESPONSE,
    ESP32C3_NOT_CONNECTED,
    ESP32C3_CONNECTED_NO_IP,
    ESP32C3_CONNECTED_IP
} ESP32C3_Status;

typedef enum {
    ESP32C3_DISCONNECTED = 0,
    ESP32C3_CONNECTED
} ESP32C3_ConnectionState;

extern ESP32C3_ConnectionState ESP_ConnState;

ESP32C3_Status ESP_Init(void);
ESP32C3_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len);
ESP32C3_ConnectionState ESP_GetConnectionState(void);


#endif /* INC_ESP32C3_AT_STM32_H_ */
