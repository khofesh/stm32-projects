/*
 * esp32_at_stm32.h
 *
 *  Created on: Sep 24, 2025
 *      Author: controllerstech.com
 */

#pragma once

#include "main.h"
#include <string.h>
#include <stdio.h>

#define ESP_UART       huart1   // UART connected to ESP32

#define ENABLE_USER_LOG   1
#define ENABLE_DEBUG_LOG  1

extern UART_HandleTypeDef ESP_UART;

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

typedef enum {
    ESP32_OK = 0,
    ESP32_ERROR,
    ESP32_TIMEOUT,
    ESP32_NO_RESPONSE,
    ESP32_NOT_CONNECTED,
    ESP32_CONNECTED_NO_IP,
    ESP32_CONNECTED_IP
} ESP32_Status;

typedef enum {
    ESP32_DISCONNECTED = 0,
    ESP32_CONNECTED
} ESP32_ConnectionState;

extern ESP32_ConnectionState ESP_ConnState;
/* ------------------------------------- */

/* ------------ API FUNCTIONS ---------- */
ESP32_Status ESP_Init(void);
ESP32_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len);
ESP32_ConnectionState ESP_GetConnectionState(void);
ESP32_Status ESP_SendToThingSpeak(const char *apiKey, float val1, float val2, float val3);
// check whether esp32 supports https or not
ESP32_Status ESP_TestSimpleAPI(void);
