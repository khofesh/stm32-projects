/*
 * esp32c3_at_stm32.c
 *
 *  Created on: Sep 24, 2025
 *      Author: controllerstech.com
 */

#include "esp32c3_at_stm32.h"

ESP32C3_ConnectionState ESP_ConnState = ESP32C3_DISCONNECTED;   // Default state
static char esp_rx_buffer[2048];
static ESP32C3_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);
static ESP32C3_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout);

ESP32C3_Status ESP_Init(void)
{
	ESP32C3_Status res;
	USER_LOG("Initializing ESP32C3...");
	HAL_Delay(1000);

//	res = ESP_SendCommand("AT+RST\r\n", "OK", 2000);
//    if (res != ESP32C3_OK){
//    	DEBUG_LOG("Failed to Reset ESP32C3...");
//    	return res;
//    }
//
//    USER_LOG("Waiting 5 Seconds for Reset to Complete...");
//    HAL_Delay(5000);  // wait for reset to complete

    res = ESP_SendCommand("AT\r\n", "OK", 2000);
    if (res != ESP32C3_OK){
    	DEBUG_LOG("ESP32C3 Not Responding...");
    	return res;
    }

    res = ESP_SendCommand("ATE0\r\n", "OK", 2000); // Disable echo
    if (res != ESP32C3_OK){
    	DEBUG_LOG("Disable echo Command Failed...");
    	return res;
    }
    USER_LOG("ESP32C3 Initialized Successfully...");
    return ESP32C3_OK;
}

ESP32C3_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len)
{
    USER_LOG("Setting in Station Mode");
    // Set in Station Mode
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=1\r\n");

    ESP32C3_Status result = ESP_SendCommand(cmd, "OK", 2000); // wait up to 2s
    if (result != ESP32C3_OK)
    {
    	USER_LOG("Station Mode Failed.");
        return result;
    }

    USER_LOG("Connecting to WiFi SSID: %s", ssid);
    // Send join command
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    result = ESP_SendCommand(cmd, "WIFI CONNECTED", 10000); // wait up to 10s
    if (result != ESP32C3_OK)
    {
    	USER_LOG("WiFi connection failed.");
        ESP_ConnState = ESP32C3_NOT_CONNECTED;
        return result;
    }

    USER_LOG("WiFi Connected. Waiting for IP...");
    ESP_ConnState = ESP32C3_CONNECTED_NO_IP;
    // Fetch IP with retries inside ESP_GetIP
    result = ESP_GetIP(ip_buffer, buffer_len);
    if (result != ESP32C3_OK)
    {
    	USER_LOG("Failed to fetch IP. Status=%d", result);
        return result;
    }

    USER_LOG("WiFi + IP ready: %s", ip_buffer);
    return ESP32C3_OK;
}

ESP32C3_ConnectionState ESP_GetConnectionState(void)
{
    return ESP_ConnState;
}

static ESP32C3_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len)
{
	DEBUG_LOG("Fetching IP Address...");

    for (int attempt = 1; attempt <= 3; attempt++)
    {
        ESP32C3_Status result = ESP_SendCommand("AT+CIFSR\r\n", "OK", 5000);
        if (result != ESP32C3_OK)
        {
        	DEBUG_LOG("CIFSR failed on attempt %d", attempt);
            continue;
        }

        char *search = esp_rx_buffer;
        char *last_ip = NULL;

        while ((search = strstr(search, "STAIP,")) != NULL)
        {
            char *ip_start = strstr(search, "STAIP,\"");
            if (ip_start)
            {
                ip_start += 7;
                char *end = strchr(ip_start, '"');
                if (end && ((end - ip_start) < buffer_len))
                {
                    last_ip = ip_start;
                }
            }
            search += 6;
        }

        if (last_ip)
        {
            char *end = strchr(last_ip, '"');
            strncpy(ip_buffer, last_ip, end - last_ip);
            ip_buffer[end - last_ip] = '\0';

            if (strcmp(ip_buffer, "0.0.0.0") == 0)
            {
            	DEBUG_LOG("Attempt %d: IP not ready yet (0.0.0.0). Retrying...", attempt);
                ESP_ConnState = ESP32C3_CONNECTED_NO_IP;
                HAL_Delay(1000);
                continue;
            }

            DEBUG_LOG("Got IP: %s", ip_buffer);
            ESP_ConnState = ESP32C3_CONNECTED_IP;
            return ESP32C3_OK;
        }

        DEBUG_LOG("Attempt %d: Failed to parse STAIP.", attempt);
        HAL_Delay(500);
    }

    DEBUG_LOG("Failed to fetch IP after retries.");
    ESP_ConnState = ESP32C3_CONNECTED_NO_IP;  // still connected, but no IP
    return ESP32C3_ERROR;
}

static ESP32C3_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t tickstart;
    int found = 0;

    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    tickstart = HAL_GetTick();

    if (strlen(cmd) > 0)
    {
        DEBUG_LOG("Sending: %s", cmd);
        if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY) != HAL_OK)
            return ESP32C3_ERROR;
    }

    while ((HAL_GetTick() - tickstart) < timeout && idx < sizeof(esp_rx_buffer) - 1)
    {
        if (HAL_UART_Receive(&ESP_UART, &ch, 1, 10) == HAL_OK)
        {
            esp_rx_buffer[idx++] = ch;
            esp_rx_buffer[idx]   = '\0';

            // check for ACK
            if (!found && strstr(esp_rx_buffer, ack))
            {
                DEBUG_LOG("Matched ACK: %s", ack);
                found = 1; // mark as found but keep reading
            }

            // handle busy response
            if (strstr(esp_rx_buffer, "busy"))
            {
                DEBUG_LOG("ESP is busy... delaying before retry");
                HAL_Delay(1500);
                idx = 0;
                memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
                continue;
            }
        }
    }

    if (found)
    {
        DEBUG_LOG("Full buffer: %s", esp_rx_buffer);
        return ESP32C3_OK;
    }

    if (idx == 0)
        return ESP32C3_NO_RESPONSE;

    DEBUG_LOG("Timeout or no ACK. Buffer: %s", esp_rx_buffer);
    return ESP32C3_TIMEOUT;
}
