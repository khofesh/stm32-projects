/*
 * esp32_at_stm32.c
 *
 *  Created on: Sep 24, 2025
 *      Author: controllerstech.com
 */

#include "esp32_at_stm32.h"
#include <stdlib.h>


ESP32_ConnectionState ESP_ConnState = ESP32_DISCONNECTED;   // Default state

static char esp_rx_buffer[2048];
static ESP32_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);
static ESP32_Status ESP_SendCommand(char *cmd, const char *ack, uint32_t timeout);
static ESP32_Status ESP_SendBinary(uint8_t *bin, size_t len, const char *ack, uint32_t timeout);
static int MQTT_BuildConnect(uint8_t *packet, const char *clientID, const char *username,
		const char *password, uint16_t keepalive);


ESP32_Status ESP_Init(void)
{
	ESP32_Status res;
	USER_LOG("Initializing ESP32...");
	HAL_Delay(1000);

	//	res = ESP_SendCommand("AT+RST\r\n", "OK", 2000);
	//    if (res != ESP32_OK){
	//    	DEBUG_LOG("Failed to Reset ESP32...");
	//    	return res;
	//    }
	//
	//    USER_LOG("Waiting 5 Seconds for Reset to Complete...");
	//    HAL_Delay(5000);  // wait for reset to complete

	res = ESP_SendCommand("AT\r\n", "OK", 2000);
	if (res != ESP32_OK){
		DEBUG_LOG("ESP32 Not Responding...");
		return res;
	}

	res = ESP_SendCommand("ATE0\r\n", "OK", 2000); // Disable echo
	if (res != ESP32_OK){
		DEBUG_LOG("Disable echo Command Failed...");
		return res;
	}
	USER_LOG("ESP32 Initialized Successfully...");
	return ESP32_OK;
}

ESP32_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len)
{
	USER_LOG("Setting in Station Mode");
	// Set in Station Mode
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "AT+CWMODE=1\r\n");

	ESP32_Status result = ESP_SendCommand(cmd, "OK", 2000); // wait up to 2s
	if (result != ESP32_OK)
	{
		USER_LOG("Station Mode Failed.");
		return result;
	}

	USER_LOG("Connecting to WiFi SSID: %s", ssid);
	// Send join command
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

	result = ESP_SendCommand(cmd, "WIFI CONNECTED", 10000); // wait up to 10s
	if (result != ESP32_OK)
	{
		USER_LOG("WiFi connection failed.");
		ESP_ConnState = ESP32_DISCONNECTED;
		return result;
	}

	USER_LOG("WiFi Connected. Waiting for IP...");
	ESP_ConnState = ESP32_CONNECTED_NO_IP;
	// Fetch IP with retries inside ESP_GetIP
	result = ESP_GetIP(ip_buffer, buffer_len);
	if (result != ESP32_OK)
	{
		USER_LOG("Failed to fetch IP. Status=%d", result);
		return result;
	}

	USER_LOG("WiFi + IP ready: %s", ip_buffer);
	return ESP32_OK;
}


ESP32_ConnectionState ESP_GetConnectionState(void)
{
	return ESP_ConnState;
}

/* ===================== ThingSpeak IMPLEMENTATION ===================== */

ESP32_Status ESP_SendToThingSpeak(const char *apiKey, float val1, float val2, float val3)
{
	char cmd[256];
	ESP32_Status result;

	USER_LOG("Connecting to ThingSpeak...");

	// 1. Start TCP connection (HTTP port 80)
	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
	result = ESP_SendCommand(cmd, "CONNECT", 5000);
	if (result != ESP32_OK)
	{
		USER_LOG("TCP connection to ThingSpeak failed.");
		return result;
	}

	// 2. Build HTTP GET request
	char httpReq[256];
	snprintf(httpReq, sizeof(httpReq),
			"GET /update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f\r\n", apiKey, val1, val2, val3);

	// 3. Tell ESP how many bytes we will send
	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", (int)strlen(httpReq));
	result = ESP_SendCommand(cmd, ">", 2000);
	if (result != ESP32_OK)
	{
		USER_LOG("CIPSEND failed.");
		return result;
	}

	// 4. Send actual request and wait for ThingSpeak response
	result = ESP_SendCommand(httpReq, "SEND OK", 5000);
	if (result != ESP32_OK)
	{
		USER_LOG("Failed to send HTTP request.");
		return result;
	}

	// 5. Parse ThingSpeak reply in esp_rx_buffer
	char *ipd = strstr(esp_rx_buffer, "+IPD,");
	if (ipd)
	{
		char *colon = strchr(ipd, ':');
		if (colon)
		{
			int entryId = atoi(colon + 1);  // convert server reply to int
			USER_LOG("ThingSpeak entry ID: %d", entryId);

			if (entryId > 0)
			{
				USER_LOG("Update successful!");
				return ESP32_OK;
			}
			else
			{
				USER_LOG("ThingSpeak returned invalid entry ID.");
				return ESP32_ERROR;
			}
		}
	}

	USER_LOG("No valid ThingSpeak response found.");
	return ESP32_ERROR;
}

ESP32_Status ESP_TestSimpleAPI(void)
{
	char cmd[256];
	ESP32_Status result;

	// Try httpbin.org
	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"SSL\",\"httpbin.org\",443\r\n");
	result = ESP_SendCommand(cmd, "CONNECT", 10000);
	if (result != ESP32_OK) return result;

	char httpReq[] = "GET /json HTTP/1.0\r\nHost: httpbin.org\r\n\r\n";

	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", (int)strlen(httpReq));
	result = ESP_SendCommand(cmd, ">", 5000);
	if (result != ESP32_OK) return result;

	result = ESP_SendCommand(httpReq, "SEND OK", 8000);
	if (result == ESP32_OK)
	{
		USER_LOG("Simple API test SUCCESS!");
		HAL_Delay(2000);
		USER_LOG("Response: %s", esp_rx_buffer);
	}

	return result;
}

/* ===================== MQTT IMPLEMENTATION ===================== */

ESP32_Status ESP_MQTT_Connect(const char *broker, uint16_t port, const char *clientID,
		const char *username, const char *password, uint16_t keepalive)
{
	char cmd[64];
	uint8_t packet[256];
	int len = 0;
	ESP32_Status res;

	USER_LOG("Connecting to MQTT broker %s:%d", broker, port);

	/****** Step 1: TCP connect ******/

	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", broker, port);
	res = ESP_SendCommand(cmd, "CONNECT", 5000);
	if (res != ESP32_OK)
	{
		DEBUG_LOG("CIPSTART Failed with ESP ERROR %d..", res);
		return res;
	}

	/****** Step 2: Build MQTT CONNECT packet ******/

	len = MQTT_BuildConnect(packet, clientID, username, password, keepalive);

	/****** Step 3: Tell ESP how many bytes to send ******/

	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
	res = ESP_SendCommand(cmd, ">", 2000);
	if (res != ESP32_OK)
	{
		DEBUG_LOG("CIPSEND Failed with ESP ERROR %d..", res);
		return res;
	}

	/****** Step 4: Send packet and wait for CONNACK ******/

	res = ESP_SendBinary(packet, len, "\x20", 5000);
	if (res != ESP32_OK)
	{
		DEBUG_LOG("Send Connect Command Failed with ESP ERROR %d..", res);
		USER_LOG("MQTT CONNACK failed.");
		return res;
	}

	USER_LOG("MQTT CONNACK received, broker accepted connection.");
	return ESP32_OK;
}

ESP32_Status ESP_MQTT_Publish(const char *topic, const char *message, uint8_t qos)
{
	char cmd[64];
	uint8_t packet[256];
	int len = 0;
	ESP32_Status res;

	USER_LOG("Publishing to MQTT Topic:message %s:%s", topic, message);

	/****** Step 1: Build MQTT Publish packet ******/

	/* Fixed Header */
	packet[len++] = 0x30 | (qos << 1); // PUBLISH, QoS
	int remLenPos = len++;  // will be calculated later

	/* Variable Header */
	// Topic
	uint16_t tlen = strlen(topic);
	packet[len++] = tlen >> 8;  // store topic len
	packet[len++] = tlen & 0xFF;  // store topic len
	memcpy(&packet[len], topic, tlen); len += tlen;  // store topic

	/* Payload */
	// Message
	uint16_t mlen = strlen(message);
	memcpy(&packet[len], message, mlen); len += mlen;  // store message

	// Remaining length
	packet[remLenPos] = len - 2;  // remove first 2 bytes of Fixed Header

	/****** Step 2: Tell ESP how many bytes to send  ******/

	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
	res = ESP_SendCommand(cmd, ">", 2000);
	if (res != ESP32_OK)
	{
		DEBUG_LOG("CIPSEND Failed with ESP ERROR %d..", res);
		return res;
	}

	/****** Step 3: Send packet and wait for ACK ******/

	res = ESP_SendBinary(packet, len, "SEND OK", 5000);
	if (res != ESP32_OK)
	{
		DEBUG_LOG("Publish Command Failed with ESP ERROR %d..", res);
		return res;
	}

	USER_LOG ("Successfully Published to Broker..");
	return ESP32_OK;
}

ESP32_Status ESP_MQTT_Ping(void)
{
	char cmd[32];
	ESP32_Status res;
	uint8_t packet[2];

	USER_LOG("Sending PINGREQ");

	/****** Step 1: Build MQTT Publish packet ******/
	/* Fixed Header */
	packet[0] = 0xC0; 	packet[1] = 0x00;  // PINGREQ

	/****** Step 2: Tell ESP how many bytes to send  ******/
	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=2\r\n");
	res = ESP_SendCommand(cmd, ">", 2000);
	if (res != ESP32_OK){
		DEBUG_LOG("CIPSEND Failed with ESP ERROR %d..", res);
		return res;
	}

	/****** Step 3: Send packet and wait for PINGRESP (0xD0 0x00) ******/
	res = ESP_SendBinary(packet, 2, "\xD0", 2000);
	if (res != ESP32_OK){
		DEBUG_LOG("Publish Command Failed with ESP ERROR %d..", res);
		return res;
	}

	USER_LOG("PINGREQ Sucessful");
	return ESP32_OK;
}

ESP32_Status ESP_CheckTCPConnection(void)
{
	return ESP_SendCommand("AT+CIPSTATUS\r\n", "STATUS:3", 2000);
}

/* ===================== Static Functions ===================== */

static ESP32_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len)
{
	DEBUG_LOG("Fetching IP Address...");

	for (int attempt = 1; attempt <= 3; attempt++)
	{
		ESP32_Status result = ESP_SendCommand("AT+CIFSR\r\n", "OK", 5000);
		if (result != ESP32_OK)
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
				ESP_ConnState = ESP32_CONNECTED_NO_IP;
				HAL_Delay(1000);
				continue;
			}

			DEBUG_LOG("Got IP: %s", ip_buffer);
			ESP_ConnState = ESP32_CONNECTED_IP;
			return ESP32_OK;
		}

		DEBUG_LOG("Attempt %d: Failed to parse STAIP.", attempt);
		HAL_Delay(500);
	}

	DEBUG_LOG("Failed to fetch IP after retries.");
	ESP_ConnState = ESP32_CONNECTED_NO_IP;  // still connected, but no IP
	return ESP32_ERROR;
}

static ESP32_Status ESP_SendCommand(char *cmd, const char *ack, uint32_t timeout)
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
			return ESP32_ERROR;
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
		return ESP32_OK;
	}

	if (idx == 0)
		return ESP32_NO_RESPONSE;

	DEBUG_LOG("Timeout or no ACK. Buffer: %s", esp_rx_buffer);
	return ESP32_TIMEOUT;
}

static ESP32_Status ESP_SendBinary(uint8_t *bin, size_t len, const char *ack, uint32_t timeout)
{
	uint8_t ch;
	uint16_t idx = 0;
	uint32_t tickstart;
	int found = 0;

	memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
	tickstart = HAL_GetTick();

	if (len > 0)
	{
		DEBUG_LOG("Sending Binary Packet");
		if (HAL_UART_Transmit(&ESP_UART, bin, len, HAL_MAX_DELAY) != HAL_OK)
		{
			return ESP32_ERROR;
		}
	}

	while ((HAL_GetTick() - tickstart) < timeout && idx < sizeof(esp_rx_buffer) - 1)
	{
		if (HAL_UART_Receive(&ESP_UART, &ch, 1, 10) == HAL_OK)
		{
			esp_rx_buffer[idx++] = ch;
			esp_rx_buffer[idx] = '\0';

			// check for ACK
			if (!found && strstr(esp_rx_buffer, ack))
			{
				DEBUG_LOG("Matched ACK: %s", ack);
				found = 1; // mark as found but keep reading
			}

			// handle Link Not valid ERROR
			if (strstr(esp_rx_buffer, "ERROR"))
			{
				DEBUG_LOG("ESP Disconnected");
				return ESP32_ERROR;
			}
		}
	}

	if (found)
	{
		DEBUG_LOG("Full buffer: %s", esp_rx_buffer);
		return ESP32_OK;
	}

	DEBUG_LOG("Timeout or no ACK. Buffer: %s", esp_rx_buffer);
	return ESP32_TIMEOUT;
}

static int MQTT_BuildConnect(uint8_t *packet, const char *clientID, const char *username,
		const char *password, uint16_t keepalive)
{
	int len = 0;
	/* Fixed Header */
	packet[len++] = 0x10;   // CONNECT packet type
	int remLenPos = len++;  // Remaining length placeholder

	/* Variable Header */
	packet[len++] = 0x00; packet[len++] = 0x04;
	packet[len++] = 'M'; packet[len++] = 'Q'; packet[len++] = 'T'; packet[len++] = 'T';
	packet[len++] = 0x04;   // Protocol Level = 4 (MQTT 3.1.1)

	uint8_t connectFlags = 0x02; // Clean Session
	if (username) connectFlags |= 0x80;
	if (password) connectFlags |= 0x40;
	packet[len++] = connectFlags;

	// Keep Alive
	packet[len++] = (keepalive >> 8) & 0xFF;
	packet[len++] = (keepalive & 0xFF);

	/* Payload */
	// Client ID
	uint16_t cid_len = strlen(clientID);
	packet[len++] = cid_len >> 8;
	packet[len++] = cid_len & 0xFF;
	memcpy(&packet[len], clientID, cid_len); len += cid_len;

	// Username
	if (username)
	{
		uint16_t ulen = strlen(username);
		packet[len++] = ulen >> 8;
		packet[len++] = ulen & 0xFF;
		memcpy(&packet[len], username, ulen); len += ulen;
	}

	// Password
	if (password)
	{
		uint16_t plen = strlen(password);
		packet[len++] = plen >> 8;
		packet[len++] = plen & 0xFF;
		memcpy(&packet[len], password, plen); len += plen;
	}

	// Remaining length from Fixed Header
	packet[remLenPos] = len - 2;  // remove first 2 bytes of Fixed Header
	return len;
}
