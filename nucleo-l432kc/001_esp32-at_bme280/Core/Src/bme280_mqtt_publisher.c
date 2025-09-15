/*
 * bme280_mqtt_publisher.c
 *
 *  Created on: Sep 15, 2025
 *      Author: fahmad
 * BME280 data publisher via MQTT using ESP32-AT module
 * For STM32L432KC
 *
 * This code publishes BME280 sensor data to MQTT broker
 * Supports both TLS and non-TLS connections
 */

#include "main.h"
#include "bme280_stm32.h"
#include "esp32_at_stm32.h"
#include "bme280.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// mqtt config
#define MQTT_BROKER_HOST "m80q"
#define MQTT_BROKER_PORT_TLS 8883
#define MQTT_BROKER_PORT_PLAIN 1883
#define MQTT_USERNAME "iot_device_user"
#define MQTT_PASSWORD "password"
#define MQTT_CLIENT_ID "stm32_bme280_sensor"

// topics
#define TOPIC_TEMPERATURE "sensors/bme280/temperature"
#define TOPIC_HUMIDITY "sensors/bme280/humidity"
#define TOPIC_PRESSURE "sensors/bme280/pressure"
#define TOPIC_ALL_DATA "sensors/bme280/all"

// wifi config
#define WIFI_SSID "Arun_Rawat"
#define WIFI_PASSWORD "arun@321"

// publishing interval (milliseconds)
#define PUBLISH_INTERVAL_MS 5000

// data struct
typedef struct {
	float temperature;
	float humidity;
	float pressure;
	uint32_t timestamp;
} sensor_data_t;

typedef enum {
    MQTT_MODE_PLAIN = 0,
    MQTT_MODE_TLS = 1
} mqtt_mode_t;

// global variables
extern struct bme280_dev bme280_device;
static sensor_data_t latest_sensor_data;
static mqtt_mode_t current_mqtt_mode = MQTT_MODE_TLS;
static uint32_t last_publish_time = 0;
static bool mqtt_connected = false;

/**
 * @brief Setup MQTT connection with or without TLS
 * @param mode MQTT_MODE_PLAIN or MQTT_MODE_TLS
 * @return ESP8266_Status
 */
ESP8266_Status mqtt_setup_connection(mqtt_mode_t mode)
{
	ESP8266_Status result;
	char ip_buffer[16];

	USER_LOG("Setting up MQTT connection (Mode: %s)",
	             mode == MQTT_MODE_TLS ? "TLS" : "Plain");

	result = ESP_Init();
	if (result != ESP8266_OK)
	{
		USER_LOG("ESP32 initialization failed");
		return result;
	}

	result = ESP_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD, ip_buffer, sizeof(ip_buffer));
	if (result != ESP8266_OK)
	{
		USER_LOG("WiFi connection failed");
		return result;
	}

	USER_LOG("WiFi connected, IP: %s", ip_buffer);

	// configure mqtt
	if (mode == MQTT_MODE_TLS)
	{
		// skip certificate verification
		result = ESP_DMA_SendCommand("AT+CIPSSLCCONF=0,0\r\n", "OK", 2000);
		if (result != ESP8266_OK) {
			USER_LOG("SSL configuration failed");
			return result;
		}

		result = ESP_MQTT_Init(MQTT_BROKER_HOST, MQTT_BROKER_PORT_TLS, MQTT_CLIENT_ID);
	}
	else
	{
		result = ESP_MQTT_Init(MQTT_BROKER_HOST, MQTT_BROKER_PORT_PLAIN, MQTT_CLIENT_ID);
	}

	if (result != ESP8266_OK)
	{
		USER_LOG("MQTT initialization failed");
		return result;
	}

	// set mqtt auth
	result = ESP_MQTT_SetAuth(MQTT_USERNAME, MQTT_PASSWORD);
	if (result != ESP8266_OK)
	{
		USER_LOG("MQTT auth setup failed");
		return result;
	}

    result = ESP_MQTT_Connect();
    if (result != ESP8266_OK)
    {
        USER_LOG("MQTT connection failed");
        return result;
    }

    mqtt_connected = true;
    current_mqtt_mode = mode;
    USER_LOG("MQTT connected successfully (%s)",
             mode == MQTT_MODE_TLS ? "TLS" : "Plain");

    return ESP8266_OK;
}

/**
 * @brief Publish sensor data as individual topics
 * @param data Pointer to sensor data structure
 * @return ESP8266_Status
 */
ESP8266_Status mqtt_publish_individual_readings(const sensor_data_t* data)
{
	ESP8266_Status result;
	char message[64];

	// publish temperature
	snprintf(message, sizeof(message), "%.2f", data->temperature);
	result = ESP_MQTT_Publish(TOPIC_TEMPERATURE, message, MQTT_QOS_0, 0);
	if (result != ESP8266_OK)
	{
		DEBUG_LOG("Failed to publish temperature");
		return result;
	}

	HAL_Delay(100);

	// publish humidity
	snprintf(message, sizeof(message), "%.2f", data->humidity);
	result = ESP_MQTT_Publish(TOPIC_HUMIDITY, message, MQTT_QOS_0, 0);
	if (result != ESP8266_OK)
	{
		DEBUG_LOG("Failed to publish humidity");
		return result;
	}

	HAL_Delay(100);

	// publish pressure
	snprintf(message, sizeof(message), "%.2f", data->pressure);
    result = ESP_MQTT_Publish(TOPIC_PRESSURE, message, MQTT_QOS_0, 0);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("Failed to publish pressure");
        return result;
    }

    return ESP8266_OK;
}

/**
 * @brief Publish sensor data in JSON format
 * @param data Pointer to sensor data structure
 * @return ESP8266_Status
 */
ESP8266_Status mqtt_publish_json_format(const sensor_data_t* data)
{
    char json_message[256];

    snprintf(json_message, sizeof(json_message),
    		"{"
    		"\"temperature\":%.2f,"
    		"\"humidity\":%.2f,"
    		"\"pressure\":%.2f,"
    		"\"timestamp\":%lu,"
    		"\"unit_temp\":\"C\","
    		"\"unit_humidity\":\"%%\","
    		"\"unit_pressure\":\"hPa\""
    		"}",
			data->temperature,
			data->humidity,
			data->pressure,
			data->timestamp
    );

    return ESP_MQTT_Publish(TOPIC_ALL_DATA, json_message, MQTT_QOS_0, 0);
}

/**
 * @brief Main function to publish sensor data
 * @param data Pointer to sensor data structure
 * @return ESP8266_Status
 */
ESP8266_Status mqtt_publish_sensor_data(const sensor_data_t* data)
{
	ESP8266_Status result;

	if (!mqtt_connected)
	{
		USER_LOG("MQTT not connected, cannot publish");
		return ESP8266_NOT_CONNECTED;
	}

	USER_LOG("Publishing sensor data: T=%.2f°C, H=%.2f%%, P=%.2fhPa",
			data->temperature, data->humidity, data->pressure);

	// publish json
    result = mqtt_publish_json_format(data);
    if (result != ESP8266_OK)
    {
        USER_LOG("Failed to publish JSON data");
        return result;
    }

    USER_LOG("Sensor data published successfully");
    return ESP8266_OK;
}

/**
 * @brief Disconnect from MQTT broker
 * @return ESP8266_Status
 */
ESP8266_Status mqtt_disconnect(void)
{
    ESP8266_Status result = ESP_MQTT_Disconnect();
    mqtt_connected = false;
    USER_LOG("MQTT disconnected");
    return result;
}

void bme280_mqtt_task(void)
{
	static bool initialized = false;
	struct bme280_data comp_data;
	int8_t bme_result;
	ESP8266_Status mqtt_result;
	uint32_t current_time = HAL_GetTick();

	if (!initialized)
	{
		mqtt_result = mqtt_setup_connection(MQTT_MODE_TLS);
		if (mqtt_result != ESP8266_OK)
		{
			USER_LOG("TLS connection failed, trying plain MQTT");
			mqtt_result = mqtt_setup_connection(MQTT_MODE_PLAIN);
			if (mqtt_result != ESP8266_OK)
			{
				USER_LOG("All MQTT connections failed, retrying in 10 seconds");
				HAL_Delay(10000);
				return;
			}
		}
		initialized = true;
	}

	// is it time to publish ?
    if ((current_time - last_publish_time) < PUBLISH_INTERVAL_MS)
    {
        return;
    }

    // read BME280
    bme_result = bme280_read_sensor_data(&comp_data, &bme280_device);
    if (bme_result != BME280_OK)
    {
        USER_LOG("Failed to read BME280 data: %d", bme_result);
        return;
    }

    latest_sensor_data.temperature = comp_data.temperature;
    latest_sensor_data.humidity = comp_data.humidity;
    latest_sensor_data.pressure = comp_data.pressure / 100.0f; // convert to hPa
    latest_sensor_data.timestamp = current_time;

    // publish data
    mqtt_result = mqtt_publish_sensor_data(&latest_sensor_data);
    if (mqtt_result != ESP8266_OK)
    {
    	USER_LOG("Failed to publish sensor data, error: %d", mqtt_result);

    	// reconnect on failure
    	mqtt_connected = false;
    	initialized = false;
    	return;
    }

    last_publish_time = current_time;
}

/**
 * @brief Switch MQTT mode (for testing both TLS and plain)
 * @param mode New MQTT mode
 */
void mqtt_switch_mode(mqtt_mode_t mode)
{
    if (current_mqtt_mode != mode)
    {
        USER_LOG("Switching MQTT mode from %s to %s",
                 current_mqtt_mode == MQTT_MODE_TLS ? "TLS" : "Plain",
                 mode == MQTT_MODE_TLS ? "TLS" : "Plain");

        // disconnect current connection
        if (mqtt_connected)
        {
            mqtt_disconnect();
        }

        // reset initialization flag to force reconnection
        mqtt_connected = false;
        current_mqtt_mode = mode;
    }
}

/**
 * @brief Get latest sensor data (for other parts of application)
 * @return Pointer to latest sensor data
 */
const sensor_data_t* get_latest_sensor_data(void)
{
    return &latest_sensor_data;
}

/**
 * @brief Check if MQTT is connected
 * @return true if connected, false otherwise
 */
bool is_mqtt_connected(void)
{
    return mqtt_connected;
}
