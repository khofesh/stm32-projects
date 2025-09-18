/*
 * bme280_mqtt_publisher.h
 *
 *  Created on: Sep 15, 2025
 *      Author: fahmad
 */

#pragma once

#include <stdbool.h>

typedef enum
{
    MQTT_MODE_PLAIN = 0,
    MQTT_MODE_TLS = 1
} mqtt_mode_t;

typedef struct
{
    float temperature;
    float humidity;
    float pressure;
    uint32_t timestamp;
} sensor_data_t;

void mqtt_switch_mode(mqtt_mode_t mode);
const sensor_data_t *get_latest_sensor_data(void);
bool is_mqtt_connected(void);

ESP8266_Status mqtt_publish_sensor_data(const sensor_data_t *data);
ESP8266_Status mqtt_publish_individual_readings(const sensor_data_t *data);
ESP8266_Status mqtt_publish_json_format(const sensor_data_t *data);
void bme280_mqtt_task(void);
ESP8266_Status mqtt_disconnect(void);
