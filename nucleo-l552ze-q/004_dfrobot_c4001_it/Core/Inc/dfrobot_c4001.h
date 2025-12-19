/**
 ******************************************************************************
 * @file    dfrobot_c4001.h
 * @brief   DFRobot C4001 mmWave Radar Sensor Driver for STM32L5
 * @note    Ported from Arduino library to STM32 HAL
 * @author  Converted for STM32L5 (Nucleo-L552ZE-Q)
 * @date    2024
 ******************************************************************************
 * @attention
 *
 * Original copyright:
 * Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * License: MIT License
 * Original author: ZhixinLiu (zhixin.liu@dfrobot.com)
 *
 ******************************************************************************
 */

#ifndef INC_DFROBOT_C4001_H_
#define INC_DFROBOT_C4001_H_

#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES */
#include "stm32l5xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* EXPORTED DEFINES */
#define C4001_I2C_ADDR_0 0x2A
#define C4001_I2C_ADDR_1 0x2B
#define C4001_TIMEOUT 100 // timeout in ms

/* REGISTERS */
#define C4001_REG_STATUS              0x00
#define C4001_REG_CTRL0               0x01
#define C4001_REG_CTRL1               0x02
#define C4001_REG_SOFT_VERSION        0x03
#define C4001_REG_RESULT_STATUS       0x10
#define C4001_REG_TRIG_SENSITIVITY    0x20
#define C4001_REG_KEEP_SENSITIVITY    0x21
#define C4001_REG_TRIG_DELAY          0x22
#define C4001_REG_KEEP_TIMEOUT_L      0x23
#define C4001_REG_KEEP_TIMEOUT_H      0x24
#define C4001_REG_E_MIN_RANGE_L       0x25
#define C4001_REG_E_MIN_RANGE_H       0x26
#define C4001_REG_E_MAX_RANGE_L       0x27
#define C4001_REG_E_MAX_RANGE_H       0x28
#define C4001_REG_E_TRIG_RANGE_L      0x29
#define C4001_REG_E_TRIG_RANGE_H      0x2A

/* SPEED MODE REGISTERS */
#define C4001_REG_RESULT_OBJ_MUN      0x10
#define C4001_REG_RESULT_RANGE_L      0x11
#define C4001_REG_RESULT_RANGE_H      0x12
#define C4001_REG_RESULT_SPEED_L      0x13
#define C4001_REG_RESULT_SPEED_H      0x14
#define C4001_REG_RESULT_ENERGY_L     0x15
#define C4001_REG_RESULT_ENERGY_H     0x16
#define C4001_REG_CFAR_THR_L          0x20
#define C4001_REG_CFAR_THR_H          0x21
#define C4001_REG_T_MIN_RANGE_L       0x22
#define C4001_REG_T_MIN_RANGE_H       0x23
#define C4001_REG_T_MAX_RANGE_L       0x24
#define C4001_REG_T_MAX_RANGE_H       0x25
#define C4001_REG_MICRO_MOTION        0x26

/* UART COMMANDS */
#define C4001_CMD_START           "sensorStart"
#define C4001_CMD_STOP            "sensorStop"
#define C4001_CMD_SAVE            "saveConfig"
#define C4001_CMD_RECOVER         "resetCfg"
#define C4001_CMD_RESET           "resetSystem"
#define C4001_CMD_SPEED_MODE      "setRunApp 1"
#define C4001_CMD_EXIST_MODE      "setRunApp 0"

/* TYPES */
typedef enum {
	C4001_INTERFACE_I2C = 1,
	C4001_INTERFACE_UART = 2
} C4001_Interface_t;

typedef enum {
	C4001_MODE_EXIST = 0x00, // presence detection mode
	C4001_MODE_SPEED = 0x01  // speed measurement mode
} C4001_Mode_t;

typedef enum {
	C4001_CMD_START_SENSOR = 0x55,
	C4001_CMD_STOP_SENSOR = 0X33,
	C4001_CMD_RESET_SENSOR = 0XCC,
	C4001_CMD_RECOVER_SENSOR = 0XAA,
	C4001_CMD_SAVE_PARAMS = 0X5C,
	C4001_CMD_CHANGE_MODE = 0X3B
} C4001_SetMode_t;

typedef enum {
	C4001_SWITCH_OFF = 0x00,
	C4001_SWITCH_ON = 0x01
} C4001_Switch_t;

typedef struct {
	uint8_t work_status; // 0=stopped, 1=running
	uint8_t work_mode;   // 0=exist mode, 1=speed mode
	uint8_t init_status; // 0=not initialized, 1=initialized
} C4001_SensorStatus_t;

typedef struct {
	uint8_t number;
	float speed;
	float range;
	uint32_t energy;
} C4001_TargetData_t;

typedef struct {
	uint8_t pwm1; // duty cycle when no target (0-100)
	uint8_t pwm2; // duty cycle when target detected (0-100)
	uint8_t timer; // timer value (0-255), time = timer * 64ms
} C4001_PwmData_t;

typedef struct {
	bool status;
	float response1;
	float response2;
	float response3;
} C4001_ResponseData_t;

typedef struct {
	uint8_t exist;
	C4001_SensorStatus_t sta;
	C4001_TargetData_t target;
} C4001_AllData_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	UART_HandleTypeDef *huart;

	uint8_t i2c_address;
	C4001_Interface_t interface;

	bool motion_detected_old; // for motion detection state tracking
} C4001_Handle_t;

/* INITIALIZATION */
HAL_StatusTypeDef C4001_Init_I2C(C4001_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef C4001_Init_UART(C4001_Handle_t *dev, UART_HandleTypeDef *huart);

/* BASIC FUNCTIONS */
bool C4001_MotionDetection(C4001_Handle_t *dev);
C4001_SensorStatus_t C4001_GetStatus(C4001_Handle_t *dev);
void C4001_SetSensor(C4001_Handle_t *dev, C4001_SetMode_t mode);
bool C4001_SetSensorMode(C4001_Handle_t *dev, C4001_Mode_t mode);

/* SENSITIVITY CONFIG */
bool C4001_SetTrigSensitivity(C4001_Handle_t *dev, uint8_t sensitivity);
uint8_t C4001_GetTrigSensitivity(C4001_Handle_t *dev);
bool C4001_SetKeepSensitivity(C4001_Handle_t *dev, uint8_t sensitivity);
uint8_t C4001_GetKeepSensitivity(C4001_Handle_t *dev);

/* TIMING CONFIG */
bool C4001_SetDelay(C4001_Handle_t *dev, uint8_t trig, uint16_t keep);
uint8_t C4001_GetTrigDelay(C4001_Handle_t *dev);
uint16_t C4001_GetKeepTimeout(C4001_Handle_t *dev);

/* RANGE CONFIG */
bool C4001_SetDetectionRange(C4001_Handle_t *dev, uint16_t min, uint16_t max, uint16_t trig);
uint16_t C4001_GetMinRange(C4001_Handle_t *dev);
uint16_t C4001_GetMaxRange(C4001_Handle_t *dev);
uint16_t C4001_GetTrigRange(C4001_Handle_t *dev);

/* SPEED MODE */
uint8_t C4001_GetTargetNumber(C4001_Handle_t *dev);
float C4001_GetTargetSpeed(C4001_Handle_t *dev);
float C4001_GetTargetRange(C4001_Handle_t *dev);
uint32_t C4001_GetTargetEnergy(C4001_Handle_t *dev);
bool C4001_SetDetectThres(C4001_Handle_t *dev, uint16_t min, uint16_t max, uint16_t thres);
uint16_t C4001_GetTMinRange(C4001_Handle_t *dev);
uint16_t C4001_GetTMaxRange(C4001_Handle_t *dev);
uint16_t C4001_GetThresRange(C4001_Handle_t *dev);

/* ADVANCED CONFIG */
bool C4001_SetIoPolarity(C4001_Handle_t *dev, uint8_t value);
uint8_t C4001_GetIoPolarity(C4001_Handle_t *dev);
bool C4001_SetPwm(C4001_Handle_t *dev, uint8_t pwm1, uint8_t pwm2, uint8_t timer);
C4001_PwmData_t C4001_GetPwm(C4001_Handle_t *dev);
void C4001_SetFrettingDetection(C4001_Handle_t *dev, C4001_Switch_t sta);
C4001_Switch_t C4001_GetFrettingDetection(C4001_Handle_t *dev);


#ifdef __cplusplus
}
#endif


#endif /* INC_DFROBOT_C4001_H_ */
