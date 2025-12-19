/**
 ******************************************************************************
 * @file    dfrobot_c4001.c
 * @brief   DFRobot C4001 mmWave Radar Sensor Driver Implementation for STM32L5
 * @note    Ported from Arduino library to STM32 HAL
 ******************************************************************************
 */

#include "dfrobot_c4001.h"

#define UART_RX_BUFFER_SIZE 200
#define UART_TX_BUFFER_SIZE 100

/* PRIVATE */
static HAL_StatusTypeDef C4001_WriteReg_I2C(C4001_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef C4001_ReadReg_I2C(C4001_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef C4001_WriteReg_UART(C4001_Handle_t *dev, uint8_t *data, uint16_t len);
static int16_t C4001_ReadReg_UART(C4001_Handle_t *dev, uint8_t *data, uint16_t len, uint32_t timeout_ms);
static bool C4001_SensorStop(C4001_Handle_t *dev);
static C4001_ResponseData_t C4001_WriteCmdRead(C4001_Handle_t *dev, const char *cmd, uint8_t count);
static void C4001_WriteCmd(C4001_Handle_t *dev, const char *cmd1, const char *cmd2, uint8_t count);
static C4001_AllData_t C4001_AnalyzeData(uint8_t *data, uint16_t len);
static C4001_ResponseData_t C4001_AnalyzeResponse(uint8_t *data, uint16_t len, uint8_t count);


/* PUBLIC */
HAL_StatusTypeDef C4001_Init_I2C(C4001_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
	if (dev == NULL || hi2c == NULL)
	{
		return HAL_ERROR;
	}

	dev->hi2c = hi2c;
	dev->huart = NULL;
	dev->i2c_address = i2c_addr << 1;
	dev->interface = C4001_INTERFACE_I2C;
	dev->motion_detected_old = false;

	return HAL_I2C_IsDeviceReady(hi2c, dev->i2c_address, 3, C4001_TIMEOUT);
}

HAL_StatusTypeDef C4001_Init_UART(C4001_Handle_t *dev, UART_HandleTypeDef *huart)
{
	if (dev == NULL || huart == NULL)
	{
		return HAL_ERROR;
	}

	dev->hi2c = NULL;
	dev->huart = huart;
	dev->interface = C4001_INTERFACE_UART;
	dev->motion_detected_old = false;

	return HAL_OK;
}

C4001_SensorStatus_t C4001_GetStatus(C4001_Handle_t *dev)
{
	C4001_SensorStatus_t status = {0};

	if (dev->interface == C4001_INTERFACE_I2C)
	{
		uint8_t temp = 0;
		C4001_ReadReg_I2C(dev, C4001_REG_STATUS, &temp, 1);
		status.work_status = (temp & 0x01);
		status.work_mode = (temp & 0x02) >> 1;
		status.init_status = (temp & 0x80) >> 7;
	}
	else
	{
		uint8_t temp[100] = {0};
		uint16_t len = 0;
		C4001_AllData_t all_data;

		C4001_ReadReg_UART(dev, temp, 100, 10);
		C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));

		while(len == 0)
		{
			HAL_Delay(1000);
			len = C4001_ReadReg_UART(dev, temp, 100, 1000);
			all_data = C4001_AnalyzeData(temp, len);
		}

		status.work_status = all_data.sta.work_status;
		status.work_mode = all_data.sta.work_mode;
		status.init_status = all_data.sta.init_status;
	}

	return status;
}

bool C4001_MotionDetection(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_RESULT_STATUS, &temp, 1);
        return (temp & 0x01) ? true : false;
    }
    else
    {
        uint8_t temp[100] = {0};
        uint16_t len = 0;
        C4001_AllData_t data;

        len = C4001_ReadReg_UART(dev, temp, 100, 100);
        data = C4001_AnalyzeData(temp, len);

        if (data.exist) {
            dev->motion_detected_old = true;
            return true;
        } else {
            return dev->motion_detected_old;
        }
    }
}

void C4001_SetSensor(C4001_Handle_t *dev, C4001_SetMode_t mode)
{
    uint8_t temp = mode;

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        switch (mode) {
            case C4001_CMD_START_SENSOR:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL0, &temp, 1);
                HAL_Delay(200);
                break;

            case C4001_CMD_STOP_SENSOR:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL0, &temp, 1);
                HAL_Delay(200);
                break;

            case C4001_CMD_RESET_SENSOR:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL0, &temp, 1);
                HAL_Delay(1500);
                break;

            case C4001_CMD_SAVE_PARAMS:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL1, &temp, 1);
                HAL_Delay(500);
                break;

            case C4001_CMD_RECOVER_SENSOR:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL1, &temp, 1);
                HAL_Delay(800);
                break;

            case C4001_CMD_CHANGE_MODE:
                C4001_WriteReg_I2C(dev, C4001_REG_CTRL1, &temp, 1);
                HAL_Delay(1500);
                break;
        }
    }
    else
    {
        switch (mode) {
            case C4001_CMD_START_SENSOR:
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
                HAL_Delay(200);
                break;

            case C4001_CMD_STOP_SENSOR:
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_STOP, strlen(C4001_CMD_STOP));
                HAL_Delay(200);
                break;

            case C4001_CMD_RESET_SENSOR:
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_RESET, strlen(C4001_CMD_RESET));
                HAL_Delay(1500);
                break;

            case C4001_CMD_SAVE_PARAMS:
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_STOP, strlen(C4001_CMD_STOP));
                HAL_Delay(200);
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_SAVE, strlen(C4001_CMD_SAVE));
                HAL_Delay(800);
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
                break;

            case C4001_CMD_RECOVER_SENSOR:
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_STOP, strlen(C4001_CMD_STOP));
                HAL_Delay(200);
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_RECOVER, strlen(C4001_CMD_RECOVER));
                HAL_Delay(800);
                C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
                HAL_Delay(500);
                break;

            default:
                break;
        }
    }
}

bool C4001_SetSensorMode(C4001_Handle_t *dev, C4001_Mode_t mode)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        C4001_SensorStatus_t status = C4001_GetStatus(dev);

        if (status.work_mode == mode)
        {
            return true;
        }
        else
        {
            C4001_SetSensor(dev, C4001_CMD_CHANGE_MODE);
            status = C4001_GetStatus(dev);
            return (status.work_mode == mode);
        }
    }
    else
    {
        C4001_SensorStop(dev);

        if (mode == C4001_MODE_EXIST)
        {
            C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_EXIST_MODE, strlen(C4001_CMD_EXIST_MODE));
        }
        else
        {
            C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_SPEED_MODE, strlen(C4001_CMD_SPEED_MODE));
        }

        HAL_Delay(50);
        C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_SAVE, strlen(C4001_CMD_SAVE));
        HAL_Delay(500);
        C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
        HAL_Delay(100);

        return true;
    }
}

/**
 * @brief Set trigger sensitivity
 * @param dev: Device handle
 * @param sensitivity: Sensitivity value (0-9)
 * @retval true on success
 */
bool C4001_SetTrigSensitivity(C4001_Handle_t *dev, uint8_t sensitivity)
{
    if (sensitivity > 9)
    {
        return false;
    }

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        C4001_WriteReg_I2C(dev, C4001_REG_TRIG_SENSITIVITY, &sensitivity, 1);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setSensitivity 255 %d", sensitivity);
        C4001_WriteCmd(dev, cmd, cmd, 1);
        return true;
    }
}

/**
 * @brief Get trigger sensitivity
 * @param dev: Device handle
 * @retval Sensitivity value
 */
uint8_t C4001_GetTrigSensitivity(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_TRIG_SENSITIVITY, &temp, 1);
        return temp;
    }
    else
    {
        C4001_ResponseData_t response;
        uint8_t temp[100] = {0};
        C4001_ReadReg_UART(dev, temp, 100, 10);
        response = C4001_WriteCmdRead(dev, "getSensitivity", 1);
        return response.status ? (uint8_t)response.response1 : 0;
    }
}

/**
 * @brief Set keep sensitivity
 * @param dev: Device handle
 * @param sensitivity: Sensitivity value (0-9)
 * @retval true on success
 */
bool C4001_SetKeepSensitivity(C4001_Handle_t *dev, uint8_t sensitivity)
{
    if (sensitivity > 9)
    {
        return false;
    }

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        C4001_WriteReg_I2C(dev, C4001_REG_KEEP_SENSITIVITY, &sensitivity, 1);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setSensitivity %d 255", sensitivity);
        C4001_WriteCmd(dev, cmd, cmd, 1);
        return true;
    }
}

uint8_t C4001_GetKeepSensitivity(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_KEEP_SENSITIVITY, &temp, 1);
        return temp;
    }
    else
    {
        C4001_ResponseData_t response;
        uint8_t temp[100] = {0};
        C4001_ReadReg_UART(dev, temp, 100, 10);
        response = C4001_WriteCmdRead(dev, "getSensitivity", 2);
        return response.status ? (uint8_t)response.response2 : 0;
    }
}

/**
 * @brief Set delay parameters
 * @param dev: Device handle
 * @param trig: Trigger delay in 0.01s units (0-200)
 * @param keep: Keep timeout in 0.5s units (4-3000)
 * @retval true on success
 */
bool C4001_SetDelay(C4001_Handle_t *dev, uint8_t trig, uint16_t keep)
{
    if (trig > 200 || keep < 4 || keep > 3000)
    {
        return false;
    }

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[3];
        temp[0] = trig;
        temp[1] = (uint8_t)(keep & 0xFF);
        temp[2] = (uint8_t)(keep >> 8);
        C4001_WriteReg_I2C(dev, C4001_REG_TRIG_DELAY, temp, 3);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "setLatency %.1f %.1f", trig * 0.01f, keep * 0.5f);
        C4001_WriteCmd(dev, cmd, cmd, 1);
        return true;
    }
}

uint8_t C4001_GetTrigDelay(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_TRIG_DELAY, &temp, 1);
        return temp;
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getLatency", 1);
        return response.status ? (uint8_t)(response.response1 * 100) : 0;
    }
}

uint16_t C4001_GetKeepTimeout(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_KEEP_TIMEOUT_L, temp, 2);
        return (((uint16_t)temp[1]) << 8) | temp[0];
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getLatency", 2);
        return response.status ? (uint16_t)(response.response2 * 2) : 0;
    }
}

/**
 * @brief Set detection range
 * @param dev: Device handle
 * @param min: Minimum range in cm (30-2000)
 * @param max: Maximum range in cm (240-2000)
 * @param trig: Trigger range in cm (240-2000)
 * @retval true on success
 */
bool C4001_SetDetectionRange(C4001_Handle_t *dev, uint16_t min, uint16_t max, uint16_t trig)
{
    if (max < 240 || max > 2000 || min < 30 || min > max)
    {
        return false;
    }

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[6];
        temp[0] = (uint8_t)(min & 0xFF);
        temp[1] = (uint8_t)(min >> 8);
        temp[2] = (uint8_t)(max & 0xFF);
        temp[3] = (uint8_t)(max >> 8);
        temp[4] = (uint8_t)(trig & 0xFF);
        temp[5] = (uint8_t)(trig >> 8);
        C4001_WriteReg_I2C(dev, C4001_REG_E_MIN_RANGE_L, temp, 6);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd1[32], cmd2[32];
        snprintf(cmd1, sizeof(cmd1), "setRange %.1f %.1f", min / 100.0f, max / 100.0f);
        snprintf(cmd2, sizeof(cmd2), "setTrigRange %.1f", trig / 100.0f);
        C4001_WriteCmd(dev, cmd1, cmd2, 2);
        return true;
    }
}

/**
 * @brief Get minimum detection range
 * @param dev: Device handle
 * @retval Minimum range in cm
 */
uint16_t C4001_GetMinRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_E_MIN_RANGE_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getRange", 1);
        return response.status ? (uint16_t)(response.response1 * 100) : 0;
    }
}

/**
 * @brief Get maximum detection range
 * @param dev: Device handle
 * @retval Maximum range in cm
 */
uint16_t C4001_GetMaxRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_E_MAX_RANGE_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getRange", 2);
        return response.status ? (uint16_t)(response.response2 * 100) : 0;
    }
}

/**
 * @brief Get trigger range
 * @param dev: Device handle
 * @retval Trigger range in cm
 */
uint16_t C4001_GetTrigRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_E_TRIG_RANGE_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getTrigRange", 1);
        return response.status ? (uint16_t)(response.response1 * 100) : 0;
    }
}

/**
 * @brief Get target number (speed mode)
 * @param dev: Device handle
 * @retval Number of targets detected
 */
uint8_t C4001_GetTargetNumber(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_RESULT_OBJ_MUN, &temp, 1);
        return temp;
    }
    else
    {
        uint8_t temp[100] = {0};
        uint16_t len = C4001_ReadReg_UART(dev, temp, 100, 100);
        C4001_AllData_t data = C4001_AnalyzeData(temp, len);
        return data.target.number;
    }
}

/**
 * @brief Get target speed (speed mode)
 * @param dev: Device handle
 * @retval Target speed in cm/s
 */
float C4001_GetTargetSpeed(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_RESULT_SPEED_L, temp, 2);
        int16_t speed_raw = (int16_t)(temp[0] | ((uint16_t)temp[1] << 8));
        return speed_raw / 100.0f;
    }
    else
    {
        uint8_t temp[100] = {0};
        uint16_t len = C4001_ReadReg_UART(dev, temp, 100, 100);
        C4001_AllData_t data = C4001_AnalyzeData(temp, len);
        return data.target.speed;
    }
}

/**
 * @brief Get target range (speed mode)
 * @param dev: Device handle
 * @retval Target range in cm
 */
float C4001_GetTargetRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_RESULT_RANGE_L, temp, 2);
        uint16_t range_raw = (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
        return range_raw / 100.0f;
    }
    else
    {
        uint8_t temp[100] = {0};
        uint16_t len = C4001_ReadReg_UART(dev, temp, 100, 100);
        C4001_AllData_t data = C4001_AnalyzeData(temp, len);
        return data.target.range;
    }
}

/**
 * @brief Get target energy (speed mode)
 * @param dev: Device handle
 * @retval Target energy value
 */
uint32_t C4001_GetTargetEnergy(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_RESULT_ENERGY_L, temp, 2);
        return (uint32_t)(temp[0] | ((uint32_t)temp[1] << 8));
    }
    else
    {
        uint8_t temp[100] = {0};
        uint16_t len = C4001_ReadReg_UART(dev, temp, 100, 100);
        C4001_AllData_t data = C4001_AnalyzeData(temp, len);
        return data.target.energy;
    }
}

/**
 * @brief Set detection threshold (speed mode)
 * @param dev: Device handle
 * @param min: Minimum range in cm (30-2000)
 * @param max: Maximum range in cm (240-2000)
 * @param thres: Threshold value (0-65535)
 * @retval true on success
 */
bool C4001_SetDetectThres(C4001_Handle_t *dev, uint16_t min, uint16_t max, uint16_t thres)
{
    if (max < 240 || max > 2000 || min < 30 || min > max)
    {
        return false;
    }

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[6];
        temp[0] = (uint8_t)(min & 0xFF);
        temp[1] = (uint8_t)(min >> 8);
        temp[2] = (uint8_t)(max & 0xFF);
        temp[3] = (uint8_t)(max >> 8);
        temp[4] = (uint8_t)(thres & 0xFF);
        temp[5] = (uint8_t)(thres >> 8);
        C4001_WriteReg_I2C(dev, C4001_REG_T_MIN_RANGE_L, temp, 6);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd1[32], cmd2[32];
        snprintf(cmd1, sizeof(cmd1), "setRange %.1f %.1f", min / 100.0f, max / 100.0f);
        snprintf(cmd2, sizeof(cmd2), "setThrFactor %.1f", thres / 10.0f);
        C4001_WriteCmd(dev, cmd1, cmd2, 2);
        return true;
    }
}

/**
 * @brief Get minimum range for speed mode
 * @param dev: Device handle
 * @retval Minimum range in cm
 */
uint16_t C4001_GetTMinRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_T_MIN_RANGE_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getRange", 1);
        return response.status ? (uint16_t)(response.response1 * 100) : 0;
    }
}

/**
 * @brief Get maximum range for speed mode
 * @param dev: Device handle
 * @retval Maximum range in cm
 */
uint16_t C4001_GetTMaxRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_T_MAX_RANGE_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getRange", 2);
        return response.status ? (uint16_t)(response.response2 * 100) : 0;
    }
}

/**
 * @brief Get threshold range
 * @param dev: Device handle
 * @retval Threshold value
 */
uint16_t C4001_GetThresRange(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[2] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_CFAR_THR_L, temp, 2);
        return (uint16_t)(temp[0] | ((uint16_t)temp[1] << 8));
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getThrFactor", 1);
        return response.status ? (uint16_t)response.response1 : 0;
    }
}

/**
 * @brief Set I/O pin polarity
 * @param dev: Device handle
 * @param value: 0=low when target, 1=high when target (default)
 * @retval true on success
 */
bool C4001_SetIoPolarity(C4001_Handle_t *dev, uint8_t value)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        C4001_WriteReg_I2C(dev, C4001_REG_CTRL1, &value, 1);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
        return true;
    }
    else
    {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setIoPolarity %d", value);
        C4001_WriteCmd(dev, cmd, cmd, 1);
        return true;
    }
}

/**
 * @brief Get I/O pin polarity
 * @param dev: Device handle
 * @retval Polarity value
 */
uint8_t C4001_GetIoPolarity(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_CTRL1, &temp, 1);
        return temp;
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getIoPolarity", 1);
        return response.status ? (uint8_t)response.response1 : 0;
    }
}

/**
 * @brief Get PWM configuration
 * @param dev: Device handle
 * @retval PWM configuration structure
 */
C4001_PwmData_t C4001_GetPwm(C4001_Handle_t *dev)
{
    C4001_PwmData_t pwm_data = {0};

    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp[3] = {0};
        C4001_ReadReg_I2C(dev, C4001_REG_CTRL1, temp, 3);
        pwm_data.pwm1 = temp[0];
        pwm_data.pwm2 = temp[1];
        pwm_data.timer = temp[2];
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getPwm", 3);
        if (response.status)
        {
            pwm_data.pwm1 = (uint8_t)response.response1;
            pwm_data.pwm2 = (uint8_t)response.response2;
            pwm_data.timer = (uint8_t)response.response3;
        }
    }

    return pwm_data;
}

/**
 * @brief Set fretting/micro-motion detection
 * @param dev: Device handle
 * @param sta: Switch state (C4001_SWITCH_ON or C4001_SWITCH_OFF)
 */
void C4001_SetFrettingDetection(C4001_Handle_t *dev, C4001_Switch_t sta)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = sta;
        C4001_WriteReg_I2C(dev, C4001_REG_MICRO_MOTION, &temp, 1);
        C4001_SetSensor(dev, C4001_CMD_SAVE_PARAMS);
    }
    else
    {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setMicroMotion %d", sta);
        C4001_WriteCmd(dev, cmd, cmd, 1);
    }
}

/**
 * @brief Get fretting detection status
 * @param dev: Device handle
 * @retval Switch state
 */
C4001_Switch_t C4001_GetFrettingDetection(C4001_Handle_t *dev)
{
    if (dev->interface == C4001_INTERFACE_I2C)
    {
        uint8_t temp = 0;
        C4001_ReadReg_I2C(dev, C4001_REG_MICRO_MOTION, &temp, 1);
        return (C4001_Switch_t)temp;
    }
    else
    {
        C4001_ResponseData_t response = C4001_WriteCmdRead(dev, "getMicroMotion", 1);
        return response.status ? (C4001_Switch_t)response.response1 : C4001_SWITCH_OFF;
    }
}

/* PRIVATE */
static HAL_StatusTypeDef C4001_WriteReg_I2C(C4001_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, len, C4001_TIMEOUT);
}

static HAL_StatusTypeDef C4001_ReadReg_I2C(C4001_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	return HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, data, len, C4001_TIMEOUT);
}

static HAL_StatusTypeDef C4001_WriteReg_UART(C4001_Handle_t *dev, uint8_t *data, uint16_t len)
{
	return HAL_UART_Transmit(dev->huart, data, len, C4001_TIMEOUT);
}

static int16_t C4001_ReadReg_UART(C4001_Handle_t *dev, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
	uint16_t i = 0;
	uint32_t start_tick = HAL_GetTick();

	while ((HAL_GetTick() - start_tick) < timeout_ms)
	{
		if (HAL_UART_Receive(dev->huart, &data[i], 1, 10) == HAL_OK)
		{
			i++;
			if (i >= len)
			{
				break;
			}
		}
	}

	return i;
}

static bool C4001_SensorStop(C4001_Handle_t *dev)
{
	uint8_t temp[200] = {0};
	uint16_t len = 0;

	C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_STOP, strlen(C4001_CMD_STOP));
	HAL_Delay(1000);
	len = C4001_ReadReg_UART(dev, temp, 200, 1000);

	while (1)
	{
		if (len != 0)
		{
			if (strstr((const char *)temp, "sensorStop") != NULL)
			{
				return true;
			}
		}

		memset(temp, 0, 200);
		HAL_Delay(400);
		C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_STOP, strlen(C4001_CMD_STOP));
		len = C4001_ReadReg_UART(dev, temp, 200, 400);
	}
}

static C4001_ResponseData_t C4001_WriteCmdRead(C4001_Handle_t *dev, const char *cmd, uint8_t count)
{
	uint8_t temp[200] = {0};
	uint16_t len = 0;
	C4001_ResponseData_t response;

	C4001_SensorStop(dev);
	C4001_WriteReg_UART(dev, (uint8_t *)cmd, strlen(cmd));
	HAL_Delay(100);
	len = C4001_ReadReg_UART(dev, temp, 200, 1000);
	response = C4001_AnalyzeResponse(temp, len, count);
	HAL_Delay(100);
	C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
	HAL_Delay(100);

	return response;
}

static void C4001_WriteCmd(C4001_Handle_t *dev, const char *cmd1, const char *cmd2, uint8_t count)
{
	C4001_SensorStop(dev);
	C4001_WriteReg_UART(dev, (uint8_t *)cmd1, strlen(cmd1));
	HAL_Delay(100);

	if (count > 1)
	{
		HAL_Delay(100);
		C4001_WriteReg_UART(dev, (uint8_t *)cmd2, strlen(cmd2));
		HAL_Delay(100);
	}

	C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_SAVE, strlen(C4001_CMD_SAVE));
	HAL_Delay(100);
	C4001_WriteReg_UART(dev, (uint8_t *)C4001_CMD_START, strlen(C4001_CMD_START));
	HAL_Delay(100);
}

static C4001_AllData_t C4001_AnalyzeData(uint8_t *data, uint16_t len)
{
	C4001_AllData_t all_data = {0};
	uint16_t location = 0;

	// find start marker "$"
	for (uint16_t i = 0; i < len; i++)
	{
		if (data[i] == '$')
		{
			location = i;
			break;
		}
	}

	if (location == len)
	{
		return all_data;
	}

	// parse presence detection frame: $DFHPD,status*
	if (strncmp((const char *)(data + location), "$DFHPD", 6) == 0)
	{
		all_data.sta.work_mode = C4001_MODE_EXIST;
		all_data.sta.work_status = 1;
		all_data.sta.init_status = 1;

		if (data[location + 7] == '1')
		{
			all_data.exist = 1;
		}
		else
		{
			all_data.exist = 0;
		}
	}
	// parse speed frame $DFDMD,num,x,range,speed,energy,...*
	else if (strncmp((const char *)(data + location), "$DFDMD", 6) == 0)
	{
        all_data.sta.work_mode = C4001_MODE_SPEED;
        all_data.sta.work_status = 1;
        all_data.sta.init_status = 1;

        char *token;
        char *parts[10];
        int index = 0;

        token = strtok((char *)(data + location), ",");
        while (token != NULL && index < 10)
        {
            parts[index++] = token;
            token = strtok(NULL, ",");
        }

        if (index >= 6)
        {
            all_data.target.number = atoi(parts[1]);
            all_data.target.range = atof(parts[3]) * 100.0f;
            all_data.target.speed = atof(parts[4]) * 100.0f;
            all_data.target.energy = (uint32_t)atof(parts[5]);
        }
	}

	return all_data;
}

static C4001_ResponseData_t C4001_AnalyzeResponse(uint8_t *data, uint16_t len, uint8_t count)
{
    C4001_ResponseData_t response = {0};
    uint16_t space[5] = {0};
    uint16_t i = 0, j = 0;

    // Find "Res" in response
    for (i = 0; i < len - 2; i++)
    {
        if (data[i] == 'R' && data[i + 1] == 'e' && data[i + 2] == 's')
        {
            break;
        }
    }

    if (i >= len - 2 || i == 0)
    {
        response.status = false;
        return response;
    }

    response.status = true;

    // Find space positions
    for (j = 0; i < len; i++)
    {
        if (data[i] == ' ' && j < 5)
        {
            space[j++] = i + 1;
        }
    }

    if (j > 0)
    {
        response.response1 = atof((const char *)(data + space[0]));
        if (j >= 2)
        {
            response.response2 = atof((const char *)(data + space[1]));
        }
        if (count >= 3 && j >= 3)
        {
            response.response3 = atof((const char *)(data + space[2]));
        }
    }

    return response;
}













