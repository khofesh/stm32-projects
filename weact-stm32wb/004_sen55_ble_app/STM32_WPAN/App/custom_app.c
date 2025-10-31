/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/custom_app.c
 * @author  MCD Application Team
 * @brief   Custom Example Application (Server)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensirion_i2c_hal.h"
#include "sen5x_i2c.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* Sen55_P2P_Server */
  uint8_t               Sen55_c_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t SEN55_Status;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Sen55_P2P_Server */
static void Custom_Sen55_c_Update_Char(void);
static void Custom_Sen55_c_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Send_SEN55_Data_To_USB(sen55_data_t *sensor_data);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* Sen55_P2P_Server */
    case CUSTOM_STM_LED_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_LED_C_READ_EVT */
      break;

    case CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
    APP_DBG_MSG("\r\n\r** CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT \n");
    APP_DBG_MSG("\r\n\r** Write Data: 0x%02X %02X \n", pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
    if (pNotification->DataTransfered.pPayload[1] == 0x01)
    {
      HAL_GPIO_WritePin(GPIOE, BOARD_LED_Pin, GPIO_PIN_SET);
    }
    if (pNotification->DataTransfered.pPayload[1] == 0x00)
    {
      HAL_GPIO_WritePin(GPIOE, BOARD_LED_Pin, GPIO_PIN_RESET);
    }
      /* USER CODE END CUSTOM_STM_LED_C_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SEN55_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SEN55_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_SEN55_C_READ_EVT */
      break;

    case CUSTOM_STM_SEN55_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SEN55_C_NOTIFY_ENABLED_EVT */
    APP_DBG_MSG("\r\n\r** CUSTOM_STM_SEN55_C_NOTIFY_ENABLED_EVT \n");

    Custom_App_Context.Sen55_c_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_SEN55_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SEN55_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SEN55_C_NOTIFY_DISABLED_EVT */
    APP_DBG_MSG("\r\n\r** CUSTOM_STM_SEN55_C_NOTIFY_DISABLED_EVT \n");

    Custom_App_Context.Sen55_c_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SEN55_C_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  UTIL_SEQ_RegTask(1 << CFG_TASK_SEN55_I2C_EVT_ID, UTIL_SEQ_RFU, Custom_Sen55_c_Send_Notification);

  Custom_App_Context.Sen55_c_Notification_Status = 0;
  Custom_App_Context.SEN55_Status = 0;
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* Sen55_P2P_Server */
__USED void Custom_Sen55_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sen55_c_UC_1*/

  /* USER CODE END Sen55_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SEN55_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sen55_c_UC_Last*/

  /* USER CODE END Sen55_c_UC_Last*/
  return;
}

void Custom_Sen55_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sen55_c_NS_1*/
  static sen55_data_t sensor_data;
  static uint32_t last_usb_send_time = 0;

  if (SEN55_ReadAllData(&sensor_data) == HAL_OK)
  {
	  uint32_t current_time = HAL_GetTick();

	  // send to usb cdc every 5 seconds (5000ms)
	  if ((current_time - last_usb_send_time) >= 5000)
	  {
		  Send_SEN55_Data_To_USB(&sensor_data);
		  last_usb_send_time = current_time;
	  }

	  // send to BLE only if notification enabled
	  if (Custom_App_Context.Sen55_c_Notification_Status)
	  {
		  updateflag = 1;

		  // pack sensor data into notification buffer
		  // format: [PM1.0(2)] [PM2.5(2)] [PM4.0(2)] [PM10(2)] [Temp(2)] [Hum(2)] [VOC(2)] [NOx(2)]
		  NotifyCharData[0] = (uint8_t)(sensor_data.pm1_0 & 0xFF);
		  NotifyCharData[1] = (uint8_t)(sensor_data.pm1_0 >> 8);
		  NotifyCharData[2] = (uint8_t)(sensor_data.pm2_5 & 0xFF);
		  NotifyCharData[3] = (uint8_t)(sensor_data.pm2_5 >> 8);
		  NotifyCharData[4] = (uint8_t)(sensor_data.pm4_0 & 0xFF);
		  NotifyCharData[5] = (uint8_t)(sensor_data.pm4_0 >> 8);
		  NotifyCharData[6] = (uint8_t)(sensor_data.pm10 & 0xFF);
		  NotifyCharData[7] = (uint8_t)(sensor_data.pm10 >> 8);
		  NotifyCharData[8] = (uint8_t)(sensor_data.temperature & 0xFF);
		  NotifyCharData[9] = (uint8_t)(sensor_data.temperature >> 8);
		  NotifyCharData[10] = (uint8_t)(sensor_data.humidity & 0xFF);
		  NotifyCharData[11] = (uint8_t)(sensor_data.humidity >> 8);
		  NotifyCharData[12] = (uint8_t)(sensor_data.voc_index & 0xFF);
		  NotifyCharData[13] = (uint8_t)(sensor_data.voc_index >> 8);
		  NotifyCharData[14] = (uint8_t)(sensor_data.nox_index & 0xFF);
		  NotifyCharData[15] = (uint8_t)(sensor_data.nox_index >> 8);

		  APP_DBG_MSG("-- CUSTOM APPLICATION SERVER : SENDING SEN55 DATA\n");
		  APP_DBG_MSG("PM1.0: %.1f µg/m³, PM2.5: %.1f µg/m³\n",
				  sensor_data.pm1_0 / 10.0f, sensor_data.pm2_5 / 10.0f);
		  APP_DBG_MSG("Temperature: %.1f °C, Humidity: %.1f %%RH\n",
				  sensor_data.temperature / 200.0f, sensor_data.humidity / 100.0f);
	  }
	  else
	  {
		  APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT - NOTIFICATION DISABLED\n");
	  }
  }
  else
  {
	  APP_DBG_MSG("-- CUSTOM APPLICATION : ERROR READING SEN55 DATA\n");
  }

  /* USER CODE END Sen55_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SEN55_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sen55_c_NS_Last*/

  /* USER CODE END Sen55_c_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
/**
 * @brief Send SEN55 sensor data to USB CDC
 * @param sensor_data Pointer to sen55_data_t structure
 */
static void Send_SEN55_Data_To_USB(sen55_data_t *sensor_data)
{
  char usb_buffer[256];
  int len;
  
  // Format sensor data as JSON
  len = snprintf(usb_buffer, sizeof(usb_buffer),
                 "{\"pm1_0\":%.1f,\"pm2_5\":%.1f,\"pm4_0\":%.1f,\"pm10\":%.1f,"
                 "\"temperature\":%.2f,\"humidity\":%.1f,\"voc_index\":%d,\"nox_index\":%d}\r\n",
                 sensor_data->pm1_0 / 10.0f,
                 sensor_data->pm2_5 / 10.0f,
                 sensor_data->pm4_0 / 10.0f,
                 sensor_data->pm10 / 10.0f,
                 sensor_data->temperature / 200.0f,
                 sensor_data->humidity / 100.0f,
                 sensor_data->voc_index,
                 sensor_data->nox_index);
  
  // Send to USB CDC
  if (len > 0 && len < sizeof(usb_buffer))
  {
    CDC_Transmit_FS((uint8_t*)usb_buffer, len);
  }
}

void P2PS_APP_SEN55_Action(void)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_SEN55_I2C_EVT_ID, CFG_SCH_PRIO_0);

  return;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
