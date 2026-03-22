/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "b_u585i_iot02a_env_sensors.h"
#include "b_u585i_iot02a_motion_sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENV_SENS_THREAD_STACK_SIZE       1024
#define MOTION_THREAD_STACK_SIZE         1024
#define TOF_GESTURE_THREAD_STACK_SIZE 	 1024
#define LIGHT_THREAD_STACK_SIZE			 1024 // light sensor
#define ENV_SENS_THREAD_PRIORITY      	 10
#define MOTION_THREAD_PRIORITY           11
#define TOF_GESTURE_THREAD_PRIORITY      12
#define LIGHT_THREAD_PRIORITY			 13
#define THREAD_SYNC_EVENT                0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static TX_THREAD env_sens_thread;
static TX_THREAD motion_thread;
static TX_THREAD tof_gesture_thread;
static TX_THREAD light_thread;
static TX_EVENT_FLAGS_GROUP sync_event_flags;

static UCHAR env_sens_thread_stack[ENV_SENS_THREAD_STACK_SIZE];
static UCHAR motion_thread_stack[MOTION_THREAD_STACK_SIZE];
static UCHAR tof_gesture_thread_stack[TOF_GESTURE_THREAD_STACK_SIZE];
static UCHAR light_thread_stack[LIGHT_THREAD_STACK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void env_sens_thread_entry(ULONG thread_input);
static void motion_thread_entry(ULONG thread_input);
static void tof_gesture_entry(ULONG thread_input);
static void light_sensor_entry(ULONG thread_input);
static int32_t sensors_init(void);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void)memory_ptr;
  /* USER CODE END App_ThreadX_MEM_POOL */
  /* USER CODE BEGIN App_ThreadX_Init */

  /* Initialize sensors */
  if (sensors_init() != 0)
  {
    printf("Sensors initialization failed!\r\n");
    return TX_NOT_DONE;
  }
  printf("Sensors initialized successfully\r\n");

  /* Create event flags group for thread synchronization */
  ret = tx_event_flags_create(&sync_event_flags, "Sync Event Flags");
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* Create Temperature Thread (higher priority, runs first) */
  ret = tx_thread_create(&env_sens_thread,
                         "Temperature Thread",
						 env_sens_thread_entry,
                         0,
						 env_sens_thread_stack,
						 ENV_SENS_THREAD_STACK_SIZE,
						 ENV_SENS_THREAD_PRIORITY,
						 ENV_SENS_THREAD_PRIORITY,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* Create Motion Thread (lower priority, runs after temperature thread) */
  ret = tx_thread_create(&motion_thread,
                         "Motion Thread",
                         motion_thread_entry,
                         0,
                         motion_thread_stack,
                         MOTION_THREAD_STACK_SIZE,
                         MOTION_THREAD_PRIORITY,
                         MOTION_THREAD_PRIORITY,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
    return ret;
  }

  /* create TOF & gesture thread */
  ret = tx_thread_create(&tof_gesture_thread,
		  "TOF Gesture Thread",
		  tof_gesture_entry,
		  0,
		  tof_gesture_thread_stack,
		  TOF_GESTURE_THREAD_STACK_SIZE,
		  TOF_GESTURE_THREAD_PRIORITY,
		  TOF_GESTURE_THREAD_PRIORITY,
		  TX_NO_TIME_SLICE,
		  TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
	  return ret;
  }

  /* create light sensor thread */
  ret = tx_thread_create(&light_thread,
		  "Light Sensor Thread",
		  light_sensor_entry,
		  0,
		  light_thread_stack,
		  LIGHT_THREAD_STACK_SIZE,
		  LIGHT_THREAD_PRIORITY,
		  LIGHT_THREAD_PRIORITY,
		  TX_NO_TIME_SLICE,
		  TX_AUTO_START);
  if (ret != TX_SUCCESS)
  {
	  return ret;
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Initialize environmental and motion sensors
  * @retval 0 on success, non-zero on failure
  */
static int32_t sensors_init(void)
{
  int32_t ret = 0;

  /* Initialize HTS221 for temperature and humidity (Instance 0) */
  ret = BSP_ENV_SENSOR_Init(0, ENV_TEMPERATURE | ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    printf("HTS221 init failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable temperature on HTS221 */
  ret = BSP_ENV_SENSOR_Enable(0, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    printf("HTS221 temperature enable failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable humidity on HTS221 */
  ret = BSP_ENV_SENSOR_Enable(0, ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    printf("HTS221 humidity enable failed: %ld\r\n", ret);
    return ret;
  }

  /* Initialize LPS22HH for pressure (Instance 1) */
  ret = BSP_ENV_SENSOR_Init(1, ENV_TEMPERATURE | ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    printf("LPS22HH init failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable pressure on LPS22HH */
  ret = BSP_ENV_SENSOR_Enable(1, ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    printf("LPS22HH pressure enable failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable temp on LPS22HH */
  ret = BSP_ENV_SENSOR_Enable(1, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    printf("LPS22HH temperature enable failed: %ld\r\n", ret);
    return ret;
  }

  /* Initialize ISM330DHCX for accelerometer and gyroscope (Instance 0) */
  ret = BSP_MOTION_SENSOR_Init(0, MOTION_GYRO | MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    printf("ISM330DHCX init failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable accelerometer */
  ret = BSP_MOTION_SENSOR_Enable(0, MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    printf("Accelerometer enable failed: %ld\r\n", ret);
    return ret;
  }

  /* Enable gyroscope */
  ret = BSP_MOTION_SENSOR_Enable(0, MOTION_GYRO);
  if (ret != BSP_ERROR_NONE)
  {
    printf("Gyroscope enable failed: %ld\r\n", ret);
    return ret;
  }

  return 0;
}

/**
  * @brief  environment Thread entry function
  * @param  thread_input: not used
  * @retval None
  */
static void env_sens_thread_entry(ULONG thread_input)
{
  (void)thread_input;
  float temperature_value = 0.0f;
  float humidity_value = 0.0f;
  float pressure_value = 0.0f;
  float lps22hh_temp = 0.0f;

  printf("\r\n=== Sensor Threads Started ===\r\n");

  while (1)
  {
    /* Note: Both HTS221 and LPS22HH on-board temperature sensors read ~10C higher
     * than ambient due to self-heating from the STM32U585 MCU and board components.
     * For accurate ambient temperature, use an external sensor or apply a calibration offset.
     */

    /* Read temperature from HTS221 (Instance 0) */
    if (BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &temperature_value) != BSP_ERROR_NONE)
    {
      printf("[Thread 1] Temperature read error\r\n");
    }

    /* Read humidity from HTS221 (Instance 0) */
    if (BSP_ENV_SENSOR_GetValue(0, ENV_HUMIDITY, &humidity_value) != BSP_ERROR_NONE)
    {
      printf("[Thread 1] Humidity read error\r\n");
    }

    /* Read pressure from LPS22HH (Instance 1) */
    if (BSP_ENV_SENSOR_GetValue(1, ENV_PRESSURE, &pressure_value) != BSP_ERROR_NONE)
    {
      printf("[Thread 1] Pressure read error\r\n");
    }

    /* Read temperature from LPS22HH (Instance 1) */
    if (BSP_ENV_SENSOR_GetValue(1, ENV_TEMPERATURE, &lps22hh_temp) != BSP_ERROR_NONE)
    {
      printf("[Thread 1] LPS22HH Temp read error\r\n");
    }

    printf("[Thread 1] temp: %.2f C, hum: %.2f %%RH, press: %.2f hPa\r\n", temperature_value, humidity_value, pressure_value);
    printf("[Thread 1] lps22hh_temp: %.2f C\r\n", lps22hh_temp);

    /* Set event flag to signal motion thread */
    tx_event_flags_set(&sync_event_flags, THREAD_SYNC_EVENT, TX_OR);

    /* Sleep for 1 second */
    tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
  }
}

/**
  * @brief  Motion Thread entry function (gyroscope and accelerometer)
  * @param  thread_input: not used
  * @retval None
  */
static void motion_thread_entry(ULONG thread_input)
{
  (void)thread_input;
  ULONG actual_flags;
  BSP_MOTION_SENSOR_Axes_t gyro_axes;
  BSP_MOTION_SENSOR_Axes_t accel_axes;

  /*
  mg (milli-g): Unit for accelerometer output
  - 1g = Earth's gravitational acceleration (~9.81 m/s²)
  - 1 mg = 0.001g = ~0.00981 m/s²
  - At rest, you'd expect ~0 mg on X/Y axes and ~1000 mg on Z axis (if board is flat)

  mdps (milli-degrees per second): Unit for gyroscope output
  - Measures angular velocity (rotation speed)
  - 1 mdps = 0.001 degrees/second
  - At rest, values should be near 0; when rotating, you'll see larger values
   */

  while (1)
  {
    /* Wait for temperature thread to complete */
    tx_event_flags_get(&sync_event_flags, THREAD_SYNC_EVENT, TX_AND_CLEAR, &actual_flags, TX_WAIT_FOREVER);

    /* Read gyroscope from ISM330DHCX (Instance 0) */
    if (BSP_MOTION_SENSOR_GetAxes(0, MOTION_GYRO, &gyro_axes) == BSP_ERROR_NONE)
    {
      printf("[Thread 2] Gyro X: %ld, Y: %ld, Z: %ld mdps\r\n",
             gyro_axes.xval, gyro_axes.yval, gyro_axes.zval);
    }
    else
    {
      printf("[Thread 2] Gyroscope read error\r\n");
    }

    /* Read accelerometer from ISM330DHCX (Instance 0) */
    if (BSP_MOTION_SENSOR_GetAxes(0, MOTION_ACCELERO, &accel_axes) == BSP_ERROR_NONE)
    {
      printf("[Thread 2] Accel X: %ld, Y: %ld, Z: %ld mg\r\n",
             accel_axes.xval, accel_axes.yval, accel_axes.zval);
    }
    else
    {
      printf("[Thread 2] Accelerometer read error\r\n");
    }

    printf("---\r\n");
  }
}

static void tof_gesture_entry(ULONG thread_input)
{

}

static void light_sensor_entry(ULONG thread_input)
{

}
/* USER CODE END 1 */
