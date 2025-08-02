/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sgp40_i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "sensirion_gas_index_algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//static uint8_t tx_buffer[1000];
RingBuffer txBuf, rxBuf;
GasIndexAlgorithmParams voc_algorithm_params;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void interface_debug_print(const char *const fmt, ...);
void debug_i2c_status(void);
void init_voc_algorithm(void);
int32_t process_voc_measurement(uint16_t sraw_voc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int16_t error = 0;
	uint16_t serial_number[3];
	uint8_t serial_number_size = 3;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  debug_i2c_status();

  init_voc_algorithm();

  error = sgp40_get_serial_number(serial_number, serial_number_size);

  if (error) {
	  interface_debug_print("Error executing sgp40_get_serial_number(): %i\n", error);

	  return 1;
  } else {
	  // printf("Serial number: %" PRIu64 "\n",
	  //         (((uint64_t)serial_number[0]) << 32) |
	  //         (((uint64_t)serial_number[1]) << 16) |
	  //         ((uint64_t)serial_number[2]));
	  interface_debug_print("serial: 0x%04x%04x%04x\n", serial_number[0], serial_number[1],
			  serial_number[2]);
	  interface_debug_print("\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	    // Start Measurement

	    // Parameters for deactivated humidity compensation:
	    uint16_t default_rh = 0x8000;
	    uint16_t default_t = 0x6666;

	    uint16_t sraw_voc;



	    error = sgp40_measure_raw_signal(default_rh, default_t, &sraw_voc);
	    if (error) {
	    	interface_debug_print("Error executing sgp40_measure_raw_signal(): "
	    			"%i\r\n",
					error);
	    	continue;
	    }
	    else
	    {
	    	int32_t voc_index = process_voc_measurement(sraw_voc);
	    	interface_debug_print("Raw: %u, VOC Index: %ld\r\n", sraw_voc, voc_index);

	    	if (voc_index <= 100)
	    	{
	    		interface_debug_print("Air Quality: Good\r\n");
	    	}
	    	else if (voc_index <= 200)
	    	{
	    		interface_debug_print("Air Quality: Moderate\r\n");
	    	}
	    	else if (voc_index <= 300)
	    	{
	    		interface_debug_print("Air Quality: Poor\r\n");
	    	}
	    	else
	    	{
	    		interface_debug_print("Air Quality: Very Poor\r\n");
	    	}
	    }

	    sensirion_i2c_hal_sleep_usec(1000000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Transmit_IT(huart, pData, len) == HAL_OK)
    {
      return 1;
    }
  }

  // If UART is busy, store in ring buffer
  if (RingBuffer_Write(&txBuf, pData, len) == RING_BUFFER_OK)
  {
    return 1;
  }

  return 0;
}

void interface_debug_print(const char *const fmt, ...)
{
	char str[256];
	uint16_t len;
	va_list args;

    memset((char *)str, 0, sizeof(char) * 256);
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 1000);
}

void debug_i2c_status(void)
{
    interface_debug_print("\nI2C State: %d\r\n", hi2c1.State);
    interface_debug_print("I2C Error: 0x%08lx\r\n", hi2c1.ErrorCode);

    // test basic i2c communication
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, (0x59 << 1), 3, 1000);
    interface_debug_print("Device Ready Status: %d\r\n", status);

    if (status == HAL_OK) {
        interface_debug_print("SGP40 device found at address 0x59\r\n");
    } else {
        interface_debug_print("SGP40 device NOT found at address 0x59\r\n");
        interface_debug_print("HAL Error: 0x%08lx\r\n", hi2c1.ErrorCode);
    }
}

void init_voc_algorithm(void)
{
    // init VOC measurement
    GasIndexAlgorithm_init(&voc_algorithm_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    interface_debug_print("VOC Algorithm initialized\r\n");
}

int32_t process_voc_measurement(uint16_t sraw_voc)
{
    int32_t voc_index_value;

    // convert uint16_t to int32_t as required by the algorithm
    int32_t voc_raw_value = (int32_t)sraw_voc;

    // process the raw value to get VOC index (0-500)
    GasIndexAlgorithm_process(&voc_algorithm_params, voc_raw_value, &voc_index_value);

    return voc_index_value;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
