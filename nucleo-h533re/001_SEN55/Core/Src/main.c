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
#include "sensirion_i2c_hal.h"
#include "sen5x_i2c.h"
#include "sensirion_common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <string.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Send(uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&huart1, data, length, HAL_MAX_DELAY);
}

void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
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
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  error = sen5x_device_reset();
  if (error) {
	    char msg[50];
	    sprintf(msg, "Error executing sen5x_device_reset(): %i\r\n", error);
	    UART_SendString(msg);
  }

  unsigned char serial_number[32];
  uint8_t serial_number_size = 32;
  error = sen5x_get_serial_number(serial_number, serial_number_size);
  if (error) {
	    char msg[60];
	    sprintf(msg, "Error executing sen5x_get_serial_number(): %i\r\n", error);
	    UART_SendString(msg);
  } else {
	    char msg[60];
	    sprintf(msg, "Serial number: %s\r\n", serial_number);
	    UART_SendString(msg);
  }

  unsigned char product_name[32];
  uint8_t product_name_size = 32;
  error = sen5x_get_product_name(product_name, product_name_size);
  if (error) {
	  char msg[60];
	  sprintf(msg, "Error executing sen5x_get_product_name(): %i\r\n", error);
	  UART_SendString(msg);
  } else {
	  char msg[60];
	  sprintf(msg, "Product name: %s\r\n", product_name);
	  UART_SendString(msg);
  }

  uint8_t firmware_major;
  uint8_t firmware_minor;
  bool firmware_debug;
  uint8_t hardware_major;
  uint8_t hardware_minor;
  uint8_t protocol_major;
  uint8_t protocol_minor;
  error = sen5x_get_version(&firmware_major, &firmware_minor, &firmware_debug,
                            &hardware_major, &hardware_minor, &protocol_major,
                            &protocol_minor);

  if (error) {
	  char msg[60];
	  sprintf(msg, "Error executing sen5x_get_version(): %i\r\n", error);
	  UART_SendString(msg);
  } else {
	  char msg[80];
	  sprintf(msg, "Firmware: %u.%u, Hardware: %u.%u\r\n", firmware_major,
			  firmware_minor, hardware_major, hardware_minor);
	  UART_SendString(msg);
  }

  // Adjust temp_offset in degrees celsius to account for additional
  // temperature offsets exceeding the SEN module's self heating.
  float temp_offset = 0.0f;
  int16_t default_slope = 0;
  uint16_t default_time_constant = 0;
  error = sen5x_set_temperature_offset_parameters(
      (int16_t)(200 * temp_offset), default_slope, default_time_constant);
  if (error) {
	    char msg[80];
	    sprintf(msg, "Error executing sen5x_set_temperature_offset_parameters(): %i\r\n", error);
	    UART_SendString(msg);
  } else {
	    char msg[80];
	    sprintf(msg, "Temperature Offset set to %.2f °C (SEN54/SEN55 only)\r\n", temp_offset);
	    UART_SendString(msg);
  }

  // start Measurement
  error = sen5x_start_measurement();
  if (error) {
      char msg[60];
      sprintf(msg, "Error executing sen5x_start_measurement(): %i\r\n", error);
      UART_SendString(msg);
  } else {
      UART_SendString("Sensor measurement started successfully\r\n");
  }


  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);

	  uint16_t mass_concentration_pm1p0;
	  uint16_t mass_concentration_pm2p5;
	  uint16_t mass_concentration_pm4p0;
	  uint16_t mass_concentration_pm10p0;
	  int16_t ambient_humidity;
	  int16_t ambient_temperature;
	  int16_t voc_index;
	  int16_t nox_index;

	  error = sen5x_read_measured_values(
			  &mass_concentration_pm1p0, &mass_concentration_pm2p5,
			  &mass_concentration_pm4p0, &mass_concentration_pm10p0,
			  &ambient_humidity, &ambient_temperature, &voc_index, &nox_index);

	  if (error) {
		  char msg[60];
		  sprintf(msg, "Error executing sen5x_read_measured_values(): %i\r\n", error);
		  UART_SendString(msg);
	  } else {
		  char msg[100];

		  // PM concentrations
		  sprintf(msg, "PM1.0: %.1f µg/m³\r\n", mass_concentration_pm1p0 / 10.0f);
		  UART_SendString(msg);

		  sprintf(msg, "PM2.5: %.1f µg/m³\r\n", mass_concentration_pm2p5 / 10.0f);
		  UART_SendString(msg);

		  sprintf(msg, "PM4.0: %.1f µg/m³\r\n", mass_concentration_pm4p0 / 10.0f);
		  UART_SendString(msg);

		  sprintf(msg, "PM10.0: %.1f µg/m³\r\n", mass_concentration_pm10p0 / 10.0f);
		  UART_SendString(msg);

		  // humidity
		  if (ambient_humidity == 0x7fff) {
			  UART_SendString("Humidity: n/a\r\n");
		  } else {
			  sprintf(msg, "Humidity: %.1f %%RH\r\n", ambient_humidity / 100.0f);
			  UART_SendString(msg);
		  }

		  // temperature
		  if (ambient_temperature == 0x7fff) {
			  UART_SendString("Temperature: n/a\r\n");
		  } else {
			  sprintf(msg, "Temperature: %.1f °C\r\n", ambient_temperature / 200.0f);
			  UART_SendString(msg);
		  }

		  // VOC index
		  if (voc_index == 0x7fff) {
			  UART_SendString("VOC index: n/a\r\n");
		  } else {
			  sprintf(msg, "VOC index: %.1f\r\n", voc_index / 10.0f);
			  UART_SendString(msg);
		  }

		  // NOx index
		  if (nox_index == 0x7fff) {
			  UART_SendString("NOx index: n/a\r\n");
		  } else {
			  sprintf(msg, "NOx index: %.1f\r\n", nox_index / 10.0f);
			  UART_SendString(msg);
		  }

		  UART_SendString("---\r\n"); // separator
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
