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
#include "dfrobot_air_quality_sensor.h"
#include <stdio.h>
#include <string.h>
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

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DFRobot_AirQualitySensor_t air_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void air_sensor_power_test(void);
void UART_Send(uint8_t *data, uint16_t length)
{
	HAL_UART_Transmit(&huart2, data, length, HAL_MAX_DELAY);
}

void UART_SendString(char *str)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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
	MX_I2C3_Init();
	MX_ICACHE_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	if (!DFRobot_AirSensor_Init(&air_sensor, &hi2c3,
								DFROBOT_AIR_SENSOR_DEFAULT_ADDR))
	{
		UART_SendString("failed to initialize the sensor\r\n");
		return 1;
	}

	if (!DFRobot_AirSensor_Begin(&air_sensor))
	{
		UART_SendString("failed to communicate with air quality sensor\r\n");
		return 1;
	}

	air_sensor_power_test();

	// get firmware version
	uint8_t version = DFRobot_AirSensor_GetVersion(&air_sensor);
	char version_buffer[50];
	sprintf(version_buffer, "Sensor firmware version: %d\r\n", version);
	UART_SendString(version_buffer);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		UART_SendString("\r\n=== Air Quality Measurements ===\r\n");

		// read PM concentrations (µg/m³)
		uint16_t pm1_0_std = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM1_0_STANDARD);
		uint16_t pm2_5_std = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM2_5_STANDARD);
		uint16_t pm10_std = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM10_STANDARD);

		uint16_t pm1_0_atm = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM1_0_ATMOSPHERE);
		uint16_t pm2_5_atm = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM2_5_ATMOSPHERE);
		uint16_t pm10_atm = DFRobot_AirSensor_GetParticleConcentration_ugm3(
			&air_sensor, PARTICLE_PM10_ATMOSPHERE);

		UART_SendString("PM Concentrations (µg/m³):\r\n");
		UART_SendString("  Standard conditions:\r\n");

		char pm_buffer[80];
		sprintf(pm_buffer, "    PM1.0: %d µg/m³\r\n", pm1_0_std);
		UART_SendString(pm_buffer);

		sprintf(pm_buffer, "    PM2.5: %d µg/m³\r\n", pm2_5_std);
		UART_SendString(pm_buffer);

		sprintf(pm_buffer, "    PM10:  %d µg/m³\r\n", pm10_std);
		UART_SendString(pm_buffer);

		UART_SendString("  Atmospheric conditions:\r\n");

		sprintf(pm_buffer, "    PM1.0: %d µg/m³\r\n", pm1_0_atm);
		UART_SendString(pm_buffer);

		sprintf(pm_buffer, "    PM2.5: %d µg/m³\r\n", pm2_5_atm);
		UART_SendString(pm_buffer);

		sprintf(pm_buffer, "    PM10:  %d µg/m³\r\n", pm10_atm);
		UART_SendString(pm_buffer);

		// read particle counts (per 0.1L air)
		uint16_t count_0_3 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
		uint16_t count_0_5 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_0_5_UM_EVERY0_1L_AIR);
		uint16_t count_1_0 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_1_0_UM_EVERY0_1L_AIR);
		uint16_t count_2_5 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_2_5_UM_EVERY0_1L_AIR);
		uint16_t count_5_0 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_5_0_UM_EVERY0_1L_AIR);
		uint16_t count_10 = DFRobot_AirSensor_GetParticleNum_Every0_1L(
			&air_sensor, PARTICLENUM_10_UM_EVERY0_1L_AIR);

		UART_SendString("\r\nParticle Counts (per 0.1L air):\r\n");

		char count_buffer[80];
		sprintf(count_buffer, "  ≥0.3µm: %d particles\r\n", count_0_3);
		UART_SendString(count_buffer);

		sprintf(count_buffer, "  ≥0.5µm: %d particles\r\n", count_0_5);
		UART_SendString(count_buffer);

		sprintf(count_buffer, "  ≥1.0µm: %d particles\r\n", count_1_0);
		UART_SendString(count_buffer);

		sprintf(count_buffer, "  ≥2.5µm: %d particles\r\n", count_2_5);
		UART_SendString(count_buffer);

		sprintf(count_buffer, "  ≥5.0µm: %d particles\r\n", count_5_0);
		UART_SendString(count_buffer);

		sprintf(count_buffer, "  ≥10µm:  %d particles\r\n", count_10);
		UART_SendString(count_buffer);

		// delay 5seconds
		HAL_Delay(5000);
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

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
	{
	}

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
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
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00707CBB;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_Pin_GPIO_Port, Led_Pin_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Key_Pin_Pin */
	GPIO_InitStruct.Pin = Key_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Key_Pin_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Led_Pin_Pin */
	GPIO_InitStruct.Pin = Led_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_Pin_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_Detect_Pin_Pin */
	GPIO_InitStruct.Pin = SD_Detect_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(SD_Detect_Pin_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void air_sensor_power_test(void)
{
	printf("\r\n=== Power Management Test ===\r\n");

	// put sensor to sleep
	if (DFRobot_AirSensor_SetLowPower(&air_sensor))
	{
		printf("Sensor set to low power mode\r\n");
	}

	HAL_Delay(3000);

	// wake up sensor
	if (DFRobot_AirSensor_Awake(&air_sensor))
	{
		printf("Sensor awakened from low power mode\r\n");
	}

	// allow time for sensor to stabilize after wake-up
	HAL_Delay(2000);
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
