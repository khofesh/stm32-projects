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
#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "itg3200.h"
#include "vcm5883l.h"
#include "driver_adxl345.h"
#include "driver_adxl345_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the Nucleo management console\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char readBuf[1];
uint8_t txData;
__IO ITStatus UartReady = SET;
__IO ITStatus UartTxComplete = SET;
RingBuffer txBuf, rxBuf;

// handles
itg3200_t gyro;
vcm5883l_handle_t mag_sensor;
adxl345_handle_t adxl345_handle;

// data
int16_t gs_raw_test[20][3];
float gs_test[20][3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
  if (UART_Transmit(&huart2, (uint8_t *)ptr, len))
  {
    // Wait for transmission to complete to ensure log integrity
    while (UartTxComplete == RESET || RingBuffer_GetDataLength(&txBuf) > 0)
    {
      // Allow other interrupts to process
      __NOP();
    }
    return len;
  }
  return 0; // Return 0 on failure
}

uint8_t adxl345_basic_init(void);
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
	vcm5883l_vector_t data;
//	const char* direction;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  RingBuffer_Init(&txBuf);
  RingBuffer_Init(&rxBuf);

  /* init itg3200 */
  itg3200_init_default(&gyro, &hi2c1, ITG3200_ADDR_AD0_LOW);
  itg3200_zero_calibrate(&gyro, 50, 10);

  printf("itg3200 initialized\r\n");

  /* init vcm5883l */
  if (!vcm5883l_init(&mag_sensor, &hi2c1))
  {
	  printf("vcml5883l initialization failed!\r\n");
	  Error_Handler();
  }

  printf("vcml5883l initialized\r\n");

  /* init adxl345 */
  if (adxl345_basic_init() != 0)
  {
	  printf("adxl345 initialization failed!\r\n");
	  Error_Handler();
  }

  printf("adxl345 initialized\r\n");

  // configure for continuous mode
  vcm5883l_set_measurement_mode(&mag_sensor, VCM5883L_CONTINOUS);
  vcm5883l_set_data_rate(&mag_sensor, VCM5883L_DATARATE_200HZ);
  // set location's declination angle
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / PI);

  // Bekasi: Magnetic Declination +0° 40' (0.667°) East
  float declination_degrees = 0.0 + (40.0 / 60.0);  // 0.667 degrees
  float declination_radians = declination_degrees * M_PI / 180.0;
  vcm5883l_set_declination_angle(&mag_sensor, declination_radians);

  float gyro_x, gyro_y, gyro_z;
  char gyro_data_text[256];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  itg3200_read_gyro(&gyro, &gyro_x, &gyro_y, &gyro_z);

	  snprintf(gyro_data_text, sizeof(gyro_data_text),
			  "{"
			  "\"gyro_x\":%.2f,"
			  "\"gyro_y\":%.2f,"
			  "\"gyro_z\":%.2f"
			  "}\r\n",
			  gyro_x,
			  gyro_y,
			  gyro_z);

	  UART_Transmit(&huart2, (uint8_t*)gyro_data_text, strlen(gyro_data_text));

	  HAL_Delay(200);

	  // read magnetometer
	  data = vcm5883l_read_raw(&mag_sensor);
	  vcm5883l_get_heading_degrees(&mag_sensor);
	  printf("X: %d, Y: %d, Z: %d | Heading: %.2f°\r\n",
	                 data.x_axis, data.y_axis, data.z_axis,
	                 mag_sensor.vector.heading_degrees);

	  HAL_Delay(200);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00B07CB4;
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
  UartTxComplete = RESET; // Mark transmission as in progress
  if (HAL_UART_Transmit_IT(huart, pData, len) != HAL_OK)
  {
    if (RingBuffer_Write(&txBuf, pData, len) != RING_BUFFER_OK)
      return 0;
  }
  return 1;
}

uint8_t adxl345_basic_init(void)
{
	uint8_t res;

    /* link interface function */
    DRIVER_ADXL345_LINK_INIT(&adxl345_handle, adxl345_handle_t);
    DRIVER_ADXL345_LINK_IIC_INIT(&adxl345_handle, adxl345_interface_iic_init);
    DRIVER_ADXL345_LINK_IIC_DEINIT(&adxl345_handle, adxl345_interface_iic_deinit);
    DRIVER_ADXL345_LINK_IIC_READ(&adxl345_handle, adxl345_interface_iic_read);
    DRIVER_ADXL345_LINK_IIC_WRITE(&adxl345_handle, adxl345_interface_iic_write);
    DRIVER_ADXL345_LINK_DELAY_MS(&adxl345_handle, adxl345_interface_delay_ms);
    DRIVER_ADXL345_LINK_DEBUG_PRINT(&adxl345_handle, adxl345_interface_debug_print);
    DRIVER_ADXL345_LINK_RECEIVE_CALLBACK(&adxl345_handle, adxl345_interface_receive_callback);

    /* init adxl345 */
    res = adxl345_init(&adxl345_handle);
    if (res != 0)
    {
    	adxl345_interface_debug_print("adxl345: init failed.\r\n");
    	return 1;
    }

    return 0;
}

// uint8_t adxl345_read(adxl345_handle_t *handle, int16_t (*raw)[3], float (*g)[3], uint16_t *len)
uint8_t adxl345_basic_read()
{
	uint16_t len = 1;

	if (adxl345_read(&adxl345_handle, (int16_t (*)[3])gs_raw_test, (float (*)[3])gs_test, (uint16_t *)&len) != 0)
	{
		adxl345_interface_debug_print("adxl345: read failed.\r\n");
		return 1;
	}

	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  UartReady = SET;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    if (RingBuffer_GetDataLength(&txBuf) > 0)
    {
      RingBuffer_Read(&txBuf, &txData, 1);
      HAL_UART_Transmit_IT(huart, &txData, 1);
    }
    else
    {
      UartTxComplete = SET; // Mark transmission as complete when buffer is empty
    }
  }
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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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
