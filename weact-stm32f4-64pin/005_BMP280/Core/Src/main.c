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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver_bmp280.h"
#include "driver_bmp280_interface.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bmp280_handle_t bmp280_handle;
//static uint8_t tx_buffer[1000];
RingBuffer txBuf, rxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t bmp280_basic_init(void);
uint8_t bmp280_basic_read(float *temperature_c, float *pressure_pa);
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
	float temperature, pressure;
	uint8_t res;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  res = bmp280_basic_init();
  if (res != 0)
  {
    /* Initialization failed - handle error */
    bmp280_interface_debug_print("BMP280 initialization failed!\r\n");
    while(1);
  }

  bmp280_interface_debug_print("BMP280 initialized successfully!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  res = bmp280_basic_read(&temperature, &pressure);
	  if (res == 0)
	  {
		  bmp280_interface_debug_print("Temperature: %.2f°C, Pressure: %.2f Pa (%.2f hPa)\r\n",
				  temperature, pressure, pressure / 100.0f);
	  }
	  else
	  {
		  bmp280_interface_debug_print("failed to read sensor data!\n");
	  }

	  HAL_Delay(1000);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

uint8_t bmp280_basic_init(void)
{
    uint8_t res;
    bmp280_info_t info;

    // chip info
    res = bmp280_info(&info);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: get info failed.\r\n");
        return 1;
    }
    else
    {
        bmp280_interface_debug_print("\r\nbmp280: chip is %s.\r\n", info.chip_name);
        bmp280_interface_debug_print("bmp280: manufacturer is %s.\r\n", info.manufacturer_name);
        bmp280_interface_debug_print("bmp280: interface is %s.\r\n", info.interface);
        bmp280_interface_debug_print("bmp280: driver version is %d.%d.\r\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        bmp280_interface_debug_print("bmp280: min supply voltage is %0.1fV.\r\n", info.supply_voltage_min_v);
        bmp280_interface_debug_print("bmp280: max supply voltage is %0.1fV.\r\n", info.supply_voltage_max_v);
        bmp280_interface_debug_print("bmp280: max current is %0.2fmA.\r\n", info.max_current_ma);
        bmp280_interface_debug_print("bmp280: temperature range is %0.1fC - %0.1fC.\r\n", info.temperature_min, info.temperature_max);
    }

    // link interface functions
    DRIVER_BMP280_LINK_INIT(&bmp280_handle, bmp280_handle_t);
    DRIVER_BMP280_LINK_IIC_INIT(&bmp280_handle, bmp280_interface_iic_init);
    DRIVER_BMP280_LINK_IIC_DEINIT(&bmp280_handle, bmp280_interface_iic_deinit);
    DRIVER_BMP280_LINK_IIC_READ(&bmp280_handle, bmp280_interface_iic_read);
    DRIVER_BMP280_LINK_IIC_WRITE(&bmp280_handle, bmp280_interface_iic_write);
    DRIVER_BMP280_LINK_SPI_INIT(&bmp280_handle, bmp280_interface_spi_init);
    DRIVER_BMP280_LINK_SPI_DEINIT(&bmp280_handle, bmp280_interface_spi_deinit);
    DRIVER_BMP280_LINK_SPI_READ(&bmp280_handle, bmp280_interface_spi_read);
    DRIVER_BMP280_LINK_SPI_WRITE(&bmp280_handle, bmp280_interface_spi_write);
    DRIVER_BMP280_LINK_DELAY_MS(&bmp280_handle, bmp280_interface_delay_ms);
    DRIVER_BMP280_LINK_DEBUG_PRINT(&bmp280_handle, bmp280_interface_debug_print);

    // set interface to SPI
    res = bmp280_set_interface(&bmp280_handle, BMP280_INTERFACE_SPI);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set interface failed.\r\n");
        return 1;
    }

    // init
    res = bmp280_init(&bmp280_handle);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: init failed.\r\n");
        return 1;
    }

    // temperature oversampling
    res = bmp280_set_temperatue_oversampling(&bmp280_handle, BMP280_OVERSAMPLING_x2);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set temperature oversampling failed.\r\n");
        (void)bmp280_deinit(&bmp280_handle);
        return 1;
    }

    // pressure oversampling
    res = bmp280_set_pressure_oversampling(&bmp280_handle, BMP280_OVERSAMPLING_x16);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set pressure oversampling failed.\r\n");
        (void)bmp280_deinit(&bmp280_handle);
        return 1;
    }

    // standby time
    res = bmp280_set_standby_time(&bmp280_handle, BMP280_STANDBY_TIME_250_MS);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set standby time failed.\r\n");
        (void)bmp280_deinit(&bmp280_handle);
        return 1;
    }

    // filter
    res = bmp280_set_filter(&bmp280_handle, BMP280_FILTER_COEFF_16);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set filter failed.\r\n");
        (void)bmp280_deinit(&bmp280_handle);
        return 1;
    }

    // normal mode
    res = bmp280_set_mode(&bmp280_handle, BMP280_MODE_NORMAL);
    if (res != 0)
    {
        bmp280_interface_debug_print("\r\nbmp280: set mode failed.\r\n");
        (void)bmp280_deinit(&bmp280_handle);
        return 1;
    }

    bmp280_interface_debug_print("bmp280: init success.\r\n");
    return 0;
}

uint8_t bmp280_basic_read(float *temperature_c, float *pressure_pa)
{
    uint32_t temperature_raw;
    uint32_t pressure_raw;

    return bmp280_read_temperature_pressure(&bmp280_handle, &temperature_raw, temperature_c, &pressure_raw, pressure_pa);
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
