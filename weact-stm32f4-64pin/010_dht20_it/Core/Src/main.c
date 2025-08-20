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
#include "driver_dht20.h"
#include "driver_dht20_interface.h"
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
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
RingBuffer txBuf, rxBuf;
dht20_handle_t dht20_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t dht20_basic_init(void);
uint8_t dht20_basic_deinit(void);
uint8_t dht20_basic_read(uint32_t *temperature_raw, float *temperature_s,
		uint32_t *humidity_raw, uint8_t *humidity_s);
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
	  uint32_t temperature_raw, humidity_raw;
	  float temperature_s;
	  uint8_t humidity_s;
	  uint8_t res;
	  uint32_t last_read_time = 0;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  RingBuffer_Init(&txBuf);
  RingBuffer_Init(&rxBuf);

  dht20_interface_debug_print("\r\n=== DHT20 Temperature & Humidity Sensor ===\r\n");
  dht20_interface_debug_print("Initializing DHT20...\r\n");

  /* init DHT20 */
  res = dht20_basic_init();
  if (res != 0)
  {
    dht20_interface_debug_print("DHT20 initialization failed!\r\n");
    Error_Handler();
  }

  dht20_interface_debug_print("DHT20 initialized successfully!\r\n");
  dht20_interface_debug_print("Starting measurements...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* read DHT20 every 2 seconds */
	  if (HAL_GetTick() - last_read_time >= 2000)
	  {
		  last_read_time = HAL_GetTick();

		  res = dht20_basic_read(&temperature_raw, &temperature_s, &humidity_raw, &humidity_s);

		  if (res == 0)
		  {
			  dht20_interface_debug_print("--- DHT20 Reading ---\r\n");
			  dht20_interface_debug_print("Temperature: %.2f°C (Raw: %lu)\r\n",
					  temperature_s, (unsigned long)temperature_raw);
			  dht20_interface_debug_print("Humidity: %d%% (Raw: %lu)\r\n",
					  humidity_s, (unsigned long)humidity_raw);
			  dht20_interface_debug_print("Timestamp: %lu ms\r\n\r\n",
					  (unsigned long)HAL_GetTick());
		  }
		  else
		  {
			  dht20_interface_debug_print("DHT20 read failed! Error code: %d\r\n\r\n", res);
		  }
	  }

	  HAL_Delay(10);
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

uint8_t dht20_basic_init(void)
{
    uint8_t res;

    /* link interface function */
    DRIVER_DHT20_LINK_INIT(&dht20_handle, dht20_handle_t);
    DRIVER_DHT20_LINK_IIC_INIT(&dht20_handle, dht20_interface_iic_init);
    DRIVER_DHT20_LINK_IIC_DEINIT(&dht20_handle, dht20_interface_iic_deinit);
    DRIVER_DHT20_LINK_IIC_READ_CMD(&dht20_handle, dht20_interface_iic_read_cmd);
    DRIVER_DHT20_LINK_IIC_WRITE_CMD(&dht20_handle, dht20_interface_iic_write_cmd);
    DRIVER_DHT20_LINK_DELAY_MS(&dht20_handle, dht20_interface_delay_ms);
    DRIVER_DHT20_LINK_DEBUG_PRINT(&dht20_handle, dht20_interface_debug_print);

    /* dht20 init */
    res = dht20_init(&dht20_handle);
    if (res != 0)
    {
        dht20_interface_debug_print("dht20: init failed.\n");
        return 1;
    }

    return 0;
}

uint8_t dht20_basic_deinit(void)
{
    uint8_t res;

    /* close dht20 */
    res = dht20_deinit(&dht20_handle);
    if (res != 0)
    {
        return 1;
    }

    return 0;
}

uint8_t dht20_basic_read(uint32_t *temperature_raw, float *temperature_s,
		uint32_t *humidity_raw, uint8_t *humidity_s)
{
    uint8_t res;

    /* read temperature and humidity */
    res = dht20_read_temperature_humidity(&dht20_handle, temperature_raw, temperature_s,
                                          humidity_raw, humidity_s);
    if (res != 0)
    {
        return 1;
    }

    return 0;
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
