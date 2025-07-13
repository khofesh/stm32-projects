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
#include "driver_st7789.h"
#include "driver_st7789_interface.h"
#include <stdlib.h>
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
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
st7789_handle_t st7789_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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

	char test_str3[] ="TEST ST7789 + STM32";

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  st7789_interface_debug_print("Starting ST7789 initialization...\r\n");

  st7789_handle_t gs_handle;
  uint8_t res;

  // Link interface functions
  DRIVER_ST7789_LINK_INIT(&gs_handle, st7789_handle_t);
  DRIVER_ST7789_LINK_SPI_INIT(&gs_handle, st7789_interface_spi_init);
  DRIVER_ST7789_LINK_SPI_DEINIT(&gs_handle, st7789_interface_spi_deinit);
  DRIVER_ST7789_LINK_SPI_WRITE_COMMAND(&gs_handle, st7789_interface_spi_write_cmd);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_INIT(&gs_handle, st7789_interface_cmd_data_gpio_init);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_DEINIT(&gs_handle, st7789_interface_cmd_data_gpio_deinit);
  DRIVER_ST7789_LINK_COMMAND_DATA_GPIO_WRITE(&gs_handle, st7789_interface_cmd_data_gpio_write);
  DRIVER_ST7789_LINK_RESET_GPIO_INIT(&gs_handle, st7789_interface_reset_gpio_init);
  DRIVER_ST7789_LINK_RESET_GPIO_DEINIT(&gs_handle, st7789_interface_reset_gpio_deinit);
  DRIVER_ST7789_LINK_RESET_GPIO_WRITE(&gs_handle, st7789_interface_reset_gpio_write);
  DRIVER_ST7789_LINK_DELAY_MS(&gs_handle, st7789_interface_delay_ms);
  DRIVER_ST7789_LINK_DEBUG_PRINT(&gs_handle, st7789_interface_debug_print);

  // Initialize the display
  st7789_interface_debug_print("Calling st7789_init()...\r\n");
  res = st7789_init(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("ST7789 init failed: %d\r\n", res);
      Error_Handler();
  }

  // wait
  st7789_interface_delay_ms(120);
  // turn on the display
  st7789_display_on(&gs_handle);

  // set display parameters (240x240 for 1.54" ST7789)
  st7789_interface_debug_print("Setting display size to 240x240...\r\n");
  res = st7789_set_column(&gs_handle, 240);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set column failed.\n");
      Error_Handler();
  }

  res = st7789_set_row(&gs_handle, 240);
  if (res != 0)
  {
	  st7789_interface_debug_print("st7789: set row failed.\n");
	  Error_Handler();
  }

  /* sleep out */
  res = st7789_sleep_out(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: sleep out failed.\n");
      Error_Handler();
  }

  /* idle mode off */
  res = st7789_idle_mode_off(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: idle mode off failed.\n");
      Error_Handler();
  }

  /* normal display mode on */
  res = st7789_normal_display_mode_on(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: normal display mode on failed.\n");
      (void)st7789_deinit(&gs_handle);

      return 1;
  }

  /* display inversion on */
  res = st7789_display_inversion_on(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: display inversion on failed.\n");
      Error_Handler();
  }

  /* set gamma */
  res = st7789_set_gamma(&gs_handle, ST7789_GAMMA_CURVE_1);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set gamma failed.\n");
      Error_Handler();
  }

  /* set memory data access control */
  res = st7789_set_memory_data_access_control(&gs_handle, ST7789_ORDER_PAGE_TOP_TO_BOTTOM | ST7789_ORDER_COLUMN_LEFT_TO_RIGHT |
                                                          ST7789_ORDER_PAGE_COLUMN_NORMAL | ST7789_ORDER_LINE_TOP_TO_BOTTOM |
                                                          ST7789_ORDER_COLOR_RGB | ST7789_ORDER_REFRESH_LEFT_TO_RIGHT);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set memory data access control failed.\n");
      Error_Handler();
  }

  // set pixel format to RGB565 (16-bit color)
  st7789_interface_debug_print("Setting pixel format to RGB565...\r\n");
  res = st7789_set_interface_pixel_format(&gs_handle, ST7789_RGB_INTERFACE_COLOR_FORMAT_262K, ST7789_CONTROL_INTERFACE_COLOR_FORMAT_16_BIT);
  if (res != 0)
  {
      st7789_interface_debug_print("ST7789 set pixel format failed: %d\r\n", res);
      Error_Handler();
  }

  res = st7789_set_display_brightness(&gs_handle, 0xFF);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set display brightness failed.\n");
      Error_Handler();
  }

  /* disable brightness control */
  res = st7789_set_display_control(&gs_handle, ST7789_BOOL_FALSE, ST7789_BOOL_FALSE, ST7789_BOOL_FALSE);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set display control failed.\n");
      Error_Handler();
  }

  /* enable color enhancement */
  res = st7789_set_brightness_control_and_color_enhancement(&gs_handle, ST7789_BOOL_TRUE,
                                                            ST7789_COLOR_ENHANCEMENT_MODE_USER_INTERFACE, ST7789_COLOR_ENHANCEMENT_LEVEL_HIGH);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set brightness control and color enhancement failed.\n");
      Error_Handler();
  }

  /* set 0x00 */
  res = st7789_set_cabc_minimum_brightness(&gs_handle, 0x00);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set cabc minimum brightness failed.\n");
      Error_Handler();
  }

  st7789_interface_debug_print("Turning display on...\r\n");
  res = st7789_display_on(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("ST7789 display on failed: %d\r\n", res);
      Error_Handler();
  }

  // Set memory access control (orientation and color order)
  res = st7789_set_ram_control(&gs_handle,
                               ST7789_RAM_ACCESS_MCU,
                               ST7789_DISPLAY_MODE_MCU,
                               ST7789_FRAME_TYPE_0,
                               ST7789_DATA_MODE_MSB,
                               ST7789_RGB_BUS_WIDTH_18_BIT,
                               ST7789_PIXEL_TYPE_0);
  if (res != 0)
  {
      st7789_interface_debug_print("st7789: set ram control failed.\n");


      return 1;
  }

  // Clear the display
  st7789_interface_debug_print("Clearing display...\r\n");
  res = st7789_clear(&gs_handle);
  if (res != 0)
  {
      st7789_interface_debug_print("ST7789 clear failed: %d\r\n", res);
  }

  st7789_interface_debug_print("ST7789 initialized successfully!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    /* rand point test */
	    st7789_interface_debug_print("st7789: rand point test.\n");

	    for (int i = 0; i < 240; i++)
	    {
	        for (int j = 0; j < 240; j++)
	        {
	            if ((rand() % 2) != 0)
	            {
	                res = st7789_draw_point(&gs_handle, i, j, 0xFFFFU);
	                if (res != 0)
	                {
	                    st7789_interface_debug_print("st7789: draw point failed.\n");


	                    return 1;
	                }
	            }
	        }
	    }

	    /* delay 1000ms */
	    st7789_interface_delay_ms(1000);

	    /* clear */
	    res = st7789_clear(&gs_handle);
	    if (res != 0)
	    {
	        st7789_interface_debug_print("st7789: clear failed.\n");


	        return 1;
	    }

	    /* write string */
	    res = st7789_write_string(&gs_handle, 20, 100, test_str3, (uint16_t)strlen(test_str3), 0xFFFFU, ST7789_FONT_16);
	    if (res != 0)
	    {
	        st7789_interface_debug_print("st7789: write string failed.\n");
	        (void)st7789_deinit(&gs_handle);

	        return 1;
	    }

	    /* delay 1000ms */
	    st7789_interface_delay_ms(1000);

	    /* clear */
	    res = st7789_clear(&gs_handle);
	    if (res != 0)
	    {
	        st7789_interface_debug_print("st7789: clear failed.\n");


	        return 1;
	    }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin|LCD_RST_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin PA4 */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
