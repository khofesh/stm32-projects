/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "driver_aht30_interface.h"
#include "driver_ssd1315_interface.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLEEP_PERIOD_S   60U
#define OLED_W           128U
#define OLED_H           64U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
static aht30_handle_t g_aht30;
static ssd1315_handle_t g_oled;
static volatile uint8_t g_wakeup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t aht30_setup(void);
static uint8_t oled_setup(void);
static uint8_t oled_show(float t, uint8_t h);
static void enter_stop(uint32_t seconds);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  if (oled_setup() != 0)
  {
    Error_Handler();
  }
  if (aht30_setup() != 0)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

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

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t t_raw;
    uint32_t h_raw;
    float t_s;
    uint8_t h_s;

    if (aht30_read_temperature_humidity(&g_aht30, &t_raw, &t_s, &h_raw, &h_s) == 0)
    {
      (void)oled_show(t_s, h_s);
    }

    enter_stop(SLEEP_PERIOD_S);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x40B285C2;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 249;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static uint8_t aht30_setup(void)
{
  DRIVER_AHT30_LINK_INIT(&g_aht30, aht30_handle_t);
  DRIVER_AHT30_LINK_IIC_INIT(&g_aht30, aht30_interface_iic_init);
  DRIVER_AHT30_LINK_IIC_DEINIT(&g_aht30, aht30_interface_iic_deinit);
  DRIVER_AHT30_LINK_IIC_READ_CMD(&g_aht30, aht30_interface_iic_read_cmd);
  DRIVER_AHT30_LINK_IIC_WRITE_CMD(&g_aht30, aht30_interface_iic_write_cmd);
  DRIVER_AHT30_LINK_DELAY_MS(&g_aht30, aht30_interface_delay_ms);
  DRIVER_AHT30_LINK_DEBUG_PRINT(&g_aht30, aht30_interface_debug_print);

  return aht30_init(&g_aht30);
}

static uint8_t oled_setup(void)
{
  DRIVER_SSD1315_LINK_INIT(&g_oled, ssd1315_handle_t);
  DRIVER_SSD1315_LINK_IIC_INIT(&g_oled, ssd1315_interface_iic_init);
  DRIVER_SSD1315_LINK_IIC_DEINIT(&g_oled, ssd1315_interface_iic_deinit);
  DRIVER_SSD1315_LINK_IIC_WRITE(&g_oled, ssd1315_interface_iic_write);
  DRIVER_SSD1315_LINK_SPI_INIT(&g_oled, ssd1315_interface_spi_init);
  DRIVER_SSD1315_LINK_SPI_DEINIT(&g_oled, ssd1315_interface_spi_deinit);
  DRIVER_SSD1315_LINK_SPI_WRITE_COMMAND(&g_oled, ssd1315_interface_spi_write_cmd);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_INIT(&g_oled, ssd1315_interface_spi_cmd_data_gpio_init);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_DEINIT(&g_oled, ssd1315_interface_spi_cmd_data_gpio_deinit);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_WRITE(&g_oled, ssd1315_interface_spi_cmd_data_gpio_write);
  DRIVER_SSD1315_LINK_RESET_GPIO_INIT(&g_oled, ssd1315_interface_reset_gpio_init);
  DRIVER_SSD1315_LINK_RESET_GPIO_DEINIT(&g_oled, ssd1315_interface_reset_gpio_deinit);
  DRIVER_SSD1315_LINK_RESET_GPIO_WRITE(&g_oled, ssd1315_interface_reset_gpio_write);
  DRIVER_SSD1315_LINK_DELAY_MS(&g_oled, ssd1315_interface_delay_ms);
  DRIVER_SSD1315_LINK_DEBUG_PRINT(&g_oled, ssd1315_interface_debug_print);

  if (ssd1315_set_interface(&g_oled, SSD1315_INTERFACE_IIC) != 0)
  {
    return 1;
  }
  if (ssd1315_set_addr_pin(&g_oled, SSD1315_ADDR_SA0_0) != 0)
  {
    return 1;
  }
  if (ssd1315_init(&g_oled) != 0)
  {
    return 1;
  }
  if (ssd1315_clear(&g_oled) != 0)
  {
    return 1;
  }

  return ssd1315_set_display(&g_oled, SSD1315_DISPLAY_ON);
}

static uint8_t oled_show(float t, uint8_t h)
{
  char line[24];
  int32_t n;

  if (ssd1315_gram_fill_rect(&g_oled, 0, 0, OLED_W - 1U, OLED_H - 1U, 0) != 0)
  {
    return 1;
  }

  /* integer formatting: newlib-nano printf has no float support by default */
  int32_t tenths = (int32_t)((t * 10.0f) + (t >= 0.0f ? 0.5f : -0.5f));
  n = snprintf(line, sizeof(line), "T %ld.%ld C", (long)(tenths / 10), (long)(tenths % 10 < 0 ? -(tenths % 10) : tenths % 10));
  if (ssd1315_gram_write_string(&g_oled, 0, 8, line, (uint16_t)n, 1, SSD1315_FONT_16) != 0)
  {
    return 1;
  }

  n = snprintf(line, sizeof(line), "H %u %%", (unsigned int)h);
  if (ssd1315_gram_write_string(&g_oled, 0, 32, line, (uint16_t)n, 1, SSD1315_FONT_16) != 0)
  {
    return 1;
  }

  return ssd1315_gram_update(&g_oled);
}

/* Stop 1: RAM and peripheral registers retained, RTC wakeup timer running on LSI.
   The SSD1315 keeps its own GRAM powered, so the last reading stays on screen. */
static void enter_stop(uint32_t seconds)
{
  g_wakeup = 0;

  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds - 1U, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SuspendTick();

  do
  {
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
  } while (g_wakeup == 0U);

  /* HSE and PLL are stopped on Stop entry; SYSCLK comes back on HSI16 */
  SystemClock_Config();
  HAL_ResumeTick();

  if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_cb)
{
  if (hrtc_cb->Instance == RTC)
  {
    g_wakeup = 1;
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/**
  * @}
  */

/**
  * @}
  */

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
