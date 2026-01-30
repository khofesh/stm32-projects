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
#include "power_management.h"

#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>
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
char readBuf[1];
uint8_t txData;
__IO ITStatus UartReady = SET;
RingBuffer txBuf, rxBuf;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
static aht30_handle_t gs_handle;
static ssd1315_handle_t gs_lcd_handle;

/* Power mode selection - change this to adjust power consumption */
#define SELECTED_POWER_MODE  POWER_MODE_LOW_POWER

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t UART_Transmit(UART_HandleTypeDef *lpuart, uint8_t *pData, uint16_t len);
void SystemClock_Config_AfterStop(void);
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
    uint8_t res;
    uint32_t temperature_raw;
    uint32_t humidity_raw;
    float temperature;
    uint8_t humidity;
    aht30_info_t info;

    uint8_t lcd_res;
    ssd1315_info_t lcd_info;
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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize power management */
  Power_Init(&hrtc);
  Power_SetMode(SELECTED_POWER_MODE);
  
  /* Enable RTC wakeup interrupt */
  HAL_NVIC_SetPriority(RTC_TAMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_TAMP_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DRIVER_AHT30_LINK_INIT(&gs_handle, aht30_handle_t);
  DRIVER_AHT30_LINK_IIC_INIT(&gs_handle, aht30_interface_iic_init);
  DRIVER_AHT30_LINK_IIC_DEINIT(&gs_handle, aht30_interface_iic_deinit);
  DRIVER_AHT30_LINK_IIC_READ_CMD(&gs_handle, aht30_interface_iic_read_cmd);
  DRIVER_AHT30_LINK_IIC_WRITE_CMD(&gs_handle, aht30_interface_iic_write_cmd);
  DRIVER_AHT30_LINK_DELAY_MS(&gs_handle, aht30_interface_delay_ms);
  DRIVER_AHT30_LINK_DEBUG_PRINT(&gs_handle, aht30_interface_debug_print);

  /* get aht30 information */
  res = aht30_info(&info);
  if (res != 0)
  {
      aht30_interface_debug_print("aht30: get info failed.\n");

      return 1;
  }
  else
  {
      /* print aht30 information */
      aht30_interface_debug_print("aht30: chip is %s.\n", info.chip_name);
      aht30_interface_debug_print("aht30: manufacturer is %s.\n", info.manufacturer_name);
      aht30_interface_debug_print("aht30: interface is %s.\n", info.interface);
      aht30_interface_debug_print("aht30: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
      aht30_interface_debug_print("aht30: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
      aht30_interface_debug_print("aht30: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
      aht30_interface_debug_print("aht30: max current is %0.2fmA.\n", info.max_current_ma);
      aht30_interface_debug_print("aht30: max temperature is %0.1fC.\n", info.temperature_max);
      aht30_interface_debug_print("aht30: min temperature is %0.1fC.\n", info.temperature_min);
  }

  /* start basic read test */
  aht30_interface_debug_print("aht30: start read test.\n");

  /* aht30 init */
  res = aht30_init(&gs_handle);
  if (res != 0)
  {
      aht30_interface_debug_print("aht30: init failed.\n");

      return 1;
  }

  /* delay 2000 ms for read */
  aht30_interface_delay_ms(2000);

  // LCD
  DRIVER_SSD1315_LINK_INIT(&gs_lcd_handle, ssd1315_handle_t);
  DRIVER_SSD1315_LINK_IIC_INIT(&gs_lcd_handle, ssd1315_interface_iic_init);
  DRIVER_SSD1315_LINK_IIC_DEINIT(&gs_lcd_handle, ssd1315_interface_iic_deinit);
  DRIVER_SSD1315_LINK_IIC_WRITE(&gs_lcd_handle, ssd1315_interface_iic_write);

  DRIVER_SSD1315_LINK_SPI_INIT(&gs_lcd_handle, ssd1315_interface_spi_init);
  DRIVER_SSD1315_LINK_SPI_DEINIT(&gs_lcd_handle, ssd1315_interface_spi_deinit);
  DRIVER_SSD1315_LINK_SPI_WRITE_COMMAND(&gs_lcd_handle, ssd1315_interface_spi_write_cmd);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_INIT(&gs_lcd_handle, ssd1315_interface_spi_cmd_data_gpio_init);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_DEINIT(&gs_lcd_handle, ssd1315_interface_spi_cmd_data_gpio_deinit);
  DRIVER_SSD1315_LINK_SPI_COMMAND_DATA_GPIO_WRITE(&gs_lcd_handle, ssd1315_interface_spi_cmd_data_gpio_write);

  DRIVER_SSD1315_LINK_RESET_GPIO_INIT(&gs_lcd_handle, ssd1315_interface_reset_gpio_init);
  DRIVER_SSD1315_LINK_RESET_GPIO_DEINIT(&gs_lcd_handle, ssd1315_interface_reset_gpio_deinit);
  DRIVER_SSD1315_LINK_RESET_GPIO_WRITE(&gs_lcd_handle, ssd1315_interface_reset_gpio_write);
  DRIVER_SSD1315_LINK_DELAY_MS(&gs_lcd_handle, ssd1315_interface_delay_ms);
  DRIVER_SSD1315_LINK_DEBUG_PRINT(&gs_lcd_handle, ssd1315_interface_debug_print);

  // Set I2C mode and address (0x78 for SA0=GND, 0x7A for SA0=VCC)
  ssd1315_set_interface(&gs_lcd_handle, SSD1315_INTERFACE_IIC);
  ssd1315_set_addr_pin(&gs_lcd_handle, SSD1315_ADDR_SA0_0);
  /* ssd1315 info */
  lcd_res = ssd1315_info(&lcd_info);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: get info failed.\n");

      return 1;
  }

  /* ssd1315 init */
  lcd_res = ssd1315_init(&gs_lcd_handle);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: init failed.\n");

      return 1;
  }

  /* close display */
  lcd_res = ssd1315_set_display(&gs_lcd_handle, SSD1315_DISPLAY_OFF);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set column address range */
  lcd_res = ssd1315_set_column_address_range(&gs_lcd_handle, 0x00, 0x7F);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set column address range failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set page address range */
  lcd_res = ssd1315_set_page_address_range(&gs_lcd_handle, 0x00, 0x07);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set page address range failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set low column start address */
  lcd_res = ssd1315_set_low_column_start_address(&gs_lcd_handle, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set low column start address failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set high column start address */
  lcd_res = ssd1315_set_high_column_start_address(&gs_lcd_handle, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set high column start address failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set display start line */
  lcd_res = ssd1315_set_display_start_line(&gs_lcd_handle, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display start line failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set fade blinking mode */
  lcd_res = ssd1315_set_fade_blinking_mode(&gs_lcd_handle, SSD1315_FADE_BLINKING_MODE_DISABLE, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set fade blinking failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* deactivate scroll */
  lcd_res = ssd1315_deactivate_scroll(&gs_lcd_handle);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set deactivate scroll failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set zoom in */
  lcd_res = ssd1315_set_zoom_in(&gs_lcd_handle, SSD1315_ZOOM_IN_DISABLE);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set set zoom in failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set contrast based on power mode */
  lcd_res = ssd1315_set_contrast(&gs_lcd_handle, Power_GetLCDContrast());
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set contrast failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set segment remap */
  lcd_res = ssd1315_set_segment_remap(&gs_lcd_handle, SSD1315_SEGMENT_COLUMN_ADDRESS_127);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set segment remap failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set scan direction */
  lcd_res = ssd1315_set_scan_direction(&gs_lcd_handle, SSD1315_SCAN_DIRECTION_COMN_1_START);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set scan direction failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set display mode */
  lcd_res = ssd1315_set_display_mode(&gs_lcd_handle, SSD1315_DISPLAY_MODE_NORMAL);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display mode failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set multiplex ratio */
  lcd_res = ssd1315_set_multiplex_ratio(&gs_lcd_handle, 0x3F);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set multiplex ratio failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set display offset */
  lcd_res = ssd1315_set_display_offset(&gs_lcd_handle, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display offset failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set display clock */
  lcd_res = ssd1315_set_display_clock(&gs_lcd_handle, 0x08, 0x00);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display clock failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set pre charge period */
  lcd_res = ssd1315_set_precharge_period(&gs_lcd_handle, 0x01, 0x0F);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set pre charge period failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set iref */
  lcd_res = ssd1315_set_iref(&gs_lcd_handle, SSD1315_IREF_ENABLE, SSD1315_IREF_VALUE_19UA_150UA);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set iref failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set hardware pins conf */
  lcd_res = ssd1315_set_com_pins_hardware_conf(&gs_lcd_handle, SSD1315_PIN_CONF_ALTERNATIVE, SSD1315_LEFT_RIGHT_REMAP_DISABLE);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set com pins hardware conf failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set deselect level 0.77 */
  lcd_res = ssd1315_set_deselect_level(&gs_lcd_handle, SSD1315_DESELECT_LEVEL_0P77);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set deselect level failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* set page memory addressing mode */
  lcd_res = ssd1315_set_memory_addressing_mode(&gs_lcd_handle, SSD1315_MEMORY_ADDRESSING_MODE_PAGE);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set memory addressing level failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* enable charge pump */
  lcd_res = ssd1315_set_charge_pump(&gs_lcd_handle, SSD1315_CHARGE_PUMP_ENABLE, SSD1315_CHARGE_PUMP_MODE_7P5V);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set charge pump failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* entire display off */
  lcd_res = ssd1315_set_entire_display(&gs_lcd_handle, SSD1315_ENTIRE_DISPLAY_OFF);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set entire display failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* enable display */
  lcd_res = ssd1315_set_display(&gs_lcd_handle, SSD1315_DISPLAY_ON);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: set display failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  /* clear screen */
  lcd_res = ssd1315_clear(&gs_lcd_handle);
  if (lcd_res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: clear failed.\n");
      (void)ssd1315_deinit(&gs_lcd_handle);

      return 1;
  }

  char temp_str[20];
  char hum_str[20];
  uint32_t sleep_duration;

  while (1)
  {
      /* Get sleep duration based on power mode */
      sleep_duration = Power_GetUpdateInterval();
      
      /* Enable peripherals for reading (if disabled in ultra-low mode) */
      Power_EnablePeripheralsForReading();
      
      /* In ultra-low power mode, turn on LCD before update */
      if (Power_ShouldTurnOffLCD())
      {
          ssd1315_set_display(&gs_lcd_handle, SSD1315_DISPLAY_ON);
      }
      
      /* read temperature and humidity */
      res = aht30_read_temperature_humidity(&gs_handle, (uint32_t *)&temperature_raw, (float *)&temperature, (uint32_t *)&humidity_raw, (uint8_t *)&humidity);
      if (res != 0)
      {
          aht30_interface_debug_print("aht30: read failed.\n");
          (void)aht30_deinit(&gs_handle);

          return 1;
      }

      /* clear display */
      ssd1315_clear(&gs_lcd_handle);

      /* format temperature string */
      snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", temperature);
      lcd_res =  ssd1315_gram_write_string(&gs_lcd_handle, 0, 0, temp_str, (uint16_t)strlen(temp_str), 1, SSD1315_FONT_12);
      if (lcd_res != 0)
      {
          ssd1315_interface_debug_print("ssd1315: gram write string failed.\n");
          (void)ssd1315_deinit(&gs_lcd_handle);

          return 1;
      }

      /* format humidity string */
      snprintf(hum_str, sizeof(hum_str), "Hum: %d %%", humidity);
      lcd_res =  ssd1315_gram_write_string(&gs_lcd_handle, 0, 16, hum_str, (uint16_t)strlen(hum_str), 1, SSD1315_FONT_12);
      if (lcd_res != 0)
      {
          ssd1315_interface_debug_print("ssd1315: gram write string failed.\n");
          (void)ssd1315_deinit(&gs_lcd_handle);

          return 1;
      }

      /* update display */
      lcd_res = ssd1315_gram_update(&gs_lcd_handle);
      if (lcd_res != 0)
      {
          ssd1315_interface_debug_print("ssd1315: gram update failed.\n");
          (void)ssd1315_deinit(&gs_lcd_handle);

          return 1;
      }

      /* In ultra-low power mode, turn off LCD after update to save power */
      if (Power_ShouldTurnOffLCD())
      {
          /* Keep display on for 2 seconds so user can read it */
          HAL_Delay(2000);
          ssd1315_set_display(&gs_lcd_handle, SSD1315_DISPLAY_OFF);
          sleep_duration -= 2000; /* Adjust sleep time */
      }
      
      /* Disable peripherals after reading to save power */
      Power_DisablePeripheralsAfterReading();
      
      /* Enter low power sleep mode instead of busy-wait delay */
      if (Power_GetMode() == POWER_MODE_NORMAL)
      {
          /* Normal mode: use regular delay */
          HAL_Delay(sleep_duration);
      }
      else
      {
          /* Low power modes: use Stop mode with RTC wakeup */
          Power_EnterStopMode(sleep_duration);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10B17DB5;
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
  HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hrtc.Init.SynchPrediv = 255;
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
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 17;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Restore system clock after Stop mode
  * @note  Called by power_management.c after waking from Stop mode
  */
void SystemClock_Config_AfterStop(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Re-enable HSI and PLL after Stop mode */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t UART_Transmit(UART_HandleTypeDef *lpuart, uint8_t *pData, uint16_t len)
{
  if(HAL_UART_Transmit_IT(lpuart, pData, len) != HAL_OK)
  {
    if(RingBuffer_Write(&txBuf, pData, len) != RING_BUFFER_OK)
      return 0;
  }
  return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
 /* Set transmission flag: transfer complete*/
 UartReady = SET;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(RingBuffer_GetDataLength(&txBuf) > 0)
  {
    RingBuffer_Read(&txBuf, &txData, 1);
    HAL_UART_Transmit_IT(huart, &txData, 1);
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
