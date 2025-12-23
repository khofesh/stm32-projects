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
#include "driver_ds3231_interface.h"
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

COM_InitTypeDef BspCOMInit;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static ds3231_handle_t gs_handle; // ds3231 handle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
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
    uint8_t res;
    int8_t reg;
    uint32_t i;
    int16_t raw;
    float s;
    ds3231_info_t info;
    ds3231_time_t time_in, time_out;
    int times = 10;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

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
  DRIVER_DS3231_LINK_INIT(&gs_handle, ds3231_handle_t);
  DRIVER_DS3231_LINK_IIC_INIT(&gs_handle, ds3231_interface_iic_init);
  DRIVER_DS3231_LINK_IIC_DEINIT(&gs_handle, ds3231_interface_iic_deinit);
  DRIVER_DS3231_LINK_IIC_READ(&gs_handle, ds3231_interface_iic_read);
  DRIVER_DS3231_LINK_IIC_WRITE(&gs_handle, ds3231_interface_iic_write);
  DRIVER_DS3231_LINK_DELAY_MS(&gs_handle, ds3231_interface_delay_ms);
  DRIVER_DS3231_LINK_DEBUG_PRINT(&gs_handle, ds3231_interface_debug_print);
  DRIVER_DS3231_LINK_RECEIVE_CALLBACK(&gs_handle, ds3231_interface_receive_callback);

  /* get ds3231 info */
  res = ds3231_info(&info);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: get info failed.\n");

      return 1;
  }
  else
  {
      /* print ds3231 info */
      ds3231_interface_debug_print("ds3231: chip is %s.\n", info.chip_name);
      ds3231_interface_debug_print("ds3231: manufacturer is %s.\n", info.manufacturer_name);
      ds3231_interface_debug_print("ds3231: interface is %s.\n", info.interface);
      ds3231_interface_debug_print("ds3231: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
      ds3231_interface_debug_print("ds3231: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
      ds3231_interface_debug_print("ds3231: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
      ds3231_interface_debug_print("ds3231: max current is %0.2fmA.\n", info.max_current_ma);
      ds3231_interface_debug_print("ds3231: max temperature is %0.1fC.\n", info.temperature_max);
      ds3231_interface_debug_print("ds3231: min temperature is %0.1fC.\n", info.temperature_min);
  }

  /* start readwrite test */
  ds3231_interface_debug_print("ds3231: start readwrite test.\n");

  /* init ds3231 */
  res = ds3231_init(&gs_handle);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: init failed.\n");

      return 1;
  }

  /* set oscillator */
  res = ds3231_set_oscillator(&gs_handle, DS3231_BOOL_TRUE);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set oscillator failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* disable alarm1 */
  res = ds3231_set_alarm_interrupt(&gs_handle, DS3231_ALARM_1, DS3231_BOOL_FALSE);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set alarm1 interrupt failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* disable alarm2 */
  res = ds3231_set_alarm_interrupt(&gs_handle, DS3231_ALARM_2, DS3231_BOOL_FALSE);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set alarm2 interrupt failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* disable 32khz output */
  res = ds3231_set_32khz_output(&gs_handle, DS3231_BOOL_FALSE);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set 32khz output failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* convert to register */
  res = ds3231_aging_offset_convert_to_register(&gs_handle, 0, (int8_t *)&reg);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: convert to register failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* set aging offset */
  res = ds3231_set_aging_offset(&gs_handle, reg);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set aging offset failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }

  /* ds3231_set_time/ds3231_get_time test */
  ds3231_interface_debug_print("ds3231: ds3231_set_time/ds3231_get_time test.\n");


  /* 12H format */
  time_in.format = DS3231_FORMAT_12H;
  time_in.am_pm = DS3231_PM;
  time_in.year = rand() % 100 + 2000;
  time_in.month = rand() % 12 + 1;
  time_in.date = rand() % 20 + 1;
  time_in.date = rand() % 20 + 1;
  time_in.week = rand() % 7 + 1;
  time_in.hour = rand() % 11 + 1;
  time_in.minute = rand() % 60;
  time_in.second = rand() % 60;
  ds3231_interface_debug_print("ds3231: set time %04d-%02d-%02d PM %02d:%02d:%02d %d in 12 format.\n",
                               time_in.year, time_in.month, time_in.date,
                               time_in.hour, time_in.minute, time_in.second, time_in.week
                              );
  res  = ds3231_set_time(&gs_handle, &time_in);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set time failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }
  for (i = 0; i < times; i++)
  {
      ds3231_interface_delay_ms(1000);
      res = ds3231_get_time(&gs_handle, &time_out);
      if (res != 0)
      {
          ds3231_interface_debug_print("ds3231: get time failed.\n");
          (void)ds3231_deinit(&gs_handle);

          return 1;
      }
      ds3231_interface_debug_print("ds3231: time is %04d-%02d-%02d PM %02d:%02d:%02d %d.\n",
                                   time_out.year, time_out.month, time_out.date,
                                   time_out.hour, time_out.minute, time_out.second, time_out.week
                                  );
  }

  /* 24H format */
  time_in.format = DS3231_FORMAT_24H;
  time_in.am_pm = DS3231_AM;
  time_in.year = rand() % 100 + 2090;
  time_in.month = rand() % 12 + 1;
  time_in.date = rand() % 20 + 1;
  time_in.week = rand() % 7 + 1;
  time_in.hour = rand() % 12 + 12;
  time_in.minute = rand() % 60;
  time_in.second = rand() % 60;
  ds3231_interface_debug_print("ds3231: set time %04d-%02d-%02d %02d:%02d:%02d %d in 24 format.\n",
                               time_in.year, time_in.month, time_in.date,
                               time_in.hour, time_in.minute, time_in.second, time_in.week
                              );
  res  = ds3231_set_time(&gs_handle, &time_in);
  if (res != 0)
  {
      ds3231_interface_debug_print("ds3231: set time failed.\n");
      (void)ds3231_deinit(&gs_handle);

      return 1;
  }
  for (i = 0; i < times; i++)
  {
      ds3231_interface_delay_ms(1000);
      res = ds3231_get_time(&gs_handle, &time_out);
      if (res != 0)
      {
          ds3231_interface_debug_print("ds3231: get time failed.\n");
          (void)ds3231_deinit(&gs_handle);

          return 1;
      }
      ds3231_interface_debug_print("ds3231: time is %04d-%02d-%02d %02d:%02d:%02d %d.\n",
                                   time_out.year, time_out.month, time_out.date,
                                   time_out.hour, time_out.minute, time_out.second, time_out.week
                                  );
  }
  ds3231_interface_debug_print("ds3231: read temperature.\n");
  for (i = 0; i < times; i++)
  {
      ds3231_interface_delay_ms(1000);
      res = ds3231_get_temperature(&gs_handle, (int16_t *)&raw, (float *)&s);
      if (res != 0)
      {
          ds3231_interface_debug_print("ds3231: get temperature failed.\n");
          (void)ds3231_deinit(&gs_handle);

          return 1;
      }
      ds3231_interface_debug_print("ds3231: temperature is %0.2f.\n", s);
  }

  /* finish readwrite test */
  ds3231_interface_debug_print("ds3231: finish readwrite test.\n");
  (void)ds3231_deinit(&gs_handle);

  while (1)
  {

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.Timing = 0x60514452;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

extern volatile I2C_State_t i2c_state;
extern volatile HAL_StatusTypeDef i2c_result;

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_state = I2C_STATE_READY;
        i2c_result = HAL_OK;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_state = I2C_STATE_READY;
        i2c_result = HAL_OK;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2c_state = I2C_STATE_ERROR;
        i2c_result = HAL_ERROR;
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
