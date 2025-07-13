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
#include "driver_max7219.h"
#include "driver_max7219_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MATRIX_CASCADE_TEST_LENGTH 4;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
max7219_handle_t g_max7219_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t max7219_dot_matrix_init(void);
void max7219_example_main(void);
void max7219_clear_all(void);
void max7219_display_pattern(void);
void max7219_display_hello(void);
void max7219_test_display(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  max7219_example_main();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t max7219_dot_matrix_init(void)
{
    uint8_t res;

    // link functions
    DRIVER_MAX7219_LINK_INIT(&g_max7219_handle, max7219_handle_t);
    DRIVER_MAX7219_LINK_SPI_INIT(&g_max7219_handle, max7219_interface_spi_init);
    DRIVER_MAX7219_LINK_SPI_DEINIT(&g_max7219_handle, max7219_interface_spi_deinit);
    DRIVER_MAX7219_LINK_SPI_WRITE(&g_max7219_handle, max7219_interface_spi_write);
    DRIVER_MAX7219_LINK_SPI_WRITE_COMMAND(&g_max7219_handle, max7219_interface_spi_write_cmd);
    DRIVER_MAX7219_LINK_DELAY_MS(&g_max7219_handle, max7219_interface_delay_ms);
    DRIVER_MAX7219_LINK_DEBUG_PRINT(&g_max7219_handle, max7219_interface_debug_print);

    // max7219 init
    res = max7219_init(&g_max7219_handle);
    if (res != 0)
    {
        max7219_interface_debug_print("MAX7219: init failed\r\n");
        return 1;
    }

    // configure for 4 cascaded displays
    max7219_cascade_t cascade_init[4];

    // set no decode
    for (int i = 0; i < 4; i++)
    {
        cascade_init[i].command = MAX7219_CASCADE_COMMAND_DECODE;
        cascade_init[i].data = MAX7219_DECODE_CODEB_DIGITS_NONE;
    }
    res = max7219_set_cascade(&g_max7219_handle, cascade_init, 4);
    if (res != 0)
    {
    	max7219_interface_debug_print("max7219: set cascade failed.\n");

    	return 1;
    }

    // set intensity
    for (int i = 0; i < 4; i++)
    {
        cascade_init[i].command = MAX7219_CASCADE_COMMAND_INTENSITY;
        cascade_init[i].data = MAX7219_INTENSITY_31_32;
    }
    res = max7219_set_cascade(&g_max7219_handle, cascade_init, 4);
    if (res != 0) return 1;

    // set scan limit digit 0-7
    for (int i = 0; i < 4; i++)
    {
        cascade_init[i].command = MAX7219_CASCADE_COMMAND_SCAN_LIMIT;
        cascade_init[i].data = MAX7219_SCAN_LIMIT_DIGIT_0_7;
    }
    res = max7219_set_cascade(&g_max7219_handle, cascade_init, 4);
    if (res != 0) return 1;

    // turn on normal mode for all displays
    for (int i = 0; i < 4; i++)
    {
        cascade_init[i].command = MAX7219_CASCADE_COMMAND_SHUT_DOWN;
        cascade_init[i].data = MAX7219_MODE_NORMAL;
    }
    res = max7219_set_cascade(&g_max7219_handle, cascade_init, 4);
    if (res != 0) return 1;

    // turn off display test for all displays
    for (int i = 0; i < 4; i++)
    {
        cascade_init[i].command = MAX7219_CASCADE_COMMAND_DISPLAY_TEST;
        cascade_init[i].data = MAX7219_DISPLAY_TEST_MODE_OFF;
    }
    res = max7219_set_cascade(&g_max7219_handle, cascade_init, 4);
    if (res != 0) return 1;

    max7219_interface_debug_print("MAX7219: 4-in-1 dot matrix initialized\r\n");
    return 0;
}

/**
 * @brief Clear all displays
 */
void max7219_clear_all(void)
{
    max7219_cascade_t cascade_data[4];

    // clear all rows for all displays
    for (int row = 1; row <= 8; row++)
    {
        for (int display = 0; display < 4; display++)
        {
            cascade_data[display].command = (max7219_cascade_command_t)row;
            cascade_data[display].data = 0x00;
        }
        max7219_set_cascade(&g_max7219_handle, cascade_data, 4);
    }
}

/**
 * @brief Display a simple pattern on all matrices
 */
void max7219_display_pattern(void)
{
    max7219_cascade_t cascade_data[4];

    // simple patterns for each display
    uint8_t patterns[4][8] = {
        // display 0: Smiley face
        {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C},
        // display 1: Heart
        {0x00, 0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00},
        // display 2: Diamond
        {0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0x7E, 0x3C, 0x18},
        // display 3: Cross
        {0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18}
    };

    // send each row to all displays
    for (int row = 0; row < 8; row++)
    {
        for (int display = 0; display < 4; display++)
        {
            cascade_data[display].command = (max7219_cascade_command_t)(row + 1);
            cascade_data[display].data = patterns[display][row];
        }
        max7219_set_cascade(&g_max7219_handle, cascade_data, 4);
    }
}

/**
 * @brief Display text "ABCD"
 */
void max7219_display_abcd(void)
{
    max7219_cascade_t cascade_data[4];

    // letter patterns (8x8 each)
    uint8_t letter_A[8] = {0x7E, 0x81, 0x81, 0x81, 0xFF, 0x81, 0x81, 0x81};
    uint8_t letter_B[8] = {0xFE, 0x81, 0x81, 0xFE, 0x81, 0x81, 0x81, 0xFE};
    uint8_t letter_C[8] = {0x7E, 0x81, 0x80, 0x80, 0x80, 0x80, 0x81, 0x7E};
    uint8_t letter_D[8] = {0xFE, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFE};

    uint8_t* letters[4] = {letter_A, letter_B, letter_C, letter_D};

    // display each letter on corresponding matrix
    for (int row = 0; row < 8; row++)
    {
        for (int display = 0; display < 4; display++)
        {
            cascade_data[display].command = (max7219_cascade_command_t)(row + 1);
            cascade_data[display].data = letters[display][row];
        }
        max7219_set_cascade(&g_max7219_handle, cascade_data, 4);
    }
}

/**
 * @brief Test all LEDs (display test mode)
 */
void max7219_test_display(void)
{
    max7219_cascade_t cascade_data[4];

    // turn on display test for all displays
    for (int i = 0; i < 4; i++)
    {
        cascade_data[i].command = MAX7219_CASCADE_COMMAND_DISPLAY_TEST;
        cascade_data[i].data = MAX7219_DISPLAY_TEST_MODE_ON;
    }
    max7219_set_cascade(&g_max7219_handle, cascade_data, 4);

    HAL_Delay(2000);

    // turn off display test for all displays
    for (int i = 0; i < 4; i++)
    {
        cascade_data[i].command = MAX7219_CASCADE_COMMAND_DISPLAY_TEST;
        cascade_data[i].data = MAX7219_DISPLAY_TEST_MODE_OFF;
    }
    max7219_set_cascade(&g_max7219_handle, cascade_data, 4);
}

void max7219_example_main(void)
{
    // init
    if (max7219_dot_matrix_init() != 0)
    {
        max7219_interface_debug_print("Failed to initialize MAX7219\r\n");
        return;
    }

    while (1)
    {
        // test all LEDs
        max7219_interface_debug_print("Testing all LEDs...\r\n");
        max7219_test_display();
        HAL_Delay(1000);

        // clear display
        max7219_interface_debug_print("Clearing display...\r\n");
        max7219_clear_all();
        HAL_Delay(1000);

        // show patterns
        max7219_interface_debug_print("Showing patterns...\r\n");
        max7219_display_pattern();
        HAL_Delay(3000);

        // clear and show ABCD
        max7219_clear_all();
        HAL_Delay(500);
        max7219_interface_debug_print("Showing ABCD...\r\n");
        max7219_display_abcd();
        HAL_Delay(3000);

        // clear for next iteration
        max7219_clear_all();
        HAL_Delay(1000);
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
