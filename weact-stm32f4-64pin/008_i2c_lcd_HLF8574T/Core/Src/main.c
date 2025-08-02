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
#include "lcd_i2c.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LCD_I2C_ADDRESS 0x27
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Test_Basic(void);
void LCD_Test_Counter(void);
void LCD_Test_CustomChars(void);
void I2C_Scanner(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  char uart_msg[] = "STM32F446RE LCD I2C Test Starting...\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

  // scan for i2c devices first
  I2C_Scanner();

  // init LCD - try common addresses if 0x27 doesn't work
  if (lcdInit(&hi2c1, LCD_I2C_ADDRESS, 2, 16)) {
    char success_msg[] = "LCD Initialized Successfully!\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)success_msg, strlen(success_msg), HAL_MAX_DELAY);

    // LCD tests
    LCD_Test_Basic();
    HAL_Delay(3000);

    LCD_Test_CustomChars();
    HAL_Delay(3000);

  } else {
    char error_msg[] = "LCD Initialization Failed! Check connections and address.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LCD_Test_Counter();
	  HAL_Delay(1000);
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
/**
 * @brief Basic LCD test - display text
 */
void LCD_Test_Basic(void) {
    lcdDisplayClear();

    // line 1
    lcdSetCursorPosition(0, 0);
    char line1[] = "STM32F446RE LCD";
    lcdPrintStr((uint8_t*)line1, strlen(line1));

    // line 2
    lcdSetCursorPosition(0, 1);
    char line2[] = "I2C Test Ready!";
    lcdPrintStr((uint8_t*)line2, strlen(line2));

    // test backlight control
    HAL_Delay(1000);
    lcdBacklightOff();
    HAL_Delay(500);
    lcdBacklightOn();
}

/**
 * @brief Counter test - displays incrementing counter
 */
void LCD_Test_Counter(void) {
    char buffer[17];

    lcdSetCursorPosition(0, 1);
    snprintf(buffer, sizeof(buffer), "Counter: %3d    ", counter);
    lcdPrintStr((uint8_t*)buffer, strlen(buffer));

    counter++;
    if (counter > 999) counter = 0;
}

/**
 * @brief Test custom characters
 */
void LCD_Test_CustomChars(void) {
    // define custom characters
    uint8_t heart[8] = {0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00}; // heart
    uint8_t smiley[8] = {0x00, 0x0A, 0x00, 0x04, 0x11, 0x0E, 0x00, 0x00}; // smiley
    uint8_t arrow[8] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00}; // arrow

    // load custom characters into CGRAM
    lcdLoadCustomChar(0, heart);
    lcdLoadCustomChar(1, smiley);
    lcdLoadCustomChar(2, arrow);

    lcdDisplayClear();

    // display custom characters
    lcdSetCursorPosition(0, 0);
    char line1[] = "Custom Chars:";
    lcdPrintStr((uint8_t*)line1, strlen(line1));

    lcdSetCursorPosition(0, 1);
    lcdPrintChar(0);  // heart
    lcdPrintChar(' ');
    lcdPrintChar(1);  // smiley
    lcdPrintChar(' ');
    lcdPrintChar(2);  // arrow
    lcdPrintChar(' ');
    char text[] = "Works!";
    lcdPrintStr((uint8_t*)text, strlen(text));
}

/**
 * @brief Scan I2C bus for devices
 */
void I2C_Scanner(void) {
    char uart_buffer[64];
    uint8_t devices_found = 0;

    snprintf(uart_buffer, sizeof(uart_buffer), "Scanning I2C bus...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    for (uint8_t address = 1; address < 128; address++) {
        // try to communicate with device
        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, 100) == HAL_OK) {
            snprintf(uart_buffer, sizeof(uart_buffer), "Device found at address: 0x%02X\r\n", address);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        snprintf(uart_buffer, sizeof(uart_buffer), "No I2C devices found! Check connections.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    } else {
        snprintf(uart_buffer, sizeof(uart_buffer), "Scan complete. Found %d device(s).\r\n", devices_found);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
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
