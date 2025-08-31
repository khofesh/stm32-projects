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
#include "wii_nunchuk.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define READ_INTERVAL_MS    200  // read nunchuk every 50ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char readBuf[1];
uint8_t txData;
__IO ITStatus UartReady = SET;
RingBuffer txBuf, rxBuf;
wii_nunchuk_handle_t nunchuk_handle;
uint32_t last_read_time = 0;
char uart_buffer[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void print_nunchuk_data(wii_nunchuk_data_t *data);
void debug_print_raw_data(uint8_t *data, uint8_t size);
void debug_print_i2c_status(I2C_HandleTypeDef *hi2c);
void test_i2c_connection(void);
void debug_nunchuk_validation(uint8_t *raw_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
wii_nunchuk_result_t wii_nunchuk_auto_init(wii_nunchuk_handle_t *handle, I2C_HandleTypeDef *hi2c)
{
    wii_nunchuk_result_t result;

    UART_Transmit(&huart1, (uint8_t*)"Trying WHITE nunchuk init...\r\n", 31);

    // Try white first
    result = wii_nunchuk_init_working(handle, hi2c, WII_NUNCHUK_WHITE);
    if (result == WII_NUNCHUK_OK) {
        // Test with a read
        HAL_Delay(100);
        result = wii_nunchuk_read_sync_fixed(handle);
        if (result == WII_NUNCHUK_OK) {
            UART_Transmit(&huart1, (uint8_t*)"WHITE nunchuk init SUCCESS! ✓\r\n", 32);
            return WII_NUNCHUK_OK;
        }
    }

    UART_Transmit(&huart1, (uint8_t*)"WHITE failed, trying BLACK nunchuk init...\r\n", 45);

    // Try black
    result = wii_nunchuk_init_working(handle, hi2c, WII_NUNCHUK_BLACK);
    if (result == WII_NUNCHUK_OK) {
        // Test with a read
        HAL_Delay(100);
        result = wii_nunchuk_read_sync_fixed(handle);
        if (result == WII_NUNCHUK_OK) {
            UART_Transmit(&huart1, (uint8_t*)"BLACK nunchuk init SUCCESS! ✓\r\n", 32);
            return WII_NUNCHUK_OK;
        }
    }

    UART_Transmit(&huart1, (uint8_t*)"Both initialization methods FAILED ✗\r\n", 39);
    return WII_NUNCHUK_ERROR_I2C_ERROR;
}

void print_nunchuk_data_debug(wii_nunchuk_data_t *data)
{
    static uint32_t counter = 0;
    counter++;

    if (data->data_valid) {
        char buffer[120];  // Smaller buffer
        int len = snprintf(buffer, sizeof(buffer),
            "[%lu] Joy:%3d,%3d Acc:%4d,%4d,%4d Btn:C%d,Z%d\r\n",
            counter,
            data->joystick_x, data->joystick_y,
            data->accel_x, data->accel_y, data->accel_z,
            data->button_c, data->button_z);

        UART_Transmit(&huart1, (uint8_t*)buffer, len);
    } else {
        char buffer[50];
        int len = snprintf(buffer, sizeof(buffer), "[%lu] INVALID\r\n", counter);
        UART_Transmit(&huart1, (uint8_t*)buffer, len);
    }
}

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize Wii Nunchuk driver
  UART_Transmit(&huart1, (uint8_t*)"Device detected at 0xA4! Initializing...\r\n", 44);

  wii_nunchuk_result_t result = wii_nunchuk_auto_init(&nunchuk_handle, &hi2c1);

  if (result == WII_NUNCHUK_OK) {
      UART_Transmit(&huart1, (uint8_t*)"Nunchuk ready for operation!\r\n", 31);
  } else {
      UART_Transmit(&huart1, (uint8_t*)"Nunchuk initialization failed!\r\n", 33);
  }

  last_read_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (HAL_GetTick() - last_read_time >= READ_INTERVAL_MS) {
	        if (wii_nunchuk_read_sync_fixed(&nunchuk_handle) == WII_NUNCHUK_OK) {
	            wii_nunchuk_data_t nunchuk_data;
	            if (wii_nunchuk_get_data(&nunchuk_handle, &nunchuk_data) == WII_NUNCHUK_OK) {
	                print_nunchuk_data_debug(&nunchuk_data);
	            }
	        } else {
	            static uint32_t error_count = 0;
	            error_count++;
	            if (error_count % 50 == 0) { // Print error every 2.5 seconds
	                UART_Transmit(&huart1, (uint8_t*)"Read error\r\n", 12);
	            }
	        }

	        last_read_time = HAL_GetTick();
	    }

	    HAL_Delay(1); // Small delay to prevent overwhelming the loop
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
  if(HAL_UART_Transmit_IT(huart, pData, len) != HAL_OK)
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

void print_nunchuk_data(wii_nunchuk_data_t *data)
{
    if (data->data_valid)
    {
        int len = snprintf(uart_buffer, sizeof(uart_buffer),
            "Joystick: X=%3d Y=%3d | Accel: X=%4d Y=%4d Z=%4d | Buttons: C=%d Z=%d\r\n",
            data->joystick_x, data->joystick_y,
            data->accel_x, data->accel_y, data->accel_z,
            data->button_c, data->button_z);

        UART_Transmit(&huart1, (uint8_t*)uart_buffer, len);
    }
    else
    {
    	UART_Transmit(&huart1, (uint8_t*)"Invalid data received\r\n", 23);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        wii_nunchuk_i2c_tx_cplt_callback(&nunchuk_handle);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        wii_nunchuk_i2c_rx_cplt_callback(&nunchuk_handle);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        wii_nunchuk_i2c_error_callback(&nunchuk_handle);

        // Debug: Print I2C error
        char error_msg[50];
        int len = snprintf(error_msg, sizeof(error_msg), "I2C Error: 0x%08lX\r\n", hi2c->ErrorCode);
        UART_Transmit(&huart1, (uint8_t*)error_msg, len);
    }
}

void debug_print_raw_data(uint8_t *data, uint8_t size)
{
    char debug_buffer[150];
    int len = snprintf(debug_buffer, sizeof(debug_buffer), "Raw data: ");

    for(int i = 0; i < size; i++) {
        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len, "0x%02X ", data[i]);
    }
    len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len, "\r\n");

    UART_Transmit(&huart1, (uint8_t*)debug_buffer, len);
}

void debug_print_i2c_status(I2C_HandleTypeDef *hi2c)
{
    char debug_buffer[100];
    int len = snprintf(debug_buffer, sizeof(debug_buffer),
        "I2C State: %d, Error: 0x%08lX\r\n",
        (int)hi2c->State, hi2c->ErrorCode);
    UART_Transmit(&huart1, (uint8_t*)debug_buffer, len);
}

void test_i2c_connection(void)
{
    UART_Transmit(&huart1, (uint8_t*)"Testing I2C connection to Nunchuk...\r\n", 39);

    // Test if device responds at address 0x52
    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, WII_NUNCHUK_I2C_ADDR, 3, 1000);

    char buffer[50];
    int len;
    if(result == HAL_OK) {
        len = snprintf(buffer, sizeof(buffer), "Nunchuk found at address 0x52!\r\n");
    } else {
        len = snprintf(buffer, sizeof(buffer), "Nunchuk NOT found (error: %d)\r\n", result);
    }
    UART_Transmit(&huart1, (uint8_t*)buffer, len);
}

void debug_nunchuk_validation(uint8_t *raw_data)
{
    UART_Transmit(&huart1, (uint8_t*)"=== Debug Nunchuk Data ===\r\n", 29);

    // Print raw data
    debug_print_raw_data(raw_data, 6);

    // Check if all bytes are 0xFF (common error)
    bool all_ff = true;
    bool all_zero = true;

    for(int i = 0; i < 6; i++) {
        if(raw_data[i] != 0xFF) all_ff = false;
        if(raw_data[i] != 0x00) all_zero = false;
    }

    if(all_ff) {
        UART_Transmit(&huart1, (uint8_t*)"ERROR: All bytes are 0xFF!\r\n", 29);
    }
    if(all_zero) {
        UART_Transmit(&huart1, (uint8_t*)"ERROR: All bytes are 0x00!\r\n", 29);
    }

    // Decode and print decoded data
    uint8_t decoded_data[6];
    for (int i = 0; i < 6; i++) {
        decoded_data[i] = (raw_data[i] ^ 0x17) + 0x17;
    }

    UART_Transmit(&huart1, (uint8_t*)"Decoded data: ", 14);
    debug_print_raw_data(decoded_data, 6);

    // Print interpreted values
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer),
        "Joystick X: %d, Y: %d\r\nAccel X: %d, Y: %d, Z: %d\r\nButtons: C=%d, Z=%d\r\n",
        decoded_data[0], decoded_data[1],
        (decoded_data[2] << 2) | ((decoded_data[5] >> 2) & 0x03),
        (decoded_data[3] << 2) | ((decoded_data[5] >> 4) & 0x03),
        (decoded_data[4] << 2) | ((decoded_data[5] >> 6) & 0x03),
        !((decoded_data[5] >> 1) & 0x01),
        !(decoded_data[5] & 0x01));

    UART_Transmit(&huart1, (uint8_t*)buffer, len);
    UART_Transmit(&huart1, (uint8_t*)"=========================\r\n", 27);
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
