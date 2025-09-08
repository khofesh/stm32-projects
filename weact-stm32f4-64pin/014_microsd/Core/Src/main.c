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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_functions.h"
#include "stdio.h"
#include "sd_benchmark.h"
#include <string.h>
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
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char readBuf[1];
uint8_t txData;
__IO ITStatus UartReady = SET;
RingBuffer txBuf, rxBuf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// printf redirection to custom UART function
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    UART_Transmit(&huart1, &c, 1);
    return ch;
}

void uart_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(buffer)) {
        UART_Transmit(&huart1, (uint8_t*)buffer, len);
    }
}

void sd_hardware_test(void) {
	printf("=== SD Hardware Test ===\r\n");

	// Test 1: Check if SDIO peripheral is enabled
	if (__HAL_RCC_SDIO_IS_CLK_ENABLED()) {
		printf("✅ SDIO clock enabled\r\n");
	} else {
		printf("❌ SDIO clock NOT enabled\r\n");
	}

	// Test 2: Check GPIO configuration
	printf("Checking GPIO pins:\r\n");
	printf("PC8 (D0) mode: 0x%lx\r\n", (GPIOC->MODER >> 16) & 0x3);
	printf("PC12 (CLK) mode: 0x%lx\r\n", (GPIOC->MODER >> 24) & 0x3);
	printf("PD2 (CMD) mode: 0x%lx\r\n", (GPIOD->MODER >> 4) & 0x3);

	// Check alternate function settings
	printf("PC8 AFR: 0x%lx\r\n", (GPIOC->AFR[1] >> 0) & 0xF);
	printf("PC12 AFR: 0x%lx\r\n", (GPIOC->AFR[1] >> 16) & 0xF);
	printf("PD2 AFR: 0x%lx\r\n", (GPIOD->AFR[0] >> 8) & 0xF);

	// Test 3: Check SDIO peripheral directly instead of BSP
	printf("Testing SDIO peripheral directly...\r\n");

	// Check if SDIO is responding
	if (hsd.Instance == SDIO) {
		printf("✅ SDIO instance configured\r\n");
		printf("SDIO POWER: 0x%lx\r\n", SDIO->POWER);
		printf("SDIO CLKCR: 0x%lx\r\n", SDIO->CLKCR);
	} else {
		printf("❌ SDIO instance not configured\r\n");
		return;
	}

	// Test 4: Try HAL SD card detection
	printf("Checking SD card state...\r\n");
	HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(&hsd);
	printf("Card State: %ld ", cardState);

	switch(cardState) {
		case HAL_SD_CARD_READY:
			printf("(Ready)\r\n");
			break;
		case HAL_SD_CARD_IDENTIFICATION:
			printf("(Identification)\r\n");
			break;
		case HAL_SD_CARD_STANDBY:
			printf("(Standby)\r\n");
			break;
		case HAL_SD_CARD_TRANSFER:
			printf("(Transfer)\r\n");
			break;
		case HAL_SD_CARD_SENDING:
			printf("(Sending)\r\n");
			break;
		case HAL_SD_CARD_RECEIVING:
			printf("(Receiving)\r\n");
			break;
		case HAL_SD_CARD_PROGRAMMING:
			printf("(Programming)\r\n");
			break;
		case HAL_SD_CARD_DISCONNECTED:
			printf("(Disconnected)\r\n");
			break;
		case HAL_SD_CARD_ERROR:
			printf("(Error)\r\n");
			break;
	}

	// Test 5: Get card info using HAL
	HAL_SD_CardInfoTypeDef cardInfo;
	if (HAL_SD_GetCardInfo(&hsd, &cardInfo) == HAL_OK) {
		printf("✅ Card info retrieved successfully\r\n");
		printf("Card Type: %lu\r\n", cardInfo.CardType);
		printf("Card Version: %lu\r\n", cardInfo.CardVersion);
		printf("Block Size: %lu\r\n", cardInfo.BlockSize);
		printf("Block Count: %lu\r\n", cardInfo.BlockNbr);
		printf("Capacity: %lu MB\r\n", (cardInfo.BlockNbr * cardInfo.BlockSize) / (1024*1024));
	} else {
		printf("❌ Failed to get card info\r\n");
		printf("Possible issues:\r\n");
		printf("- SD card not inserted or defective\r\n");
		printf("- Wiring issues\r\n");
		printf("- Clock configuration problems\r\n");
		printf("- Power supply issues\r\n");
	}

	// Test 6: Try to read a sector using HAL
	printf("Attempting to read sector 0 using HAL...\r\n");
	uint8_t testBuffer[512];

	if (HAL_SD_ReadBlocks(&hsd, testBuffer, 0, 1, 5000) == HAL_OK) {
		printf("✅ Successfully read sector 0\r\n");
		printf("First 16 bytes: ");
		for (int i = 0; i < 16; i++) {
			printf("%02X ", testBuffer[i]);
		}
		printf("\r\n");

		// Check for MBR signature
		if (testBuffer[510] == 0x55 && testBuffer[511] == 0xAA) {
			printf("✅ Valid MBR signature found\r\n");
		} else {
			printf("⚠️  No MBR signature (might be superfloppy format)\r\n");
		}
	} else {
		printf("❌ Failed to read sector 0\r\n");

		// Get the last error
		uint32_t errorState = HAL_SD_GetError(&hsd);
		printf("SD Error: 0x%lX\r\n", errorState);

		if (errorState & HAL_SD_ERROR_CMD_RSP_TIMEOUT) printf("- Command response timeout\r\n");
		if (errorState & HAL_SD_ERROR_DATA_TIMEOUT) printf("- Data timeout\r\n");
		if (errorState & HAL_SD_ERROR_CMD_CRC_FAIL) printf("- Command CRC failed\r\n");
		if (errorState & HAL_SD_ERROR_DATA_CRC_FAIL) printf("- Data CRC failed\r\n");
		if (errorState & HAL_SD_ERROR_UNSUPPORTED_FEATURE) printf("- Unsupported feature\r\n");
	}

	printf("=== End Hardware Test ===\r\n\r\n");
}


uint8_t bufr[100];
UINT br;
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
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  RingBuffer_Init(&txBuf);
  RingBuffer_Init(&rxBuf);

  HAL_Delay(1000);

  printf("Initializing SD peripheral...\r\n");
  if (HAL_SD_Init(&hsd) != HAL_OK) {
      printf("❌ HAL_SD_Init failed!\r\n");
      Error_Handler();
  } else {
      printf("✅ HAL_SD_Init successful\r\n");
  }

  // Now test hardware after proper initialization
  sd_hardware_test();

  HAL_Delay(5000);

  // Now try mounting with enhanced debugging
  if (sd_mount() == FR_OK) {
	  sd_get_space_kb();
	  HAL_Delay(10);
	  sd_list_files_limited();
	  sd_unmount();
  } else {
	  printf("Mount failed - skipping file operations\r\n");
  }

  HAL_Delay(1000);
  sd_mount();
  sd_read_file("WeAct Studio.txt", (char*)bufr, 100, &br);
  printf("DATA: %s\r\n", bufr);
  sd_unmount();

//    sd_mount();
//    sd_write_file("FILE8.TXT", "This is File 8 and it is in the root of the SD Card\n");
//    sd_read_file("FILE8.txt", (char*)bufr, 100, &br);
//    printf("DATA: %s\n\n", bufr);
//    sd_unmount();

//    sd_mount();
//    sd_append_file("FILE8.TXT", "THis is appended text to file8\n");
//    sd_read_file("FILE8.txt", (char*) bufr, 100, &br);
//    printf("DATA: %s\n\n", bufr);
//    sd_unmount();

  #define max_records 20
    CsvRecord myrecords[max_records];
    int record_count = 0;
    sd_mount();
    sd_read_csv("file4.csv", myrecords, max_records, &record_count);
    sd_unmount();


//    sd_benchmark();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */
//  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  /* USER CODE END SDIO_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
    if(huart->gState == HAL_UART_STATE_READY) {
        if(HAL_UART_Transmit_IT(huart, pData, len) == HAL_OK) {
            return 0;
        }
    }

    return (RingBuffer_Write(&txBuf, pData, len) == RING_BUFFER_OK) ? 0 : 1;
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
