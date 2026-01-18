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
#include "arducam.h"
#include <stdio.h>
#include <string.h>
#include "stm32l5xx_nucleo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Frame markers for host communication */
#define FRAME_START_MARKER_0    0xFF
#define FRAME_START_MARKER_1    0xAA
#define FRAME_END_MARKER_0      0xBB
#define FRAME_END_MARKER_1      0xCC

/* Commands from host */
#define CMD_SET_RES_320x240     0x00
#define CMD_SET_RES_640x480     0x01
#define CMD_SET_RES_1024x768    0x02
#define CMD_SET_RES_1280x960    0x03
#define CMD_SET_RES_1600x1200   0x04
#define CMD_SET_RES_2048x1536   0x05
#define CMD_SET_RES_2592x1944   0x06
#define CMD_SINGLE_CAPTURE      0x10
#define CMD_INIT_JPEG           0x11
#define CMD_START_STREAMING     0x20
#define CMD_STOP_STREAMING      0x21
#define CMD_QUALITY_HIGH        0xD0
#define CMD_QUALITY_DEFAULT     0xD1
#define CMD_QUALITY_LOW         0xD2

/* Capture modes */
#define MODE_IDLE               0
#define MODE_SINGLE             1
#define MODE_STREAMING          2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static arducam_handle_t cam;

#define IMAGE_BUFFER_SIZE   (64 * 1024)  /* 64KB for JPEG */
static uint8_t image_buffer[IMAGE_BUFFER_SIZE];

/* Streaming state */
static volatile uint8_t capture_mode = MODE_IDLE;
static volatile uint8_t start_capture = 0;
static volatile uint8_t rx_command = 0xFF;
static volatile uint8_t rx_data_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef camera_init(void);
static void process_command(uint8_t cmd);
static HAL_StatusTypeDef capture_and_send_image(void);
static void send_image_data(uint8_t *data, uint32_t length);
static void uart_send_string(const char *str);
static void start_uart_receive(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Print single character to USART2 */
void print_uart2(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
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
  /* Enable USART2 interrupt for command reception */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if (camera_init() != HAL_OK) {
      printf("Camera initialization failed!\r\n");
      BSP_LED_On(LED_RED);
  } else {
      printf("Camera ready. Waiting for commands...\r\n");
      BSP_LED_On(LED_GREEN);
  }

  /* Start receiving commands from host */
  start_uart_receive();

  while (1)
  {
      /* Check for received command */
      if (rx_data_ready) {
          rx_data_ready = 0;
          process_command(rx_command);
          start_uart_receive();  /* Restart reception */
      }

      /* Handle capture modes */
      if (capture_mode == MODE_SINGLE) {
          if (start_capture == 1) {
              /* Start single capture */
              printf("Starting single capture...\r\n");
              arducam_exit_standby(&cam);
              HAL_Delay(300);  /* Wait for sensor to wake up */

              /* Flush and clear FIFO */
              arducam_flush_fifo(&cam);
              arducam_clear_fifo_flag(&cam);

              /* Start capture */
              arducam_start_capture(&cam);
              printf("Capture started, waiting for completion...\r\n");
              start_capture = 0;
          }

          /* Check if capture is done */
          bool done = false;
          arducam_is_capture_done(&cam, &done);
          if (done) {
              /* Wait a bit more to ensure FIFO is filled */
              HAL_Delay(50);

              printf("Capture done!\r\n");
              BSP_LED_Toggle(LED_BLUE);
              capture_and_send_image();
              arducam_clear_fifo_flag(&cam);
              arducam_enter_standby(&cam);
              capture_mode = MODE_IDLE;
          }
      }
      else if (capture_mode == MODE_STREAMING) {
          /* Check for stop command */
          if (rx_data_ready && rx_command == CMD_STOP_STREAMING) {
              rx_data_ready = 0;
              capture_mode = MODE_IDLE;
              start_capture = 0;
              uart_send_string("ACK CMD CAM stop video streaming. END");
              arducam_enter_standby(&cam);
              start_uart_receive();
              continue;
          }

          if (start_capture == 2) {
              /* Start streaming capture */
              arducam_flush_fifo(&cam);
              arducam_clear_fifo_flag(&cam);
              arducam_start_capture(&cam);
              start_capture = 0;
          }

          /* Check if capture is done */
          bool done = false;
          arducam_is_capture_done(&cam, &done);
          if (done) {
              BSP_LED_Toggle(LED_BLUE);
              capture_and_send_image();
              start_capture = 2;  /* Continue streaming */
          }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static HAL_StatusTypeDef camera_init(void)
{
    HAL_StatusTypeDef status;
    uint16_t chip_id;
    char msg[64];
    uint8_t test_read;

    /* Configure ArduCAM */
    arducam_config_t config = {
        .hspi = &hspi1,
        .hi2c = &hi2c1,
        .cs_port = GPIOD,
        .cs_pin = GPIO_PIN_14,
        .sensor_model = ARDUCAM_OV5642
    };

    /* Initialize driver */
    status = arducam_init(&cam, &config);
    if (status != HAL_OK) {
        printf("ArduCAM init failed!\r\n");
        return status;
    }
    printf("ArduCAM driver initialized\r\n");

    /* Debug: Test raw SPI read/write */
    printf("Testing SPI communication...\r\n");

    /* Write 0x55 to test register */
    status = arducam_write_reg(&cam, 0x00, 0x55);
    printf("  Write 0x55 to reg 0x00: %s\r\n", (status == HAL_OK) ? "OK" : "FAIL");

    /* Read back */
    status = arducam_read_reg(&cam, 0x00, &test_read);
    printf("  Read reg 0x00: 0x%02X (expected 0x55), status: %s\r\n",
           test_read, (status == HAL_OK) ? "OK" : "FAIL");

    /* Write 0xAA to test register */
    status = arducam_write_reg(&cam, 0x00, 0xAA);
    printf("  Write 0xAA to reg 0x00: %s\r\n", (status == HAL_OK) ? "OK" : "FAIL");

    /* Read back */
    status = arducam_read_reg(&cam, 0x00, &test_read);
    printf("  Read reg 0x00: 0x%02X (expected 0xAA), status: %s\r\n",
           test_read, (status == HAL_OK) ? "OK" : "FAIL");

    /* Verify SPI communication */
    status = arducam_verify_spi(&cam);
    if (status != HAL_OK) {
        printf("SPI verification failed! Check wiring.\r\n");
        printf("  Possible causes:\r\n");
        printf("  - CS pin not connected (PD14)\r\n");
        printf("  - SPI wiring: SCK=PA5, MISO=PA6, MOSI=PA7\r\n");
        printf("  - ArduCAM not powered\r\n");
        printf("  - SPI mode mismatch (Mode 0: CPOL=0, CPHA=0)\r\n");
        return status;
    }
    printf("SPI communication OK\r\n");

    /* I2C scan to find devices */
    printf("Scanning I2C bus...\r\n");
    uint8_t found_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
            printf("  Found device at 0x%02X\r\n", addr);
            found_addr = addr;
        }
    }
    if (found_addr == 0) {
        printf("  No I2C devices found!\r\n");
        printf("  Check wiring: SDA=PB9, SCL=PB8\r\n");
        printf("  Check power to ArduCAM module\r\n");
    }

    /* Verify I2C communication and read chip ID */
    printf("Trying OV5642 at address 0x%02X...\r\n", cam.sensor_addr);
    status = arducam_verify_i2c(&cam, &chip_id);
    if (status != HAL_OK) {
        printf("I2C verification failed!\r\n");
        printf("  Expected OV5642 at 0x3C (7-bit)\r\n");
        printf("  Check wiring: SDA=PB9, SCL=PB8\r\n");
        printf("  Ensure ArduCAM is powered (5V and 3.3V)\r\n");

        /* Try alternate address */
        printf("Trying alternate address 0x78 (8-bit = 0x3C << 1)...\r\n");
        cam.sensor_addr = 0x78 >> 1;  /* Try 0x3C */
        status = arducam_verify_i2c(&cam, &chip_id);
        if (status != HAL_OK) {
            printf("Still failed. Chip ID read: 0x%04X\r\n", chip_id);
            return status;
        }
    }
    snprintf(msg, sizeof(msg), "Sensor detected, Chip ID: 0x%04X\r\n", chip_id);
    printf(msg);

    /* Set format to JPEG */
    arducam_set_format(&cam, ARDUCAM_FMT_JPEG);

    /* Initialize camera sensor */
    status = arducam_init_cam(&cam);
    if (status != HAL_OK) {
        printf("Camera sensor init failed!\r\n");
        return status;
    }
    printf("Camera sensor initialized\r\n");

    /* Set VSYNC polarity - CRITICAL for capture to work */
    status = arducam_set_bit(&cam, ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    if (status != HAL_OK) {
        printf("Failed to set VSYNC level!\r\n");
        return status;
    }
    printf("VSYNC level set\r\n");

    HAL_Delay(1000);  /* Wait for sensor to stabilize */

    /* Set resolution to 640x480 */
    status = arducam_ov5642_set_jpeg_size(&cam, OV5642_640x480);
    if (status != HAL_OK) {
        printf("Failed to set resolution!\r\n");
        return status;
    }
    printf("Resolution set to 640x480\r\n");

    HAL_Delay(1000);  /* Wait for sensor to stabilize */

    /* Enter standby mode to reduce power consumption and heat */
    status = arducam_enter_standby(&cam);
    if (status != HAL_OK) {
        printf("Failed to enter standby mode!\r\n");
        return status;
    }
    printf("Camera in standby mode (low power)\r\n");

    return HAL_OK;
}

/**
 * @brief Start UART receive for command byte (non-blocking)
 */
static void start_uart_receive(void)
{
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_command, 1);
}

/**
 * @brief UART receive complete callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        rx_data_ready = 1;
    }
}

/**
 * @brief Send string via UART
 */
static void uart_send_string(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
}

/**
 * @brief Send image data with frame markers
 */
static void send_image_data(uint8_t *data, uint32_t length)
{
    uint8_t header[6];
    uint8_t footer[2];

    /* Start marker */
    header[0] = FRAME_START_MARKER_0;
    header[1] = FRAME_START_MARKER_1;

    /* Length (4 bytes, little-endian) */
    header[2] = (length >> 0) & 0xFF;
    header[3] = (length >> 8) & 0xFF;
    header[4] = (length >> 16) & 0xFF;
    header[5] = (length >> 24) & 0xFF;

    /* End marker */
    footer[0] = FRAME_END_MARKER_0;
    footer[1] = FRAME_END_MARKER_1;

    /* Send header */
    HAL_UART_Transmit(&huart2, header, 6, 1000);

    /* Send image data in chunks */
    uint32_t remaining = length;
    uint8_t *ptr = data;
    while (remaining > 0) {
        uint16_t chunk = (remaining > 1024) ? 1024 : (uint16_t)remaining;
        HAL_UART_Transmit(&huart2, ptr, chunk, 1000);
        ptr += chunk;
        remaining -= chunk;
    }

    /* Send footer */
    HAL_UART_Transmit(&huart2, footer, 2, 1000);
}

/**
 * @brief Capture image and send to host
 */
static HAL_StatusTypeDef capture_and_send_image(void)
{
    HAL_StatusTypeDef status;
    uint32_t fifo_length = 0;
    uint8_t len1, len2, len3;

    /* Debug: Read raw FIFO size registers */
    arducam_read_reg(&cam, 0x42, &len1);
    arducam_read_reg(&cam, 0x43, &len2);
    arducam_read_reg(&cam, 0x44, &len3);
    printf("Raw FIFO regs: 0x42=%02X, 0x43=%02X, 0x44=%02X\r\n", len1, len2, len3);

    /* Read FIFO length */
    status = arducam_read_fifo_length(&cam, &fifo_length);
    if (status != HAL_OK) {
        printf("Failed to read FIFO length\r\n");
        return status;
    }

    printf("FIFO length: %lu bytes\r\n", fifo_length);

    /* Validate length */
    if (fifo_length == 0 || fifo_length > IMAGE_BUFFER_SIZE) {
        printf("Invalid FIFO length: %lu\r\n", fifo_length);
        return HAL_ERROR;
    }

    /* Read image data from FIFO */
    status = arducam_read_fifo_burst(&cam, image_buffer, fifo_length);
    if (status != HAL_OK) {
        printf("Failed to read FIFO data\r\n");
        return status;
    }

    /* Debug: Check JPEG markers */
    printf("First bytes: %02X %02X, Last bytes: %02X %02X\r\n",
           image_buffer[0], image_buffer[1],
           image_buffer[fifo_length-2], image_buffer[fifo_length-1]);

    /* Send image to host */
    send_image_data(image_buffer, fifo_length);

    return HAL_OK;
}

/**
 * @brief Process command from host
 */
static void process_command(uint8_t cmd)
{
    HAL_StatusTypeDef status;

    switch (cmd) {
        /* Resolution commands */
        case CMD_SET_RES_320x240:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_320x240);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_320x240");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_640x480:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_640x480);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_640x480");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_1024x768:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_1024x768);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_1024x768");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_1280x960:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_1280x960);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_1280x960");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_1600x1200:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_1600x1200);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_1600x1200");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_2048x1536:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_2048x1536);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_2048x1536");
            }
            HAL_Delay(1000);
            break;

        case CMD_SET_RES_2592x1944:
            status = arducam_ov5642_set_jpeg_size(&cam, OV5642_2592x1944);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD switch to OV5642_2592x1944");
            }
            HAL_Delay(1000);
            break;

        /* Single capture */
        case CMD_SINGLE_CAPTURE:
            capture_mode = MODE_SINGLE;
            start_capture = 1;
            break;

        /* Re-initialize JPEG mode */
        case CMD_INIT_JPEG:
            arducam_set_format(&cam, ARDUCAM_FMT_JPEG);
            arducam_init_cam(&cam);
            arducam_set_bit(&cam, ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
            uart_send_string("ACK CMD CAM JPEG mode initialized");
            break;

        /* Start streaming */
        case CMD_START_STREAMING:
            arducam_exit_standby(&cam);
            HAL_Delay(100);
            capture_mode = MODE_STREAMING;
            start_capture = 2;
            uart_send_string("ACK CMD CAM start video streaming.");
            break;

        /* Stop streaming */
        case CMD_STOP_STREAMING:
            capture_mode = MODE_IDLE;
            start_capture = 0;
            arducam_enter_standby(&cam);
            uart_send_string("ACK CMD CAM stop video streaming. END");
            break;

        /* Quality commands */
        case CMD_QUALITY_HIGH:
            status = arducam_ov5642_set_compress_quality(&cam, QUALITY_HIGH);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD Set to high quality");
            }
            break;

        case CMD_QUALITY_DEFAULT:
            status = arducam_ov5642_set_compress_quality(&cam, QUALITY_DEFAULT);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD Set to default quality");
            }
            break;

        case CMD_QUALITY_LOW:
            status = arducam_ov5642_set_compress_quality(&cam, QUALITY_LOW);
            if (status == HAL_OK) {
                uart_send_string("ACK CMD Set to low quality");
            }
            break;

        default:
            /* Unknown command - ignore */
            break;
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
