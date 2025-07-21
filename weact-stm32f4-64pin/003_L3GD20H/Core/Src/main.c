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
#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "l3gd20h_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOT_TIME 10 // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static int16_t data_raw_angular_rate[3];
static float_t angular_rate_mdps[3];
static uint8_t whoamI;
static l3gd20h_status_reg_t status;
// static uint8_t rst;
static uint8_t tx_buffer[1000];
RingBuffer txBuf, rxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
// static void platform_init(void);
void test_spi_communication(void);
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);
void l3gd20h_init(stmdev_ctx_t *dev_ctx);
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
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  platform_delay(BOOT_TIME);
  /* Initialize magnetic sensors driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hspi1;

  // test SPI communication
  test_spi_communication();

  // initialization
  l3gd20h_init(&dev_ctx);

  /* check device ID */
  l3gd20h_dev_id_get(&dev_ctx, &whoamI);

  if (whoamI != L3GD20H_ID)
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "WHO_AM_I Error: 0x%02X (expected 0x%02X)\r\n", whoamI, L3GD20H_ID);
    tx_com(tx_buffer, strlen((char *)tx_buffer));
    while (1)
    {
      /* manage here device not found */
    }
  }
  else
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "L3GD20H detected successfully! WHO_AM_I: 0x%02X\r\n", whoamI);
    tx_com(tx_buffer, strlen((char *)tx_buffer));
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Read device status register */
    l3gd20h_dev_status_get(&dev_ctx, &status);

    if (status.zyxda)
    {
      /* Read IMU data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));

      if (l3gd20h_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate) == 0)
      {
        // convert to milli-degrees per second
        angular_rate_mdps[0] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate[0]);
        angular_rate_mdps[1] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate[1]);
        angular_rate_mdps[2] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate[2]);

        // print both raw and converted values for debugging
        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                 "Raw: X=%d Y=%d Z=%d | mdps: X=%.2f Y=%.2f Z=%.2f\r\n",
                 data_raw_angular_rate[0], data_raw_angular_rate[1], data_raw_angular_rate[2],
                 angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char *)tx_buffer));
      }
      else
      {
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Failed to read angular rate data\r\n");
        tx_com(tx_buffer, strlen((char *)tx_buffer));
      }
    }
    else
    {
      // print status for debugging
      static uint32_t no_data_counter = 0;
      no_data_counter++;
      if (no_data_counter % 1000000 == 0)
      {
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "No new data available. Status: 0x%02X\r\n",
                 *(uint8_t *)&status);
        tx_com(tx_buffer, strlen((char *)tx_buffer));
      }
    }

    HAL_Delay(1000);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  // for L3GD20H, we need to clear the MSB for write operations
  uint8_t write_reg = reg & 0x7F;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);

  // send register address
  if (HAL_SPI_Transmit((SPI_HandleTypeDef *)handle, &write_reg, 1, 1000) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return -1;
  }

  // send data
  if (HAL_SPI_Transmit((SPI_HandleTypeDef *)handle, (uint8_t *)bufp, len, 1000) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return -1;
  }

  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  // for L3GD20H, set MSB for read operations, and if reading multiple bytes, set bit 6 for auto-increment
  uint8_t read_reg = reg | 0x80;
  if (len > 1)
  {
    // auto-increment for multi-byte reads
    read_reg |= 0x40;
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);

  // send register address with read bit set
  if (HAL_SPI_Transmit((SPI_HandleTypeDef *)handle, &read_reg, 1, 1000) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return -1;
  }

  // read data
  if (HAL_SPI_Receive((SPI_HandleTypeDef *)handle, bufp, len, 1000) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return -1;
  }

  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  //	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
  UART_Transmit(&huart1, tx_buffer, len);
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

// static void platform_init(void)
//{
//
// }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t data[100];
    uint16_t len = RingBuffer_Read(&txBuf, data, sizeof(data));
    if (len > 0)
    {
      HAL_UART_Transmit_IT(huart, data, len);
    }
  }
}

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

void test_spi_communication(void)
{
  uint8_t received_data = 0;

  snprintf((char *)tx_buffer, sizeof(tx_buffer), "Testing SPI communication...\r\n");
  UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char *)tx_buffer));

  // Read WHO_AM_I register directly
  if (platform_read(&hspi1, L3GD20H_WHO_AM_I, &received_data, 1) == 0)
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Manual test - WHO_AM_I: 0x%02X\r\n", received_data);
  }
  else
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Manual test - SPI communication failed\r\n");
  }
  UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen((char *)tx_buffer));
}

void l3gd20h_init(stmdev_ctx_t *dev_ctx)
{
  uint8_t rst;

  // 1. Disable I2C interface to ensure SPI mode
  l3gd20h_i2c_interface_set(dev_ctx, L3GD20H_I2C_DISABLE);
  platform_delay(10);

  // 2. Software reset
  l3gd20h_dev_reset_set(dev_ctx, PROPERTY_ENABLE);
  do
  {
    l3gd20h_dev_reset_get(dev_ctx, &rst);
    platform_delay(1);
  } while (rst);

  platform_delay(50);

  // 3. Enable Block Data Update
  l3gd20h_block_data_update_set(dev_ctx, PROPERTY_ENABLE);

  // 4. Set full scale
  l3gd20h_gy_full_scale_set(dev_ctx, L3GD20H_2000dps);

  // 5. Set Output Data Rate
  l3gd20h_gy_data_rate_set(dev_ctx, L3GD20H_100Hz);

  // 6. Enable all axis
  l3gd20h_gy_axis_t axis_enable = {0};
  axis_enable.xen = PROPERTY_ENABLE;
  axis_enable.yen = PROPERTY_ENABLE;
  axis_enable.zen = PROPERTY_ENABLE;
  l3gd20h_gy_axis_set(dev_ctx, axis_enable);

  platform_delay(100);
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
