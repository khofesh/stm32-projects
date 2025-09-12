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
#include "esp8266_stm32.h"
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
__IO ITStatus UartTxComplete = SET;
RingBuffer txBuf, rxBuf;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
  if (UART_Transmit(&huart1, (uint8_t *)ptr, len))
  {
    // Wait for transmission to complete to ensure log integrity
    while (UartTxComplete == RESET || RingBuffer_GetDataLength(&txBuf) > 0)
    {
      // Allow other interrupts to process
      __NOP();
    }
    return len;
  }
  return 0; // Return 0 on failure
}

char ip_buf[16];

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
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize ring buffers for UART interrupt-based transmission
  RingBuffer_Init(&txBuf);
  RingBuffer_Init(&rxBuf);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  if (ESP_Init() != ESP8266_OK)
  {
    USER_LOG("Failed to initialize... Check Debug logs");
    Error_Handler();
  }

  if (ESP_ConnectWiFi("Arun_Rawat", "arun@321", ip_buf, sizeof(ip_buf)) != ESP8266_OK)
  {
    USER_LOG("Failed to connect to wifi... Check Debug logs");
    Error_Handler();
  }

  // Initialize and connect to MQTT broker
  USER_LOG("Setting up MQTT...");
  if (ESP_MQTT_Init("test.mosquitto.org", 1883, "STM32_Device_001") != ESP8266_OK)
  {
    USER_LOG("MQTT initialization failed");
    Error_Handler();
  }

  // Set MQTT authentication
  // ESP_MQTT_SetAuth("your_username", "your_password");

  if (ESP_MQTT_Connect() != ESP8266_OK)
  {
    USER_LOG("MQTT connection failed");
    Error_Handler();
  }

  // Subscribe to control topics
  ESP_MQTT_Subscribe("stm32/led/control", MQTT_QOS_0);
  ESP_MQTT_Subscribe("stm32/system/command", MQTT_QOS_0);
  ESP_MQTT_Subscribe("stm32/sensor/request", MQTT_QOS_1);

  USER_LOG("MQTT setup complete. Device ready for IoT communication!");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_publish = 0;
  uint32_t publish_interval = 10000; // Publish every 10 seconds
  MQTT_Message received_msg;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Check for incoming MQTT messages
    if (ESP_MQTT_CheckMessages(&received_msg) == ESP8266_OK)
    {
      // Handle received MQTT messages
      if (strcmp(received_msg.topic, "stm32/led/control") == 0)
      {
        if (strcmp(received_msg.message, "ON") == 0)
        {
          USER_LOG("LED Control: Turning LED ON");
          // Add your LED control code here
          ESP_MQTT_Publish("stm32/led/status", "LED_ON", MQTT_QOS_0, 0);
        }
        else if (strcmp(received_msg.message, "OFF") == 0)
        {
          USER_LOG("LED Control: Turning LED OFF");
          // Add your LED control code here
          ESP_MQTT_Publish("stm32/led/status", "LED_OFF", MQTT_QOS_0, 0);
        }
      }
      else if (strcmp(received_msg.topic, "stm32/system/command") == 0)
      {
        if (strcmp(received_msg.message, "STATUS") == 0)
        {
          char status_msg[128];
          snprintf(status_msg, sizeof(status_msg),
                   "ONLINE,IP:%s,UPTIME:%lu", ip_buf, HAL_GetTick());
          ESP_MQTT_Publish("stm32/system/status", status_msg, MQTT_QOS_0, 0);
        }
        else if (strcmp(received_msg.message, "RESTART") == 0)
        {
          ESP_MQTT_Publish("stm32/system/status", "RESTARTING", MQTT_QOS_0, 0);
          HAL_Delay(1000);
          NVIC_SystemReset();
        }
      }
      else if (strcmp(received_msg.topic, "stm32/sensor/request") == 0)
      {
        // Simulate sensor data
        char sensor_data[128];
        uint32_t temp = 25 + (HAL_GetTick() % 10);     // Simulated temperature
        uint32_t humidity = 60 + (HAL_GetTick() % 20); // Simulated humidity

        snprintf(sensor_data, sizeof(sensor_data),
                 "{\"temperature\":%lu,\"humidity\":%lu,\"timestamp\":%lu}",
                 temp, humidity, HAL_GetTick());
        ESP_MQTT_Publish("stm32/sensor/data", sensor_data, MQTT_QOS_1, 0);
      }
    }

    // Periodic status publishing
    if ((HAL_GetTick() - last_publish) >= publish_interval)
    {
      char heartbeat[64];
      snprintf(heartbeat, sizeof(heartbeat), "ALIVE_%lu", HAL_GetTick());
      ESP_MQTT_Publish("stm32/heartbeat", heartbeat, MQTT_QOS_0, 0);

      last_publish = HAL_GetTick();
    }

    // Small delay to prevent busy waiting
    HAL_Delay(100);
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
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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
uint8_t UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
  UartTxComplete = RESET; // Mark transmission as in progress
  if (HAL_UART_Transmit_IT(huart, pData, len) != HAL_OK)
  {
    if (RingBuffer_Write(&txBuf, pData, len) != RING_BUFFER_OK)
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
  if (huart == &huart1)
  {
    if (RingBuffer_GetDataLength(&txBuf) > 0)
    {
      RingBuffer_Read(&txBuf, &txData, 1);
      HAL_UART_Transmit_IT(huart, &txData, 1);
    }
    else
    {
      UartTxComplete = SET; // Mark transmission as complete when buffer is empty
    }
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    // UART4 DMA RX half complete - update tail pointer for circular buffer
    // This helps in handling continuous reception
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart4)
  {
    // Handle UART4 errors - restart DMA reception if needed
    HAL_UART_DMAStop(&huart4);
    ESP_DMA_StartReceive();
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
