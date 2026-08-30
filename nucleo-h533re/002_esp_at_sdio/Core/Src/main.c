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
#include <string.h>
#include "sdio_host_transport.h"
#include "sdio_host_reg.h"
#include "sdio_host_log.h"
#include "sdio_driver_port.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AT_CMD_BUF_LEN   256U   /* one AT command line from the console */
#define AT_RSP_BUF_LEN   4096U  /* one AT response chunk from the slave */
#define AT_RX_ERR_REINIT_COUNT  4U
/* AT+RESTORE answers "OK" before it erases NVS and restarts, so the link stays
   usable for a moment. Keep draining until AT_RESTORE_DRAIN_MS so that "OK"
   reaches the console, then leave the bus alone until AT_RESTORE_REINIT_MS -
   clocking a slave that is mid-reboot only produces noise. */
#define AT_RESTORE_DRAIN_MS     600U
#define AT_RESTORE_REINIT_MS    2500U
#define AT_REINIT_RETRY_MS      500U
/* Re-enumeration alone cannot rescue a slave that came up wedged; after this
   many soft attempts the recovery escalates to the ESP reset pin. */
#define AT_REINIT_HARD_RESET_AT 3U
/* A single pass never needs more than this many CMD53 reads. Purely a stop for
   a slave that keeps reporting a queue it never drains - without it the drain
   loop is the one place the console can hang for good. */
#define AT_RX_DRAIN_MAX_PKTS    64U

#define AT_RESTORE_IDLE         0U
#define AT_RESTORE_DRAINING     1U
#define AT_RESTORE_QUIET        2U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

DCACHE_HandleTypeDef hdcache1;

SD_HandleTypeDef hsd1;

/* USER CODE BEGIN PV */
/* CMD53 buffers: the SDIO port requires 4-byte alignment */
static uint8_t at_cmd_buf[AT_CMD_BUF_LEN] __attribute__((aligned(4)));
static uint8_t at_rsp_buf[AT_RSP_BUF_LEN] __attribute__((aligned(4)));

static volatile uint16_t at_cmd_len;
static volatile uint8_t  at_cmd_ready;
static uint16_t at_cmd_idx;
static uint8_t at_rx_err_count;
static uint8_t at_restore_state;
static uint8_t at_reinit_fail_count;
static uint32_t at_restore_drain_at;
static uint32_t at_restore_reinit_at;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCACHE1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t AtUpper(uint8_t ch)
{
  if ((ch >= (uint8_t)'a') && (ch <= (uint8_t)'z'))
  {
    ch = (uint8_t)(ch - ((uint8_t)'a' - (uint8_t)'A'));
  }
  return ch;
}

/**
  * @brief Does this line make the slave restart out from under the host?
  *
  * AT+RESTORE erases the parameter area and reboots; AT+RST just reboots. Both
  * leave the SDIO link enumerated against a card that no longer exists, so both
  * have to be followed by a re-enumeration rather than by more traffic.
  *
  * @param buf  Command line, terminated with CRLF by the console handler.
  */
static uint8_t AtCommandRebootsSlave(const uint8_t *buf, uint16_t len)
{
  static const char *const cmds[] = { "AT+RESTORE", "AT+RST" };
  uint8_t c;

  for (c = 0U; c < (uint8_t)(sizeof(cmds) / sizeof(cmds[0])); c++)
  {
    uint16_t n = (uint16_t)strlen(cmds[c]);
    uint16_t i;

    /* The CRLF is part of the line, so an exact match is n + 2 long */
    if (len < (uint16_t)(n + 2U))
    {
      continue;
    }

    for (i = 0U; i < n; i++)
    {
      if (AtUpper(buf[i]) != (uint8_t)cmds[c][i])
      {
        break;
      }
    }

    if ((i == n) && ((buf[i] == (uint8_t)'\r') || (buf[i] == (uint8_t)'\n')))
    {
      return 1U;
    }
  }

  return 0U;
}

/**
  * @brief Bring the link back up after the slave has restarted underneath us.
  *
  * Re-enumeration on its own (CMD0/CMD5/CMD3/CMD7 with the ESP left running) is
  * what an AT+RESTORE needs and is the cheap case. It cannot help a slave that
  * booted into a bad state, so a run of failures falls back to sdio_host_init(),
  * which pulls the ESP reset pin - the one recovery that always terminates.
  */
static sdio_err_t AtSdioRecover(void)
{
  sdio_err_t err = sdio_host_reinit();

  if (err == SDIO_SUCCESS)
  {
    at_reinit_fail_count = 0U;
    return err;
  }

  at_reinit_fail_count++;

  if (at_reinit_fail_count >= AT_REINIT_HARD_RESET_AT)
  {
    printf("soft reinit failed %u times, resetting slave\r\n",
           (unsigned int)at_reinit_fail_count);
    at_reinit_fail_count = 0U;
    err = sdio_host_init();
  }

  return err;
}

/**
  * @brief Read queued packets out to the console until the slave reports none.
  *
  * @param quiet  Non-zero while the slave is on its way down after AT+RESTORE:
  *               a failed read there is expected, so it is neither reported nor
  *               counted towards the link-lost threshold.
  */
static void AtDrainRx(uint8_t quiet)
{
  uint32_t pkts;

  for (pkts = 0U; pkts < AT_RX_DRAIN_MAX_PKTS; pkts++)
  {
    size_t size_read = 0U;
    /* wait_ms = 1 makes sdio_host_get_packet() check once and return
       ERR_TIMEOUT when the slave has nothing queued. */
    sdio_err_t err = sdio_host_get_packet(at_rsp_buf, AT_RSP_BUF_LEN, &size_read, 1U);

    if ((err == ERR_TIMEOUT) || (err == ERR_NOT_FOUND))
    {
      if (quiet == 0U)
      {
        at_rx_err_count = 0U;
      }
      break;
    }

    if ((err != SDIO_SUCCESS) && (err != ERR_NOT_FINISHED))
    {
      if (quiet != 0U)
      {
        break;
      }

      /* Retried on the next pass rather than abandoned - the data is still
         queued at the slave and nothing else will come back for it. */
      printf("rx packet error: %d\r\n", (int)err);
      at_rx_err_count++;

      if (at_rx_err_count >= AT_RX_ERR_REINIT_COUNT)
      {
        printf("SDIO link lost, reinitializing...\r\n");

        if (AtSdioRecover() == SDIO_SUCCESS)
        {
          printf("Sdio reinit done\r\n");
          at_rx_err_count = 0U;
        }
        else
        {
          printf("SDIO reinit error\r\n");
          at_rx_err_count = AT_RX_ERR_REINIT_COUNT - 1U;
        }
      }
      break;
    }

    if (size_read != 0U)
    {
      (void)HAL_UART_Transmit(&hcom_uart[COM1], at_rsp_buf, (uint16_t)size_read, 1000U);
      at_rx_err_count = 0U;
    }
  }
}

/**
  * @brief Drive the wait-then-re-enumerate sequence that follows AT+RESTORE.
  *
  * @return Non-zero while the restore is still in progress, i.e. while the main
  *         loop must not send anything of its own.
  */
static uint8_t AtRestoreService(void)
{
  uint32_t now = HAL_GetTick();

  if (at_restore_state == AT_RESTORE_DRAINING)
  {
    if ((int32_t)(now - at_restore_drain_at) < 0)
    {
      /* The slave stops answering partway through this window, by design */
      sdio_driver_set_quiet(1U);
      AtDrainRx(1U);
      sdio_driver_set_quiet(0U);
      return 1U;
    }
    at_restore_state = AT_RESTORE_QUIET;
  }

  if ((int32_t)(now - at_restore_reinit_at) < 0)
  {
    return 1U;
  }

  printf("Reinitializing SDIO after slave restart...\r\n");

  if (AtSdioRecover() == SDIO_SUCCESS)
  {
    printf("Sdio reinit done\r\n");
    at_restore_state = AT_RESTORE_IDLE;
    at_rx_err_count = 0U;
    return 0U;
  }

  printf("SDIO reinit error\r\n");
  at_restore_reinit_at = HAL_GetTick() + AT_REINIT_RETRY_MS;
  return 1U;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DCACHE1_Init();
  MX_ICACHE_Init();
  MX_SDMMC1_SD_Init();
  /* USER CODE BEGIN 2 */
  /* Report MemManage/BusFault/UsageFault on their own vectors instead of
     letting them escalate into an anonymous HardFault */
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk |
                SCB_SHCSR_BUSFAULTENA_Msk |
                SCB_SHCSR_MEMFAULTENA_Msk;
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

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

  /* USER CODE BEGIN WHILE */
  BSP_COM_SelectLogPort(COM1);

  printf("\r\nhost ready, start initializing slave...\r\n");
  if (sdio_host_init() != SDIO_SUCCESS)
  {
    printf("SDIO init error\r\n");
    Error_Handler();
  }
  printf("Sdio init done\r\n");
  BSP_LED_On(LED_GREEN);

  /* Console echoes AT commands typed by the user into the SDIO link. RX is
     driven off the registers in USART2_IRQHandler(), not HAL_UART_Receive_IT():
     printf() blocks inside HAL_UART_Transmit() on the same handle and holds
     huart->Lock, which would make every re-arm from the ISR fail with
     HAL_UART_STATE_BUSY and silently kill the console. */
  __HAL_UART_CLEAR_FLAG(&hcom_uart[COM1], UART_CLEAR_OREF | UART_CLEAR_FEF |
                                          UART_CLEAR_NEF | UART_CLEAR_PEF);
  SET_BIT(COM1_UART->CR1, USART_CR1_RXNEIE_RXFNEIE);

  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (at_restore_state != AT_RESTORE_IDLE)
    {
      if (AtRestoreService() != 0U)
      {
        continue;
      }
    }

    if (at_cmd_ready != 0U)
    {
      uint8_t reboots = AtCommandRebootsSlave(at_cmd_buf, at_cmd_len);
      sdio_err_t err = sdio_host_send_packet(at_cmd_buf, at_cmd_len);

      if (err == ERR_TIMEOUT)
      {
        printf("send timeout\r\n");
      }
      else if (err != SDIO_SUCCESS)
      {
        printf("send error: %d\r\n", (int)err);
      }
      else if (reboots != 0U)
      {
        printf("reboot command sent, waiting for slave restart\r\n");
        at_restore_state = AT_RESTORE_DRAINING;
        at_restore_drain_at = HAL_GetTick() + AT_RESTORE_DRAIN_MS;
        at_restore_reinit_at = HAL_GetTick() + AT_RESTORE_REINIT_MS;
        at_rx_err_count = 0U;
        at_reinit_fail_count = 0U;
      }
      at_cmd_ready = 0U;
      if (at_restore_state != AT_RESTORE_IDLE)
      {
        continue;
      }
    }

    /* Slave pulls D1 low when it has a packet for us. The interrupt only
       shortens the idle wait - it is never the thing that decides whether to
       read, because it has to be cleared before the packet can be fetched. A
       read that fails after that point (one transient CRC on the length
       register is enough) leaves the queued packet with no edge left to
       announce it: the host stops draining, the slave's send queue fills, its
       token count stops advancing, and every later AT command ends in
       "buffer is not enough: 0, 1 required" for good. So the queue is polled
       on every pass and the interrupt is treated as a hint. */
    if (sdio_host_wait_int(10U) == SDIO_SUCCESS)
    {
      uint32_t intr_raw = 0U;
      uint32_t intr_st = 0U;

      if ((sdio_host_get_intr(&intr_raw, &intr_st) == SDIO_SUCCESS) && (intr_st != 0U))
      {
        (void)sdio_host_clear_intr(intr_raw);
      }
    }

    /* Drain until the slave reports nothing queued */
    AtDrainRx(0U);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief DCACHE1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCACHE1_Init(void)
{

  /* USER CODE BEGIN DCACHE1_Init 0 */

  /* USER CODE END DCACHE1_Init 0 */

  /* USER CODE BEGIN DCACHE1_Init 1 */

  /* USER CODE END DCACHE1_Init 1 */
  hdcache1.Instance = DCACHE1;
  hdcache1.Init.ReadBurstType = DCACHE_READ_BURST_WRAP;
  if (HAL_DCACHE_Init(&hdcache1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCACHE1_Init 2 */

  /* USER CODE END DCACHE1_Init 2 */

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
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */
  /* SDMMC1 is used in SDIO mode (ESP-AT slave), not as an SD card. The SD card
     identification below would fail and trap in Error_Handler(); the peripheral
     is brought up by sdio_driver_init() instead. */
  return;
  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  * @brief  Console RX: accumulate one AT command line, then hand it to the main
  *         loop. Kept short - the SDIO transfer happens outside.
  *
  * Either CR or LF ends the line; the CRLF that ESP-AT expects is appended here
  * so a terminal that sends only one of the two still works. Two bytes of head-
  * room are always kept for it.
  */
void AtConsoleRxIrq(void)
{
  uint32_t isr = COM1_UART->ISR;
  uint8_t ch;

  /* An uncleared ORE/FE/NE/PE would re-enter this handler forever */
  if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U)
  {
    COM1_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                     USART_ICR_NECF | USART_ICR_PECF;
  }

  if ((isr & USART_ISR_RXNE_RXFNE) == 0U)
  {
    return;
  }

  ch = (uint8_t)COM1_UART->RDR;

  /* Echo so the typed line is visible. Non-blocking on purpose: the SDIO
     polling loops in the main context must not be stalled by this handler. */
  if ((COM1_UART->ISR & USART_ISR_TXE_TXFNF) != 0U)
  {
    COM1_UART->TDR = ch;
  }

  /* Previous command not consumed yet: drop the byte rather than race main */
  if (at_cmd_ready != 0U)
  {
    return;
  }

  if ((ch == '\r') || (ch == '\n'))
  {
    if (at_cmd_idx == 0U)
    {
      return;   /* empty line, or the second half of a CRLF pair */
    }

    at_cmd_buf[at_cmd_idx] = '\r';
    at_cmd_buf[at_cmd_idx + 1U] = '\n';
    at_cmd_len = at_cmd_idx + 2U;
    at_cmd_idx = 0U;
    at_cmd_ready = 1U;
    return;
  }

  if ((ch == '\b') || (ch == 0x7FU))
  {
    if (at_cmd_idx != 0U)
    {
      at_cmd_idx--;
    }
    return;
  }

  /* Line too long: drop it and resynchronise on the next terminator */
  if (at_cmd_idx >= (AT_CMD_BUF_LEN - 2U))
  {
    at_cmd_idx = 0U;
    return;
  }

  at_cmd_buf[at_cmd_idx] = ch;
  at_cmd_idx++;
}

/**
  * @brief  Dump the fault status registers and the stacked exception frame.
  *
  * Writes straight to the USART data register: the HAL handle may be locked or
  * mid-transfer at the point of the fault, and printf() must not be re-entered
  * from a handler.
  */
static void FaultPutc(char c)
{
  while ((COM1_UART->ISR & USART_ISR_TXE_TXFNF) == 0U)
  {
  }
  COM1_UART->TDR = (uint8_t)c;
}

static void FaultPuts(const char *s)
{
  while (*s != '\0')
  {
    FaultPutc(*s);
    s++;
  }
}

static void FaultHex(const char *label, uint32_t v)
{
  static const char digits[] = "0123456789ABCDEF";
  int8_t i;

  FaultPuts(label);
  FaultPuts(" 0x");
  for (i = 28; i >= 0; i -= 4)
  {
    FaultPutc(digits[(v >> (uint8_t)i) & 0xFU]);
  }
  FaultPuts("\r\n");
}

void FaultReport(const uint32_t *sp, const char *name)
{
  const uint32_t *frame = NULL;
  uint32_t i;

  FaultPuts("\r\n*** ");
  FaultPuts(name);
  FaultPuts(" ***\r\n");

  FaultHex("CFSR ", SCB->CFSR);
  FaultHex("HFSR ", SCB->HFSR);
  FaultHex("MMFAR", SCB->MMFAR);
  FaultHex("BFAR ", SCB->BFAR);

  /* sp is the handler's SP after its own prologue, so walk up to the stacked
     frame: PC in flash and the xPSR Thumb bit set identify it. */
  for (i = 0U; i < 24U; i++)
  {
    uint32_t pc = sp[i + 6U];
    uint32_t psr = sp[i + 7U];

    if ((pc >= 0x08000000UL) && (pc < 0x08080000UL) && ((psr & 0x01000000UL) != 0U))
    {
      frame = &sp[i];
      break;
    }
  }

  if (frame == NULL)
  {
    FaultPuts("stack frame not found\r\n");
    return;
  }

  FaultHex("R0   ", frame[0]);
  FaultHex("R1   ", frame[1]);
  FaultHex("R2   ", frame[2]);
  FaultHex("R3   ", frame[3]);
  FaultHex("R12  ", frame[4]);
  FaultHex("LR   ", frame[5]);
  FaultHex("PC   ", frame[6]);
  FaultHex("xPSR ", frame[7]);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
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
