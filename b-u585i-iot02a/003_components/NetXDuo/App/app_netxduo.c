/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
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
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_netxduo.h"
#include "app_azure_rtos.h"
#include "console.h"
#include "msg.h"
#include "nxd_dhcp_client.h"
#include "nxd_bsd.h"
#include <stdbool.h>
#include <inttypes.h>
#include "io_pattern/mx_wifi_io.h"
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
/* USER CODE BEGIN PV */
static TX_THREAD AppMainThread;
static CHAR AppMainThreadName[] = "App Main thread";

static TX_THREAD AppMain2Thread;
static CHAR AppMain2ThreadName[] = "App Main2 thread";

static __IO bool AppMain2ThreadRunning = true;

static TX_THREAD AppWiFiBasicsThread;
static CHAR AppWiFiBasicsThreadName[] = "App WiFiBasics thread";

/* Global byte pool pointer for ping and other modules */
TX_BYTE_POOL *AppBytePool = NULL;

static NX_PACKET_POOL AppPacketPool;

NX_IP IpInstance;
static CHAR IpInstanceName[] = "NetX IP Instance 0";

static NX_DHCP DhcpClient;
static TX_SEMAPHORE DhcpSemaphore;

static NX_DNS DnsClient;
#ifdef NX_DNS_CACHE_ENABLE
static UCHAR DnsLocalCache[2048];
#endif /* NX_DNS_CACHE_ENABLE */

static const cmd_t cmdlist[] =
{
  {"help",                                         "print this message",          help_cmd},
  {"quit",                                                       "quit",          quit_cmd},
  {"scan",                                                  "Wifi Scan",          scan_cmd},
  {"ping",                    "Ping <hostname> (www.st.com by default)",          ping_cmd},
  {  NULL,                                                         NULL,              NULL}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static VOID App_Main_Thread_Entry(ULONG thread_input);
static VOID App_Main2_Thread_Entry(ULONG thread_input);

static VOID App_WiFi_Thread_Entry(ULONG thread_input);

static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);

static void NetXDuo_DeInit(void);

static UINT dns_create(NX_DNS *dns_ptr);
/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
   /* USER CODE BEGIN App_NetXDuo_MEM_POOL */
  /* Store byte pool pointer for use by ping and other modules */
  AppBytePool = byte_pool;
  /* USER CODE END App_NetXDuo_MEM_POOL */
  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */

  /* USER CODE BEGIN MX_NetXDuo_Init */
  MSG_INFO("\n003_components application started.\n");

  MSG_INFO("# build: %s-%s, %s, %s %s\n",
		  "SPI",
		  "RTOS",
#if (defined(MX_WIFI_NETWORK_BYPASS_MODE) && (MX_WIFI_NETWORK_BYPASS_MODE == 1))
		  "Network on host",
#else
		  "Network on module",
#endif /* MX_WIFI_NETWORK_BYPASS_MODE */
		  __TIME__, __DATE__);

  MSG_INFO("\nTX_TIMER_TICKS_PER_SECOND: %" PRIu32 "\n", (uint32_t)TX_TIMER_TICKS_PER_SECOND);
  MSG_INFO("NX_IP_PERIODIC_RATE      : %" PRIu32 "\n\n", (uint32_t)NX_IP_PERIODIC_RATE);

  /* Initialize the NetX system. */
  nx_system_initialize();
  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
