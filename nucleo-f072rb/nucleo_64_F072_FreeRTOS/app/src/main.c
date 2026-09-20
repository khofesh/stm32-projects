/*
 * main.c
 *
 *  Created on: Sep 2, 2026
 *      Author: fahmad
 */

#include "stm32f0xx.h"
#include "main.h"

static void SystemClock_Config();

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	(void)xTask;
	my_printf("STACK OVERFLOW: %s\r\n", pcTaskName);
	for(;;) {}
}

//FreeRTOS tasks
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTaskHWM(void *pvParameters);

TaskHandle_t vTask1_handle;
TaskHandle_t vTask2_handle;
TaskHandle_t vTaskHWM_handle;

SemaphoreHandle_t xSem;
SemaphoreHandle_t xConsoleMutex;
QueueHandle_t xConsoleQueue;

// Define the message_t type as an array of 64 char
typedef uint8_t message_t[64];

int main()
{
	uint32_t free_heap_size;

	SystemClock_Config();

	BSP_LED_Init();
	BSP_PB_Init();
	BSP_Console_Init();

	// Report Free Heap Size
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("\r\nFree Heap Size is %d bytes\r\n", free_heap_size);

	// Create Semaphore object (this is not a 'give')
	my_printf("\r\nNow creating Binary Semaphore...\r\n");
	xSem = xSemaphoreCreateBinary();
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

	// Create Queue to hold console messages
	my_printf("\r\nNow creating Message Queue...\r\n");
	xConsoleQueue = xQueueCreate(4, sizeof(message_t));
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

	// Create a Mutex for accessing the console
	my_printf("\r\nNow creating Mutex...\r\n");
	xConsoleMutex = xSemaphoreCreateMutex();
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

	// Register the Trace User Event Channels
	my_printf("\r\nNow registering Trace events...\r\n");
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

	// Create Tasks
	my_printf("\r\nNow creating Tasks...\r\n");
	xTaskCreate(vTask1,	"Task_1",	128, NULL, 2, &vTask1_handle);
	xTaskCreate(vTask2,	"Task_2",	128, NULL, 3, &vTask2_handle);
	xTaskCreate(vTaskHWM,	"Task_HWM",	128, NULL, 1, &vTaskHWM_handle);
	free_heap_size = xPortGetFreeHeapSize();
	my_printf("Free Heap Size is %d bytes\r\n", free_heap_size);

	// Start the Scheduler
	my_printf("\r\nNow Starting Scheduler...\r\n");
	vTaskStartScheduler();

	while(1)
	{

	}
}

/*
 * clock configuration for this board
 * HSE input Bypass Mode -> 8MHz
 * SYSCLK, AHB, APB1 -> 48MHz
 * 	PA8 as MCO with /16 prescaler -> 3MHz
 */
static void SystemClock_Config()
{
	uint32_t HSE_Status;
	uint32_t PLL_Status;
	uint32_t SW_Status;
	uint32_t timeout = 0;
	timeout = 1000000;

	// start HSE in bypass mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// wait until HSE is ready
	do
	{
		HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while((HSE_Status == 0) && (timeout > 0 ));

	// select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// wait until PLL is ready
	do
	{
		PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((PLL_Status == 0) && (timeout > 0));

	// set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	// set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// enable FLASH prefetch buffer and set Flash latency
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	/* --- Until this point, MCU was still clocked by HSI at 8MHz ---*/
	/* --- Switching to PLL at 48MHz Now!  Fasten your seat belt! ---*/
	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// Wait until PLL becomes main switch input
	do
	{
		SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));
	/* --- Here we go! ---*/
	/*--- Use PA8 as MCO output at 48/16 = 3MHz ---*/
	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;
	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);
	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);
	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
}

/*
 *	Task_1
 */
void vTask1(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
	}
}

/*
 *	Task_2
 */
void vTask2(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		// Wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));
	}
}

/*
 * vTaskHWM
 */
void vTaskHWM (void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint32_t	count;
	uint16_t	hwm_Task1, hwm_Task2, hwm_TaskHWM;
	uint32_t	free_heap_size;

	count = 0;

	// Prepare console layout using ANSI escape sequences
	my_printf("%c[0m",   0x1B);	// Remove all text attributes
	my_printf("%c[2J",   0x1B); 	// Clear console
	my_printf("%c[1;0H", 0x1B);	// Move cursor [1:0]

	my_printf("High Water Marks console");

	my_printf("%c[3;0H", 0x1B);	// Move cursor line 3
	my_printf("Iteration");

	my_printf("%c[4;0H", 0x1B);	// Move cursor line 4
	my_printf("Task1");

	my_printf("%c[5;0H", 0x1B);	// Move cursor line 5
	my_printf("Task2");

	my_printf("%c[6;0H", 0x1B);	// Move cursor line 6
	my_printf("TaskHWM");

	my_printf("%c[7;0H", 0x1B);	// Move cursor line 7
	my_printf("Free Heap");

	while(1)
	{
		  // Gather High Water Marks
		  hwm_Task1	= uxTaskGetStackHighWaterMark(vTask1_handle);
		  hwm_Task2 	= uxTaskGetStackHighWaterMark(vTask2_handle);
		  hwm_TaskHWM	= uxTaskGetStackHighWaterMark(vTaskHWM_handle);

		  // Get free Heap size
		  free_heap_size = xPortGetFreeHeapSize();

		  // Display results into console
		  my_printf("%c[0;31;40m", 0x1B); 	// Red over black

		  my_printf("%c[3;12H", 0x1B);
		  my_printf("%5d", count);

		  my_printf("%c[1;33;44m", 0x1B); 	// Yellow over blue

		  my_printf("%c[4;12H", 0x1B);
		  my_printf("%5d", hwm_Task1);

		  my_printf("%c[5;12H", 0x1B);
		  my_printf("%5d", hwm_Task2);

		  my_printf("%c[6;12H", 0x1B);
		  my_printf("%5d", hwm_TaskHWM);

		  my_printf("%c[1;35;40m", 0x1B); 	// Majenta over black
		  my_printf("%c[7;12H", 0x1B);
		  my_printf("%5d", free_heap_size);

		  my_printf("%c[0m", 0x1B); 		// Remove all text attributes
		  count++;

		// Wait for 500ms
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
	}
}

