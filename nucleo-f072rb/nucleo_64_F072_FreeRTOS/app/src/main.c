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
void vTaskConsole(void *pvParameters);

// kernel objects
QueueHandle_t xConsoleQueue;

// define the message_t type as an array of 64 char
typedef uint8_t msg_t[64];

int main()
{
	SystemClock_Config();

	BSP_LED_Init();
	BSP_PB_Init();
	BSP_Console_Init();

	xConsoleQueue = xQueueCreate(4, sizeof(msg_t));

	xTaskCreate(vTask1, "Task_1", 256, NULL, 3, NULL);
	xTaskCreate(vTask2, "Task_2", 256, NULL, 2, NULL);
	xTaskCreate(vTaskConsole, 	"Task_Console", 256, NULL, 1, NULL);

	// start the scheduler
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
 *	Task1
 */
void vTask1 (void *pvParameters)
{
	msg_t msg;
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		my_sprintf((char*)msg, "with great power comes great responsibility\r\n");

		xQueueSendToBack(xConsoleQueue, &msg, 0);

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
	}
}

/*
 *	Task2
 */
void vTask2 (void *pvParameters)
{
	msg_t msg;
	uint8_t index = 0;
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		my_sprintf((char*)msg, "%d# ", index);

		xQueueSendToBack(xConsoleQueue, &msg, 0);

		(index == 9) ? index = 0 : index++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
	}
}

/*
 * Task_Console
 */
void vTaskConsole (void *pvParameters)
{
	msg_t msg;

	while(1)
	{
		// Wait for something in the message Queue
		xQueueReceive(xConsoleQueue, &msg, portMAX_DELAY);

		// Send message to console
		my_printf((char *)msg);
	}
}

