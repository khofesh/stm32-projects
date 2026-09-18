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

SemaphoreHandle_t xSem;

int main()
{
	SystemClock_Config();

	BSP_LED_Init();
	BSP_PB_Init();
	BSP_Console_Init();

	xSem = xSemaphoreCreateBinary();

	xTaskCreate(vTask1, "Task_1", 256, NULL, 1, NULL);
	xTaskCreate(vTask2, "Task_2", 256, NULL, 2, NULL);

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
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		BSP_LED_Toggle();

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
	}
}

/*
 *	Task2
 */
void vTask2 (void *pvParameters)
{
	portBASE_TYPE	xStatus;

	// Initialize the user Push-Button
	BSP_PB_Init();

	// Set priority for EXTI line 4 to 15, and enable interrupt
	NVIC_SetPriority(EXTI4_15_IRQn, configMAX_API_CALL_INTERRUPT_PRIORITY + 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	while(1)
	{
		// Wait here for Semaphore with 100ms timeout
		xStatus = xSemaphoreTake(xSem, 100);

		// Test the result of the take attempt
		if (xStatus == pdPASS)
		{
			// The semaphore was taken as expected

			// Display console message
			my_printf("#");
		}

		else
		{
			// The 100ms timeout elapsed without Semaphore being taken

			// Display another message
			my_printf(".");
		}
	}
}


