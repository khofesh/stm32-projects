/*
 * main.c
 *
 *  Created on: Sep 2, 2026
 *      Author: fahmad
 */

#include "stm32f0xx.h"

int main()
{
	uint32_t i;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	while(1)
	{
		GPIOA->ODR ^= GPIO_ODR_5;

		for (i=0; i < 100000; i++);
	}
}
