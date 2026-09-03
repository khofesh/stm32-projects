/*
 * printf-stdarg.c
 *
 *  Created on: Sep 3, 2026
 *      Author: fahmad
 */

#include <stdarg.h>
#include "stm32f0xx.h"

static void printchar(char **str, int c)
{
	extern int putchar(int c);
	if (str)
	{
		**str = c;
		++(*str);
	}
	else
	{
		while ( (USART2->ISR & USART_ISR_TC) != USART_ISR_TC);
		USART2->TDR = c;
	}
}

int my_printf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	return print(0, format, args);
}

int my_sprintf(char *out, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	return print(&out, format, args);
}
