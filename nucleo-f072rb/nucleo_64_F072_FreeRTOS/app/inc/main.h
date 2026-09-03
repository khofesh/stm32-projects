/*
 * main.h
 *
 *  Created on: Sep 2, 2026
 *      Author: fahmad
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f0xx.h"
#include "bsp.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"

// printf() and sprintf() from printf-stdarg.c
int my_printf(const char *format, ...);
int my_sprintf(char *out, const char *format, ...);

#endif /* MAIN_H_ */
