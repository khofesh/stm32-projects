/**
  ******************************************************************************
  * @file    mx_rtos_abs.h
  * @author  MCD Application Team
  * @brief   Header for mx_wifi.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MX_RTOS_ABS_H
#define MX_RTOS_ABS_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mx_wifi_conf.h"
#include "stdlib.h"
#include "stddef.h"


#if MX_WIFI_USE_CMSIS_OS
  // existing CMSIS-OS stuff
#else
  /* No-OS queue structure */
  typedef struct noos_queue_s
  {
    uint16_t len;
    uint16_t in;
    uint16_t idx;
    uint16_t rd;
    uint16_t wr;
    void **fifo;
  } noos_queue_t;

  /* No-OS function declarations */
  int32_t noos_sem_signal(volatile uint32_t *sem);
  int32_t noos_sem_wait(__IO uint32_t *sem, uint32_t timeout, void (*idle_func)(uint32_t duration));
  int32_t noos_fifo_init(noos_queue_t **qret, uint16_t len);
  void noos_fifo_deinit(noos_queue_t *q);
  int32_t noos_fifo_push(noos_queue_t *queue, void *p, uint32_t timeout, void (*idle_func)(uint32_t duration));
  void *noos_fifo_pop(noos_queue_t *queue, uint32_t timeout, void (*idle_func)(uint32_t duration));

#endif /* MX_WIFI_USE_CMSIS_OS */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MX_RTOS_ABS_H */
