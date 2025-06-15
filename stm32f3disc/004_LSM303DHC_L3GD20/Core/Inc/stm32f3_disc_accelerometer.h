/*
 * stm32f3_disc_accelerometer.h
 *
 *  Created on: Mar 15, 2025
 *      Author: fahmad
 */

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery.h"
/* Include Gyroscope component driver */
#include "../../Drivers/LSM303DLHC/lsm303dlhc.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY_ACCELEROMETER
  * @{
  */

/** @defgroup STM32F3_DISCOVERY_ACCELEROMETER_Exported_Types Exported Types
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELEROMETER_Exported_Constants Exported Constants
  * @{
  */
typedef enum
{
  ACCELERO_OK = 0,
  ACCELERO_ERROR = 1,
  ACCELERO_TIMEOUT = 2
}
ACCELERO_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELEROMETER_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_ACCELEROMETER_Exported_Functions Exported Functions
  * @{
  */
/* Acc functions */
uint8_t   BSP_ACCELERO_Init(void);
void      BSP_ACCELERO_Reset(void);
void      BSP_ACCELERO_GetXYZ(int16_t *pDataXYZ);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif
