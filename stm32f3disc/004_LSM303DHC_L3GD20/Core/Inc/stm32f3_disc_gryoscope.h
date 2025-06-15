/*
 * stm32f3_disc_gryoscope.h
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
#include "../../Drivers/L3GD20/l3gd20.h"


/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY_GYROSCOPE
  * @{
  */

/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Exported_Types Exported Types
  * @{
  */
typedef enum
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
}
GYRO_StatusTypeDef;

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Exported_Constants Exported Constants
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Exported_Functions  Exported Functions
  * @{
  */
/* Sensor Configuration Functions */
uint8_t BSP_GYRO_Init(void);
void BSP_GYRO_Reset(void);
uint8_t BSP_GYRO_ReadID(void);
void    BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void BSP_GYRO_EnableIT(uint8_t IntPin);
void BSP_GYRO_DisableIT(uint8_t IntPin);
void BSP_GYRO_GetXYZ(float* pfData);

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
