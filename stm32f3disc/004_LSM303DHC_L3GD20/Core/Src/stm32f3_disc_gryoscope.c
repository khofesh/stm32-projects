/**
  ******************************************************************************
  * @file    stm32f3_discovery_gyroscope.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the l3gd20
  *          MEMS gyroscope available on STM32F3-Discovery Kit.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3_disc_gryoscope.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */

/** @defgroup STM32F3_DISCOVERY_GYROSCOPE STM32F3-DISCOVERY GYROSCOPE
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Private_Types Private Types
  * @{
  */
/**
  * @}
  */

/* Private defines ------------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Private_Constants Private Constants
  * @{
  */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Private_Macros Private Macros
  * @{
  */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Private_Variables Private Variables
  * @{
  */
static GYRO_DrvTypeDef *GyroscopeDrv;

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup STM32F3_DISCOVERY_GYROSCOPE_Private_FunctionPrototypes Private Functions
  * @{
  */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32F3_DISCOVERY_GYROSCOPE_Exported_Functions
  * @{
  */

/**
  * @brief  Set gyroscope Initialization.
  * @retval GYRO_OK if no problem during initialization
  */
uint8_t BSP_GYRO_Init(void)
{
  uint8_t ret = GYRO_ERROR;
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef         Gyro_InitStructure;
  GYRO_FilterConfigTypeDef Gyro_FilterStructure = {0,0};

  if(
		  (L3gd20Drv.ReadID() == I_AM_L3GD20) ||
		  (L3gd20Drv.ReadID() == I_AM_L3GD20_TR) ||
		  (L3gd20Drv.ReadID() == 0xD3)) // works on MY discovery board
  {
    /* Initialize the gyroscope driver structure */
    GyroscopeDrv = &L3gd20Drv;

    /* MEMS configuration ----------------------------------------------------*/
    /* Fill the gyroscope structure */
    Gyro_InitStructure.Power_Mode       = L3GD20_MODE_ACTIVE;
    Gyro_InitStructure.Output_DataRate  = L3GD20_OUTPUT_DATARATE_1;
    Gyro_InitStructure.Axes_Enable      = L3GD20_AXES_ENABLE;
    Gyro_InitStructure.Band_Width       = L3GD20_BANDWIDTH_4;
    Gyro_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    Gyro_InitStructure.Endianness       = L3GD20_BLE_LSB;
    Gyro_InitStructure.Full_Scale       = L3GD20_FULLSCALE_500;

    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl = (uint16_t) (Gyro_InitStructure.Power_Mode  | Gyro_InitStructure.Output_DataRate | \
                       Gyro_InitStructure.Axes_Enable | Gyro_InitStructure.Band_Width);

    ctrl |= (uint16_t) ((Gyro_InitStructure.BlockData_Update | Gyro_InitStructure.Endianness | \
                         Gyro_InitStructure.Full_Scale) << 8);

    /* Initialize the gyroscope */
    GyroscopeDrv->Init(ctrl);

    Gyro_FilterStructure.HighPassFilter_Mode_Selection   = L3GD20_HPM_NORMAL_MODE_RES;
    Gyro_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;

    ctrl = (uint8_t) ((Gyro_FilterStructure.HighPassFilter_Mode_Selection |\
                       Gyro_FilterStructure.HighPassFilter_CutOff_Frequency));

    /* Configure the gyroscope main parameters */
    GyroscopeDrv->FilterConfig(ctrl) ;

    GyroscopeDrv->FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);

    ret = GYRO_OK;
  }


  return ret;
}

/**
  * @brief  Read ID of gyroscope component
  * @retval ID
  */
uint8_t BSP_GYRO_ReadID(void)
{
  uint8_t id = 0x00;

  if(GyroscopeDrv->ReadID != NULL)
  {
    id = GyroscopeDrv->ReadID();
  }
  return id;
}

/**
  * @brief  Reboot memory content of GYROSCOPE
  * @retval None
  */
void BSP_GYRO_Reset(void)
{
  if(GyroscopeDrv->Reset != NULL)
  {
    GyroscopeDrv->Reset();
  }
}


/**
  * @brief  Configure INT1 interrupt
  * @param  pIntConfig pointer to a L3GD20_InterruptConfig_TypeDef
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfig)
{
uint16_t interruptconfig = 0x0000;

  if(GyroscopeDrv->ConfigIT != NULL)
  {
    /* Configure latch Interrupt request and axe interrupts */
    interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| \
                                  pIntConfig->Interrupt_Axes) << 8);

    interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);

	GyroscopeDrv->ConfigIT(interruptconfig);
  }
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntPin Interrupt pin
  *      This parameter can be:
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void BSP_GYRO_EnableIT(uint8_t IntPin)
{
  if(GyroscopeDrv->EnableIT != NULL)
  {
	GyroscopeDrv->EnableIT(IntPin);
  }
}

/**
  * @brief  Disable INT1 or INT2 interrupt
  * @param  IntPin Interrupt pin
  *      This parameter can be:
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void BSP_GYRO_DisableIT(uint8_t IntPin)
{
  if(GyroscopeDrv->DisableIT != NULL)
  {
    GyroscopeDrv->DisableIT(IntPin);
  }
}

/**
  * @brief  Get XYZ angular acceleration
  * @param pfData pointer on floating array
  * @retval None
  */
void BSP_GYRO_GetXYZ(float* pfData)
{
  if(GyroscopeDrv->GetXYZ!= NULL)
  {
	GyroscopeDrv->GetXYZ(pfData);
  }
}

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

