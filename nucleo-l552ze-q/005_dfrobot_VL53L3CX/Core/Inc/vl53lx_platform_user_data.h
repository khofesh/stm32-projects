
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#include "stm32l5xx_hal.h"

#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53lx_def.h"

#ifdef __cplusplus
extern "C"
{
#endif





typedef struct {

	VL53LX_DevData_t   Data;
	uint8_t   i2c_slave_address;
	uint8_t   comms_type;
	uint16_t  comms_speed_khz;
	uint32_t  new_data_ready_poll_duration_ms;

	I2C_HandleTypeDef *I2cHandle;
	uint8_t   I2cDevAddr;
	int     Present;
	int 	Enabled;
	int LoopState;
	int FirstStreamCountZero;
	int 	Idle;
	int		Ready;
	uint8_t RangeStatus;
	FixPoint1616_t SignalRateRtnMegaCps;
	VL53LX_DeviceState   device_state;  /*!< Device State */

} VL53LX_Dev_t;



typedef VL53LX_Dev_t *VL53LX_DEV;



#define VL53LXDevDataGet(Dev, field) (Dev->Data.field)



#define VL53LXDevDataSet(Dev, field, VL53LX_p_003) ((Dev->Data.field) = (VL53LX_p_003))



#define VL53LXDevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)



#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)


#ifdef __cplusplus
}
#endif

#endif


