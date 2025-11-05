/*
 * vcm5883l.h
 *
 *  Created on: Nov 5, 2025
 *      Author: fahmad
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* i2c address */
#define VCM5883L_ADDRESS             (0x0C << 1)  // Left shifted for HAL

/* register addresses */
#define VCM5883L_REG_OUT_X_L          0x00
#define VCM5883L_REG_OUT_X_H          0x01
#define VCM5883L_REG_OUT_Y_L          0x02
#define VCM5883L_REG_OUT_Y_H          0x03
#define VCM5883L_REG_OUT_Z_L          0x04
#define VCM5883L_REG_OUT_Z_H          0x05
#define VCM5883L_CTR_REG1             0x0B
#define VCM5883L_CTR_REG2             0x0A



#ifdef __cplusplus
}
#endif
