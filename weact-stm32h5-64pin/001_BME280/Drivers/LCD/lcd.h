/*
 * lcd.h
 *
 *  Created on: Mar 15, 2025
 *      Author: fahmad
 */

#pragma once

#include "stm32h5xx_hal.h"

// device I2C address
#define LCD_ADDRESS    (0x7c>>1)
#define RGB_ADDRESS    (0xc4>>1)
#define RGB_ADDRESS_V5 (0x30)

// commands
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE    0x08
#define LCD_1LINE    0x00
#define LCD_5x8DOTS  0x04

// rgb color definitions
#define WHITE 0
#define RED 1
#define GREEN 2
#define BLUE 3

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t displayfunction;
    uint8_t displaycontrol;
    uint8_t displaymode;
    uint8_t numlines;
    uint8_t currline;
    uint8_t rgb_chip_addr;
} LCD_HandleTypeDef;

// function prototypes
HAL_StatusTypeDef LCD_Init(LCD_HandleTypeDef* hlcd, I2C_HandleTypeDef* hi2c);

HAL_StatusTypeDef LCD_Clear(LCD_HandleTypeDef* hlcd);
HAL_StatusTypeDef LCD_Home(LCD_HandleTypeDef* hlcd);

HAL_StatusTypeDef LCD_SetCursor(LCD_HandleTypeDef* hlcd, uint8_t col, uint8_t row);
HAL_StatusTypeDef LCD_Display(LCD_HandleTypeDef* hlcd);
HAL_StatusTypeDef LCD_NoDisplay(LCD_HandleTypeDef* hlcd);
HAL_StatusTypeDef LCD_Print(LCD_HandleTypeDef* hlcd, char* str);
HAL_StatusTypeDef LCD_WriteChar(LCD_HandleTypeDef* hlcd, uint8_t data);
HAL_StatusTypeDef LCD_SetRGB(LCD_HandleTypeDef* hlcd, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef LCD_SetColor(LCD_HandleTypeDef* hlcd, uint8_t color);
HAL_StatusTypeDef LCD_BlinkLED(LCD_HandleTypeDef* hlcd);
HAL_StatusTypeDef LCD_NoBlinkLED(LCD_HandleTypeDef* hlcd);
