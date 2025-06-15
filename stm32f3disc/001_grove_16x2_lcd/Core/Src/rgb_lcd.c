/*
 * rgb_lcd.c
 *
 *  Created on: Jan 26, 2025
 *      Author: fahmad
 */

#include <string.h>
#include <stdio.h>

#include "rgb_lcd.h"

static HAL_StatusTypeDef LCD_Command(LCD_HandleTypeDef *hlcd, uint8_t cmd)
{
	uint8_t data[2] = {0x80, cmd};
	return HAL_I2C_Master_Transmit(hlcd->hi2c, LCD_ADDRESS << 1, data, 2, 100);
}

static HAL_StatusTypeDef LCD_Write(LCD_HandleTypeDef* hlcd, uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    return HAL_I2C_Master_Transmit(hlcd->hi2c, LCD_ADDRESS << 1, buf, 2, 100);
}

static HAL_StatusTypeDef LCD_SetReg(LCD_HandleTypeDef* hlcd, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(hlcd->hi2c, hlcd->rgb_chip_addr << 1, buf, 2, 100);
}

HAL_StatusTypeDef LCD_Init(LCD_HandleTypeDef* hlcd, I2C_HandleTypeDef* hi2c) {
    HAL_StatusTypeDef status;

    hlcd->hi2c = hi2c;
    hlcd->displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    hlcd->displaymode = 0;
    hlcd->numlines = 2;
    hlcd->currline = 0;
    hlcd->rgb_chip_addr = 0;

    // Wait for more than 40ms after power rises above 2.7V
    HAL_Delay(50);

    // Now we pull both RS and R/W low to begin commands
    status = LCD_Command(hlcd, LCD_FUNCTIONSET | hlcd->displayfunction);
    if(status != HAL_OK) return status;
    HAL_Delay(5);  // Wait more than 4.1ms

    // Second try
    status = LCD_Command(hlcd, LCD_FUNCTIONSET | hlcd->displayfunction);
    if(status != HAL_OK) return status;
    HAL_Delay(1);

    // Third try
    status = LCD_Command(hlcd, LCD_FUNCTIONSET | hlcd->displayfunction);
    if(status != HAL_OK) return status;

    // Finally, set # lines, font size, etc.
    status = LCD_Command(hlcd, LCD_FUNCTIONSET | hlcd->displayfunction);
    if(status != HAL_OK) return status;

    // Turn the display on with no cursor or blinking default
    hlcd->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    status = LCD_Display(hlcd);
    if(status != HAL_OK) return status;

    // Clear display
    status = LCD_Clear(hlcd);
    if(status != HAL_OK) return status;

    // Initialize to default text direction
    hlcd->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    status = LCD_Command(hlcd, LCD_ENTRYMODESET | hlcd->displaymode);
    if(status != HAL_OK) return status;

    // Initialize RGB controller
    uint8_t rgbData = 0;
    status = HAL_I2C_Master_Transmit(hlcd->hi2c, RGB_ADDRESS_V5 << 1, &rgbData, 1, 100);
    if(status == HAL_OK) {
        hlcd->rgb_chip_addr = RGB_ADDRESS_V5;
        LCD_SetReg(hlcd, 0x00, 0x07); // Reset the chip
        HAL_Delay(1);
        LCD_SetReg(hlcd, 0x04, 0x15); // Set all LED always on
    } else {
    	status = HAL_I2C_Master_Transmit(hlcd->hi2c, RGB_ADDRESS << 1, &rgbData, 1, 100);
    	if(status == HAL_OK) {
    		hlcd->rgb_chip_addr = RGB_ADDRESS;
    		LCD_SetReg(hlcd, 0x00, 0x00); // Normal mode
    		LCD_SetReg(hlcd, 0x08, 0xFF); // Set LEDs controllable by both PWM and GRPPWM registers
    		LCD_SetReg(hlcd, 0x01, 0x20); // Set blinking mode
    	} else {
            // don't return error - LCD can still work without backlight
            hlcd->rgb_chip_addr = 0;
        }
    }

    // only try to set color if we found an RGB controller
    if (hlcd->rgb_chip_addr != 0) {
        status = LCD_SetColor(hlcd, WHITE);
        if(status != HAL_OK) {
            printf("Failed to set RGB color\r\n");
            // don't return error - LCD can still work
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef LCD_Clear(LCD_HandleTypeDef* hlcd) {
    HAL_StatusTypeDef status = LCD_Command(hlcd, LCD_CLEARDISPLAY);
    HAL_Delay(2); // This command takes a long time
    return status;
}

HAL_StatusTypeDef LCD_Home(LCD_HandleTypeDef* hlcd) {
    HAL_StatusTypeDef status = LCD_Command(hlcd, LCD_RETURNHOME);
    HAL_Delay(2); // This command takes a long time
    return status;
}

HAL_StatusTypeDef LCD_SetCursor(LCD_HandleTypeDef* hlcd, uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x80, 0xC0, 0x94, 0xD4};
    if (row >= hlcd->numlines) {
        row = hlcd->numlines - 1;
    }
    return LCD_Command(hlcd, row_offsets[row] + col);
}

HAL_StatusTypeDef LCD_Display(LCD_HandleTypeDef* hlcd) {
    hlcd->displaycontrol |= LCD_DISPLAYON;
    return LCD_Command(hlcd, LCD_DISPLAYCONTROL | hlcd->displaycontrol);
}

HAL_StatusTypeDef LCD_NoDisplay(LCD_HandleTypeDef* hlcd) {
    hlcd->displaycontrol &= ~LCD_DISPLAYON;
    return LCD_Command(hlcd, LCD_DISPLAYCONTROL | hlcd->displaycontrol);
}

HAL_StatusTypeDef LCD_Print(LCD_HandleTypeDef* hlcd, char* str) {
    HAL_StatusTypeDef status = HAL_OK;
    while (*str && status == HAL_OK) {
        status = LCD_WriteChar(hlcd, *str++);
    }
    return status;
}

HAL_StatusTypeDef LCD_WriteChar(LCD_HandleTypeDef* hlcd, uint8_t data) {
    return LCD_Write(hlcd, data);
}

HAL_StatusTypeDef LCD_SetRGB(LCD_HandleTypeDef* hlcd, uint8_t r, uint8_t g, uint8_t b) {
    if (hlcd->rgb_chip_addr == 0) {
        return HAL_OK; // silently ignore if no RGB controller
    }

    HAL_StatusTypeDef status;

    if (hlcd->rgb_chip_addr == RGB_ADDRESS_V5) {
        status = LCD_SetReg(hlcd, 0x06, r);
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x07, g);
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x08, b);
    } else {
        status = LCD_SetReg(hlcd, 0x04, r);
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x03, g);
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x02, b);
    }

    return status;
}

HAL_StatusTypeDef LCD_SetColor(LCD_HandleTypeDef* hlcd, uint8_t color) {
    const uint8_t colors[][3] = {
        {255, 255, 255},  // WHITE
        {255, 0, 0},      // RED
        {0, 255, 0},      // GREEN
        {0, 0, 255}       // BLUE
    };

    if (color > 3) return HAL_ERROR;

    return LCD_SetRGB(hlcd, colors[color][0], colors[color][1], colors[color][2]);
}

HAL_StatusTypeDef LCD_BlinkLED(LCD_HandleTypeDef* hlcd) {
    HAL_StatusTypeDef status;

    if (hlcd->rgb_chip_addr == RGB_ADDRESS_V5) {
        status = LCD_SetReg(hlcd, 0x04, 0x2A);  // Attach all LED to PWM1
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x01, 0x06);  // Blink every second
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x02, 0x7F);  // Half on, half off
    } else {
        status = LCD_SetReg(hlcd, 0x07, 0x17);  // Blink every second
        if(status != HAL_OK) return status;
        status = LCD_SetReg(hlcd, 0x06, 0x7F);  // Half on, half off
    }

    return status;
}

HAL_StatusTypeDef LCD_NoBlinkLED(LCD_HandleTypeDef* hlcd) {
    if (hlcd->rgb_chip_addr == RGB_ADDRESS_V5) {
        return LCD_SetReg(hlcd, 0x04, 0x15);
    } else {
        HAL_StatusTypeDef status = LCD_SetReg(hlcd, 0x07, 0x00);
        if(status != HAL_OK) return status;
        return LCD_SetReg(hlcd, 0x06, 0xFF);
    }
}
