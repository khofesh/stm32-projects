/**
 * @file arducam.h
 * @brief ArduCAM driver for STM32L5 (Nucleo L552RE-Q)
 *
 * Converted from Raspberry Pi Pico SDK to STM32 HAL
 *
 * Pin Configuration:
 *   I2C1: SDA=PB9, SCL=PB8
 *   SPI1: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PD14
 */

#ifndef ARDUCAM_H
#define ARDUCAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l5xx_hal.h"

/*============================================================================
 * Sensor Definitions
 *============================================================================*/

/* Image format */
#define ARDUCAM_FMT_BMP     0
#define ARDUCAM_FMT_JPEG    1
#define ARDUCAM_FMT_RAW     2

/* Sensor models */
#define ARDUCAM_OV7670      0
#define ARDUCAM_MT9D111_A   1
#define ARDUCAM_OV7675      2
#define ARDUCAM_OV5642      3
#define ARDUCAM_OV3640      4
#define ARDUCAM_OV2640      5
#define ARDUCAM_OV9655      6
#define ARDUCAM_MT9M112     7
#define ARDUCAM_OV7725      8
#define ARDUCAM_OV7660      9
#define ARDUCAM_MT9M001     10
#define ARDUCAM_OV5640      11
#define ARDUCAM_MT9D111_B   12
#define ARDUCAM_OV9650      13
#define ARDUCAM_MT9V111     14
#define ARDUCAM_MT9T112     15
#define ARDUCAM_MT9D112     16
#define ARDUCAM_MT9V034     17
#define ARDUCAM_MT9M034     18

/*----------------------------------------------------------------------------
 * OV2640 Resolution Settings
 *----------------------------------------------------------------------------*/
#define OV2640_160x120      0
#define OV2640_176x144      1
#define OV2640_320x240      2
#define OV2640_352x288      3
#define OV2640_640x480      4
#define OV2640_800x600      5
#define OV2640_1024x768     6
#define OV2640_1280x1024    7
#define OV2640_1600x1200    8

/*----------------------------------------------------------------------------
 * OV5642 Resolution Settings
 *----------------------------------------------------------------------------*/
#define OV5642_320x240      0
#define OV5642_640x480      1
#define OV5642_1024x768     2
#define OV5642_1280x960     3
#define OV5642_1600x1200    4
#define OV5642_2048x1536    5
#define OV5642_2592x1944    6
#define OV5642_1920x1080    7

/*----------------------------------------------------------------------------
 * Image Settings
 *----------------------------------------------------------------------------*/

/* Light Mode */
#define LIGHT_MODE_AUTO             0
#define LIGHT_MODE_SUNNY            1
#define LIGHT_MODE_CLOUDY           2
#define LIGHT_MODE_OFFICE           3
#define LIGHT_MODE_HOME             4

/* Color Saturation */
#define SATURATION_PLUS_4           0
#define SATURATION_PLUS_3           1
#define SATURATION_PLUS_2           2
#define SATURATION_PLUS_1           3
#define SATURATION_DEFAULT          4
#define SATURATION_MINUS_1          5
#define SATURATION_MINUS_2          6
#define SATURATION_MINUS_3          7
#define SATURATION_MINUS_4          8

/* Brightness */
#define BRIGHTNESS_PLUS_4           0
#define BRIGHTNESS_PLUS_3           1
#define BRIGHTNESS_PLUS_2           2
#define BRIGHTNESS_PLUS_1           3
#define BRIGHTNESS_DEFAULT          4
#define BRIGHTNESS_MINUS_1          5
#define BRIGHTNESS_MINUS_2          6
#define BRIGHTNESS_MINUS_3          7
#define BRIGHTNESS_MINUS_4          8

/* Contrast */
#define CONTRAST_PLUS_4             0
#define CONTRAST_PLUS_3             1
#define CONTRAST_PLUS_2             2
#define CONTRAST_PLUS_1             3
#define CONTRAST_DEFAULT            4
#define CONTRAST_MINUS_1            5
#define CONTRAST_MINUS_2            6
#define CONTRAST_MINUS_3            7
#define CONTRAST_MINUS_4            8

/* Special Effects */
#define EFFECT_ANTIQUE              0
#define EFFECT_BLUISH               1
#define EFFECT_GREENISH             2
#define EFFECT_REDDISH              3
#define EFFECT_BW                   4
#define EFFECT_NEGATIVE             5
#define EFFECT_BW_NEGATIVE          6
#define EFFECT_NORMAL               7
#define EFFECT_SEPIA                8
#define EFFECT_OVEREXPOSURE         9
#define EFFECT_SOLARIZE             10
#define EFFECT_BLUEISH              11
#define EFFECT_YELLOWISH            12

/* Exposure */
#define EXPOSURE_MINUS_17_EV        0
#define EXPOSURE_MINUS_13_EV        1
#define EXPOSURE_MINUS_10_EV        2
#define EXPOSURE_MINUS_07_EV        3
#define EXPOSURE_MINUS_03_EV        4
#define EXPOSURE_DEFAULT            5
#define EXPOSURE_PLUS_03_EV         6
#define EXPOSURE_PLUS_07_EV         7
#define EXPOSURE_PLUS_10_EV         8
#define EXPOSURE_PLUS_13_EV         9
#define EXPOSURE_PLUS_17_EV         10

/* Sharpness */
#define SHARPNESS_AUTO_DEFAULT      0
#define SHARPNESS_AUTO_1            1
#define SHARPNESS_AUTO_2            2
#define SHARPNESS_MANUAL_OFF        3
#define SHARPNESS_MANUAL_1          4
#define SHARPNESS_MANUAL_2          5
#define SHARPNESS_MANUAL_3          6
#define SHARPNESS_MANUAL_4          7
#define SHARPNESS_MANUAL_5          8

/* Mirror/Flip */
#define MIRROR_NONE                 0
#define MIRROR_HORIZONTAL           1
#define FLIP_VERTICAL               2
#define MIRROR_FLIP_BOTH            3

/* JPEG Quality */
#define QUALITY_HIGH                0
#define QUALITY_DEFAULT             1
#define QUALITY_LOW                 2

/* Test Pattern */
#define TEST_COLOR_BAR              0
#define TEST_COLOR_SQUARE           1
#define TEST_BW_SQUARE              2
#define TEST_DLI                    3

/*============================================================================
 * ArduChip Register Definitions
 *============================================================================*/

#define ARDUCHIP_RWBIT              0x80    /* Read/Write bit (bit 7) */

#define ARDUCHIP_TEST1              0x00    /* Test register */
#define ARDUCHIP_FRAMES             0x01    /* Frame control register */

#define ARDUCHIP_TIM                0x03    /* Timing control */
#define VSYNC_LEVEL_MASK            0x02    /* 0=High active, 1=Low active */
#define LCD_BKEN_MASK               0x04    /* 0=Enable, 1=Disable */
#define PCLK_DELAY_MASK             0x08    /* 0=No delay, 1=Delay one PCLK */

#define ARDUCHIP_FIFO               0x04    /* FIFO and I2C control */
#define FIFO_CLEAR_MASK             0x01
#define FIFO_START_MASK             0x02
#define FIFO_RDPTR_RST_MASK         0x10
#define FIFO_WRPTR_RST_MASK         0x20

#define ARDUCHIP_GPIO               0x06    /* GPIO Write Register */
#define GPIO_RESET_MASK             0x01    /* 0=Reset, 1=Normal */
#define GPIO_PWDN_MASK              0x02    /* 0=Normal, 1=Standby */
#define GPIO_PWREN_MASK             0x04    /* 0=LDO disable, 1=LDO enable */

#define BURST_FIFO_READ             0x3C    /* Burst FIFO read */
#define SINGLE_FIFO_READ            0x3D    /* Single FIFO read */

#define ARDUCHIP_REV                0x40    /* ArduChip revision */
#define VER_LOW_MASK                0x3F
#define VER_HIGH_MASK               0xC0

#define ARDUCHIP_TRIG               0x41    /* Trigger source */
#define VSYNC_MASK                  0x01
#define SHUTTER_MASK                0x02
#define CAP_DONE_MASK               0x08

#define FIFO_SIZE1                  0x42    /* FIFO size[7:0] */
#define FIFO_SIZE2                  0x43    /* FIFO size[15:8] */
#define FIFO_SIZE3                  0x44    /* FIFO size[18:16] */

/*============================================================================
 * I2C Configuration
 *============================================================================*/

#define I2C_ADDR_8BIT               0
#define I2C_ADDR_16BIT              1
#define I2C_REG_8BIT                0
#define I2C_REG_16BIT               1
#define I2C_DAT_8BIT                0
#define I2C_DAT_16BIT               1

/* Register termination values */
#define SENSOR_REG_TERM_8BIT        0xFF
#define SENSOR_REG_TERM_16BIT       0xFFFF
#define SENSOR_VAL_TERM_8BIT        0xFF
#define SENSOR_VAL_TERM_16BIT       0xFFFF

/* Maximum FIFO size */
#define MAX_FIFO_SIZE               0x7FFFFF    /* 8MB default */

/*============================================================================
 * Data Structures
 *============================================================================*/

/**
 * @brief Sensor register entry
 */
typedef struct {
    uint16_t reg;
    uint16_t val;
} arducam_sensor_reg_t;

/**
 * @brief ArduCAM handle structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /* SPI handle */
    I2C_HandleTypeDef *hi2c;        /* I2C handle */
    GPIO_TypeDef *cs_port;          /* CS GPIO port */
    uint16_t cs_pin;                /* CS GPIO pin */
    uint8_t sensor_model;           /* Sensor model (OV2640, OV5642, etc.) */
    uint8_t sensor_addr;            /* Sensor I2C address (7-bit) */
    uint8_t image_format;           /* Image format (JPEG, BMP, RAW) */
} arducam_handle_t;

/**
 * @brief ArduCAM initialization configuration
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /* SPI handle */
    I2C_HandleTypeDef *hi2c;        /* I2C handle */
    GPIO_TypeDef *cs_port;          /* CS GPIO port */
    uint16_t cs_pin;                /* CS GPIO pin */
    uint8_t sensor_model;           /* Sensor model */
} arducam_config_t;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Initialization
 *----------------------------------------------------------------------------*/

/**
 * @brief Initialize ArduCAM driver
 * @param handle Pointer to ArduCAM handle
 * @param config Pointer to configuration structure
 * @return HAL status
 */
HAL_StatusTypeDef arducam_init(arducam_handle_t *handle, const arducam_config_t *config);

/**
 * @brief Initialize camera sensor
 * @param handle Pointer to ArduCAM handle
 * @return HAL status
 */
HAL_StatusTypeDef arducam_init_cam(arducam_handle_t *handle);

/**
 * @brief Set image format
 * @param handle Pointer to ArduCAM handle
 * @param format Image format (ARDUCAM_FMT_JPEG, ARDUCAM_FMT_BMP, ARDUCAM_FMT_RAW)
 */
void arducam_set_format(arducam_handle_t *handle, uint8_t format);

/*----------------------------------------------------------------------------
 * SPI Control
 *----------------------------------------------------------------------------*/

/**
 * @brief Set CS pin high
 * @param handle Pointer to ArduCAM handle
 */
void arducam_cs_high(arducam_handle_t *handle);

/**
 * @brief Set CS pin low
 * @param handle Pointer to ArduCAM handle
 */
void arducam_cs_low(arducam_handle_t *handle);

/**
 * @brief Write to ArduChip register
 * @param handle Pointer to ArduCAM handle
 * @param addr Register address
 * @param data Data to write
 * @return HAL status
 */
HAL_StatusTypeDef arducam_write_reg(arducam_handle_t *handle, uint8_t addr, uint8_t data);

/**
 * @brief Read from ArduChip register
 * @param handle Pointer to ArduCAM handle
 * @param addr Register address
 * @param data Pointer to store read data
 * @return HAL status
 */
HAL_StatusTypeDef arducam_read_reg(arducam_handle_t *handle, uint8_t addr, uint8_t *data);

/**
 * @brief Set register bit
 * @param handle Pointer to ArduCAM handle
 * @param addr Register address
 * @param bit Bit to set
 * @return HAL status
 */
HAL_StatusTypeDef arducam_set_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit);

/**
 * @brief Clear register bit
 * @param handle Pointer to ArduCAM handle
 * @param addr Register address
 * @param bit Bit to clear
 * @return HAL status
 */
HAL_StatusTypeDef arducam_clear_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit);

/**
 * @brief Get register bit status
 * @param handle Pointer to ArduCAM handle
 * @param addr Register address
 * @param bit Bit to check
 * @param status Pointer to store bit status
 * @return HAL status
 */
HAL_StatusTypeDef arducam_get_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit, uint8_t *status);

/*----------------------------------------------------------------------------
 * FIFO Control
 *----------------------------------------------------------------------------*/

/**
 * @brief Flush FIFO
 * @param handle Pointer to ArduCAM handle
 * @return HAL status
 */
HAL_StatusTypeDef arducam_flush_fifo(arducam_handle_t *handle);

/**
 * @brief Clear FIFO flag
 * @param handle Pointer to ArduCAM handle
 * @return HAL status
 */
HAL_StatusTypeDef arducam_clear_fifo_flag(arducam_handle_t *handle);

/**
 * @brief Start capture
 * @param handle Pointer to ArduCAM handle
 * @return HAL status
 */
HAL_StatusTypeDef arducam_start_capture(arducam_handle_t *handle);

/**
 * @brief Read single byte from FIFO
 * @param handle Pointer to ArduCAM handle
 * @param data Pointer to store read data
 * @return HAL status
 */
HAL_StatusTypeDef arducam_read_fifo(arducam_handle_t *handle, uint8_t *data);

/**
 * @brief Get FIFO length
 * @param handle Pointer to ArduCAM handle
 * @param length Pointer to store FIFO length
 * @return HAL status
 */
HAL_StatusTypeDef arducam_read_fifo_length(arducam_handle_t *handle, uint32_t *length);

/**
 * @brief Set FIFO burst mode
 * @param handle Pointer to ArduCAM handle
 * @return HAL status
 */
HAL_StatusTypeDef arducam_set_fifo_burst(arducam_handle_t *handle);

/**
 * @brief Read FIFO in burst mode
 * @param handle Pointer to ArduCAM handle
 * @param buffer Buffer to store data
 * @param length Number of bytes to read
 * @return HAL status
 */
HAL_StatusTypeDef arducam_read_fifo_burst(arducam_handle_t *handle, uint8_t *buffer, uint32_t length);

/*----------------------------------------------------------------------------
 * Sensor I2C Communication
 *----------------------------------------------------------------------------*/

/**
 * @brief Write 8-bit value to 8-bit register
 * @param handle Pointer to ArduCAM handle
 * @param reg Register address
 * @param val Value to write
 * @return HAL status
 */
HAL_StatusTypeDef arducam_wr_sensor_reg8_8(arducam_handle_t *handle, uint8_t reg, uint8_t val);

/**
 * @brief Read 8-bit value from 8-bit register
 * @param handle Pointer to ArduCAM handle
 * @param reg Register address
 * @param val Pointer to store read value
 * @return HAL status
 */
HAL_StatusTypeDef arducam_rd_sensor_reg8_8(arducam_handle_t *handle, uint8_t reg, uint8_t *val);

/**
 * @brief Write 8-bit value to 16-bit register
 * @param handle Pointer to ArduCAM handle
 * @param reg Register address
 * @param val Value to write
 * @return HAL status
 */
HAL_StatusTypeDef arducam_wr_sensor_reg16_8(arducam_handle_t *handle, uint16_t reg, uint8_t val);

/**
 * @brief Read 8-bit value from 16-bit register
 * @param handle Pointer to ArduCAM handle
 * @param reg Register address
 * @param val Pointer to store read value
 * @return HAL status
 */
HAL_StatusTypeDef arducam_rd_sensor_reg16_8(arducam_handle_t *handle, uint16_t reg, uint8_t *val);

/**
 * @brief Write register array (8-bit reg, 8-bit val)
 * @param handle Pointer to ArduCAM handle
 * @param reg_list Register array (terminated by 0xFF, 0xFF)
 * @return HAL status
 */
HAL_StatusTypeDef arducam_wr_sensor_regs8_8(arducam_handle_t *handle, const arducam_sensor_reg_t *reg_list);

/**
 * @brief Write register array (16-bit reg, 8-bit val)
 * @param handle Pointer to ArduCAM handle
 * @param reg_list Register array (terminated by 0xFFFF, 0xFF)
 * @return HAL status
 */
HAL_StatusTypeDef arducam_wr_sensor_regs16_8(arducam_handle_t *handle, const arducam_sensor_reg_t *reg_list);

/*----------------------------------------------------------------------------
 * OV2640 Specific Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Set OV2640 JPEG resolution
 * @param handle Pointer to ArduCAM handle
 * @param size Resolution (OV2640_160x120, OV2640_320x240, etc.)
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_jpeg_size(arducam_handle_t *handle, uint8_t size);

/**
 * @brief Set OV2640 light mode
 * @param handle Pointer to ArduCAM handle
 * @param mode Light mode
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_light_mode(arducam_handle_t *handle, uint8_t mode);

/**
 * @brief Set OV2640 color saturation
 * @param handle Pointer to ArduCAM handle
 * @param saturation Saturation level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_saturation(arducam_handle_t *handle, uint8_t saturation);

/**
 * @brief Set OV2640 brightness
 * @param handle Pointer to ArduCAM handle
 * @param brightness Brightness level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_brightness(arducam_handle_t *handle, uint8_t brightness);

/**
 * @brief Set OV2640 contrast
 * @param handle Pointer to ArduCAM handle
 * @param contrast Contrast level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_contrast(arducam_handle_t *handle, uint8_t contrast);

/**
 * @brief Set OV2640 special effects
 * @param handle Pointer to ArduCAM handle
 * @param effect Effect type
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov2640_set_special_effects(arducam_handle_t *handle, uint8_t effect);

/*----------------------------------------------------------------------------
 * OV5642 Specific Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Set OV5642 JPEG resolution
 * @param handle Pointer to ArduCAM handle
 * @param size Resolution
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_jpeg_size(arducam_handle_t *handle, uint8_t size);

/**
 * @brief Set OV5642 light mode
 * @param handle Pointer to ArduCAM handle
 * @param mode Light mode
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_light_mode(arducam_handle_t *handle, uint8_t mode);

/**
 * @brief Set OV5642 color saturation
 * @param handle Pointer to ArduCAM handle
 * @param saturation Saturation level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_saturation(arducam_handle_t *handle, uint8_t saturation);

/**
 * @brief Set OV5642 brightness
 * @param handle Pointer to ArduCAM handle
 * @param brightness Brightness level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_brightness(arducam_handle_t *handle, uint8_t brightness);

/**
 * @brief Set OV5642 contrast
 * @param handle Pointer to ArduCAM handle
 * @param contrast Contrast level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_contrast(arducam_handle_t *handle, uint8_t contrast);

/**
 * @brief Set OV5642 special effects
 * @param handle Pointer to ArduCAM handle
 * @param effect Effect type
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_special_effects(arducam_handle_t *handle, uint8_t effect);

/**
 * @brief Set OV5642 hue
 * @param handle Pointer to ArduCAM handle
 * @param degree Hue degree
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_hue(arducam_handle_t *handle, uint8_t degree);

/**
 * @brief Set OV5642 exposure
 * @param handle Pointer to ArduCAM handle
 * @param level Exposure level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_exposure(arducam_handle_t *handle, uint8_t level);

/**
 * @brief Set OV5642 sharpness
 * @param handle Pointer to ArduCAM handle
 * @param sharpness Sharpness level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_sharpness(arducam_handle_t *handle, uint8_t sharpness);

/**
 * @brief Set OV5642 mirror/flip
 * @param handle Pointer to ArduCAM handle
 * @param mode Mirror/flip mode
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_mirror_flip(arducam_handle_t *handle, uint8_t mode);

/**
 * @brief Set OV5642 compress quality
 * @param handle Pointer to ArduCAM handle
 * @param quality Quality level
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_set_compress_quality(arducam_handle_t *handle, uint8_t quality);

/**
 * @brief Set OV5642 test pattern
 * @param handle Pointer to ArduCAM handle
 * @param pattern Test pattern
 * @return HAL status
 */
HAL_StatusTypeDef arducam_ov5642_test_pattern(arducam_handle_t *handle, uint8_t pattern);

/*----------------------------------------------------------------------------
 * Utility Functions
 *----------------------------------------------------------------------------*/

/**
 * @brief Check if capture is done
 * @param handle Pointer to ArduCAM handle
 * @param done Pointer to store capture done status
 * @return HAL status
 */
HAL_StatusTypeDef arducam_is_capture_done(arducam_handle_t *handle, bool *done);

/**
 * @brief Verify SPI communication (test register read/write)
 * @param handle Pointer to ArduCAM handle
 * @return HAL_OK if communication successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef arducam_verify_spi(arducam_handle_t *handle);

/**
 * @brief Verify I2C communication (read chip ID)
 * @param handle Pointer to ArduCAM handle
 * @param chip_id Pointer to store chip ID (optional, can be NULL)
 * @return HAL_OK if communication successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef arducam_verify_i2c(arducam_handle_t *handle, uint16_t *chip_id);

#ifdef __cplusplus
}
#endif

#endif /* ARDUCAM_H */
