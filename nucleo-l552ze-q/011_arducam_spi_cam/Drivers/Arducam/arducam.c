/**
 * @file arducam.c
 * @brief ArduCAM driver implementation for STM32L5 (Nucleo L552RE-Q)
 *
 * Converted from Raspberry Pi Pico SDK to STM32 HAL
 */

#include "arducam.h"
#include "ov2640_regs.h"
#include "ov5642_regs.h"
#include <string.h>

/*============================================================================
 * Private Defines
 *============================================================================*/

#define ARDUCAM_I2C_TIMEOUT     100     /* I2C timeout in ms */
#define ARDUCAM_SPI_TIMEOUT     100     /* SPI timeout in ms */

/*============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief Delay in milliseconds
 */
static inline void arducam_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/*============================================================================
 * Initialization Functions
 *============================================================================*/

HAL_StatusTypeDef arducam_init(arducam_handle_t *handle, const arducam_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return HAL_ERROR;
    }

    /* Copy configuration */
    handle->hspi = config->hspi;
    handle->hi2c = config->hi2c;
    handle->cs_port = config->cs_port;
    handle->cs_pin = config->cs_pin;
    handle->sensor_model = config->sensor_model;
    handle->image_format = ARDUCAM_FMT_JPEG;  /* Default to JPEG */

    /* Set sensor I2C address based on model */
    switch (handle->sensor_model) {
        case ARDUCAM_OV2640:
            handle->sensor_addr = 0x30;  /* 7-bit address */
            break;
        case ARDUCAM_OV5642:
            handle->sensor_addr = 0x3C;  /* 7-bit address */
            break;
        default:
            handle->sensor_addr = 0x30;  /* Default */
            break;
    }

    /* Set CS high initially */
    arducam_cs_high(handle);

    return HAL_OK;
}

void arducam_set_format(arducam_handle_t *handle, uint8_t format)
{
    if (handle != NULL) {
        handle->image_format = format;
    }
}

HAL_StatusTypeDef arducam_init_cam(arducam_handle_t *handle)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (handle == NULL) {
        return HAL_ERROR;
    }

    switch (handle->sensor_model) {
        case ARDUCAM_OV2640:
            /* Reset sensor */
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x01);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x12, 0x80);
            if (status != HAL_OK) return status;
            arducam_delay_ms(100);

            if (handle->image_format == ARDUCAM_FMT_JPEG) {
                status = arducam_wr_sensor_regs8_8(handle, OV2640_JPEG_INIT);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_regs8_8(handle, OV2640_YUV422);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_regs8_8(handle, OV2640_JPEG);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x01);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_reg8_8(handle, 0x15, 0x00);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_regs8_8(handle, OV2640_320x240_JPEG);
                if (status != HAL_OK) return status;
            } else {
                status = arducam_wr_sensor_regs8_8(handle, OV2640_QVGA);
                if (status != HAL_OK) return status;
            }
            break;

        case ARDUCAM_OV5642:
            /* Reset sensor */
            status = arducam_wr_sensor_reg16_8(handle, 0x3008, 0x80);
            if (status != HAL_OK) return status;
            arducam_delay_ms(100);

            if (handle->image_format == ARDUCAM_FMT_RAW) {
                status = arducam_wr_sensor_regs16_8(handle, OV5642_1280x960_RAW);
                if (status != HAL_OK) return status;
                status = arducam_wr_sensor_regs16_8(handle, OV5642_640x480_RAW);
                if (status != HAL_OK) return status;
            } else {
                status = arducam_wr_sensor_regs16_8(handle, OV5642_QVGA_Preview);
                if (status != HAL_OK) return status;
                arducam_delay_ms(100);

                if (handle->image_format == ARDUCAM_FMT_JPEG) {
                    arducam_delay_ms(100);
                    status = arducam_wr_sensor_regs16_8(handle, OV5642_JPEG_Capture_QSXGA);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_regs16_8(handle, ov5642_320x240);
                    if (status != HAL_OK) return status;
                    arducam_delay_ms(100);

                    status = arducam_wr_sensor_reg16_8(handle, 0x3818, 0xA8);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x3621, 0x10);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x3801, 0xB0);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x4407, 0x04);
                    if (status != HAL_OK) return status;
                } else {
                    /* BMP mode */
                    uint8_t reg_val;
                    status = arducam_wr_sensor_reg16_8(handle, 0x4740, 0x21);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x501e, 0x2a);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x5002, 0xf8);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x501f, 0x01);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x4300, 0x61);
                    if (status != HAL_OK) return status;

                    status = arducam_rd_sensor_reg16_8(handle, 0x3818, &reg_val);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x3818, (reg_val | 0x60) & 0xFF);
                    if (status != HAL_OK) return status;

                    status = arducam_rd_sensor_reg16_8(handle, 0x3621, &reg_val);
                    if (status != HAL_OK) return status;
                    status = arducam_wr_sensor_reg16_8(handle, 0x3621, reg_val & 0xDF);
                    if (status != HAL_OK) return status;
                }
            }
            break;

        default:
            return HAL_ERROR;
    }

    return HAL_OK;
}

/*============================================================================
 * SPI Control Functions
 *============================================================================*/

void arducam_cs_high(arducam_handle_t *handle)
{
    if (handle != NULL) {
        HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
    }
}

void arducam_cs_low(arducam_handle_t *handle)
{
    if (handle != NULL) {
        HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
    }
}

HAL_StatusTypeDef arducam_write_reg(arducam_handle_t *handle, uint8_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t buf[2];

    if (handle == NULL) {
        return HAL_ERROR;
    }

    buf[0] = addr | ARDUCHIP_RWBIT;  /* Set write bit */
    buf[1] = data;

    arducam_cs_low(handle);
    status = HAL_SPI_Transmit(handle->hspi, buf, 2, ARDUCAM_SPI_TIMEOUT);
    arducam_cs_high(handle);

    arducam_delay_ms(1);

    return status;
}

HAL_StatusTypeDef arducam_read_reg(arducam_handle_t *handle, uint8_t addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data;

    if (handle == NULL || data == NULL) {
        return HAL_ERROR;
    }

    tx_data = addr & 0x7F;  /* Clear write bit for read */

    arducam_cs_low(handle);
    status = HAL_SPI_Transmit(handle->hspi, &tx_data, 1, ARDUCAM_SPI_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(handle->hspi, data, 1, ARDUCAM_SPI_TIMEOUT);
    }
    arducam_cs_high(handle);

    return status;
}

HAL_StatusTypeDef arducam_set_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    HAL_StatusTypeDef status;

    status = arducam_read_reg(handle, addr, &temp);
    if (status != HAL_OK) return status;

    return arducam_write_reg(handle, addr, temp | bit);
}

HAL_StatusTypeDef arducam_clear_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit)
{
    uint8_t temp;
    HAL_StatusTypeDef status;

    status = arducam_read_reg(handle, addr, &temp);
    if (status != HAL_OK) return status;

    return arducam_write_reg(handle, addr, temp & (~bit));
}

HAL_StatusTypeDef arducam_get_bit(arducam_handle_t *handle, uint8_t addr, uint8_t bit, uint8_t *status_out)
{
    uint8_t temp;
    HAL_StatusTypeDef status;

    if (status_out == NULL) {
        return HAL_ERROR;
    }

    status = arducam_read_reg(handle, addr, &temp);
    if (status == HAL_OK) {
        *status_out = temp & bit;
    }

    return status;
}

/*============================================================================
 * FIFO Control Functions
 *============================================================================*/

HAL_StatusTypeDef arducam_flush_fifo(arducam_handle_t *handle)
{
    return arducam_write_reg(handle, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

HAL_StatusTypeDef arducam_clear_fifo_flag(arducam_handle_t *handle)
{
    return arducam_write_reg(handle, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

HAL_StatusTypeDef arducam_start_capture(arducam_handle_t *handle)
{
    return arducam_write_reg(handle, ARDUCHIP_FIFO, FIFO_START_MASK);
}

HAL_StatusTypeDef arducam_read_fifo(arducam_handle_t *handle, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = SINGLE_FIFO_READ;

    if (handle == NULL || data == NULL) {
        return HAL_ERROR;
    }

    arducam_cs_low(handle);
    status = HAL_SPI_Transmit(handle->hspi, &tx_data, 1, ARDUCAM_SPI_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(handle->hspi, data, 1, ARDUCAM_SPI_TIMEOUT);
    }
    arducam_cs_high(handle);

    return status;
}

HAL_StatusTypeDef arducam_read_fifo_length(arducam_handle_t *handle, uint32_t *length)
{
    HAL_StatusTypeDef status;
    uint8_t len1, len2, len3;

    if (handle == NULL || length == NULL) {
        return HAL_ERROR;
    }

    status = arducam_read_reg(handle, FIFO_SIZE1, &len1);
    if (status != HAL_OK) return status;

    status = arducam_read_reg(handle, FIFO_SIZE2, &len2);
    if (status != HAL_OK) return status;

    status = arducam_read_reg(handle, FIFO_SIZE3, &len3);
    if (status != HAL_OK) return status;

    len3 &= 0x7F;
    *length = ((uint32_t)len3 << 16) | ((uint32_t)len2 << 8) | len1;
    *length &= 0x07FFFFF;

    return HAL_OK;
}

HAL_StatusTypeDef arducam_set_fifo_burst(arducam_handle_t *handle)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = BURST_FIFO_READ;
    uint8_t dummy;

    if (handle == NULL) {
        return HAL_ERROR;
    }

    status = HAL_SPI_TransmitReceive(handle->hspi, &tx_data, &dummy, 1, ARDUCAM_SPI_TIMEOUT);

    return status;
}

HAL_StatusTypeDef arducam_read_fifo_burst(arducam_handle_t *handle, uint8_t *buffer, uint32_t length)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = BURST_FIFO_READ;

    if (handle == NULL || buffer == NULL || length == 0) {
        return HAL_ERROR;
    }

    arducam_cs_low(handle);

    /* Send burst read command */
    status = HAL_SPI_Transmit(handle->hspi, &tx_data, 1, ARDUCAM_SPI_TIMEOUT);
    if (status != HAL_OK) {
        arducam_cs_high(handle);
        return status;
    }

    /* Read data - for large transfers, may need to split */
    uint32_t remaining = length;
    uint8_t *ptr = buffer;

    while (remaining > 0) {
        uint16_t chunk = (remaining > 65535) ? 65535 : (uint16_t)remaining;
        status = HAL_SPI_Receive(handle->hspi, ptr, chunk, ARDUCAM_SPI_TIMEOUT * 10);
        if (status != HAL_OK) {
            arducam_cs_high(handle);
            return status;
        }
        ptr += chunk;
        remaining -= chunk;
    }

    arducam_cs_high(handle);

    return HAL_OK;
}

/*============================================================================
 * Sensor I2C Communication Functions
 *============================================================================*/

HAL_StatusTypeDef arducam_wr_sensor_reg8_8(arducam_handle_t *handle, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    if (handle == NULL) {
        return HAL_ERROR;
    }

    buf[0] = reg;
    buf[1] = val;

    return HAL_I2C_Master_Transmit(handle->hi2c, handle->sensor_addr << 1, buf, 2, ARDUCAM_I2C_TIMEOUT);
}

HAL_StatusTypeDef arducam_rd_sensor_reg8_8(arducam_handle_t *handle, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || val == NULL) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->sensor_addr << 1, &reg, 1, ARDUCAM_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    return HAL_I2C_Master_Receive(handle->hi2c, handle->sensor_addr << 1, val, 1, ARDUCAM_I2C_TIMEOUT);
}

HAL_StatusTypeDef arducam_wr_sensor_reg16_8(arducam_handle_t *handle, uint16_t reg, uint8_t val)
{
    uint8_t buf[3];

    if (handle == NULL) {
        return HAL_ERROR;
    }

    buf[0] = (reg >> 8) & 0xFF;  /* High byte first */
    buf[1] = reg & 0xFF;
    buf[2] = val;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c, handle->sensor_addr << 1, buf, 3, ARDUCAM_I2C_TIMEOUT);
    arducam_delay_ms(2);

    return status;
}

HAL_StatusTypeDef arducam_rd_sensor_reg16_8(arducam_handle_t *handle, uint16_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;
    uint8_t reg_buf[2];

    if (handle == NULL || val == NULL) {
        return HAL_ERROR;
    }

    reg_buf[0] = (reg >> 8) & 0xFF;
    reg_buf[1] = reg & 0xFF;

    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->sensor_addr << 1, reg_buf, 2, ARDUCAM_I2C_TIMEOUT);
    if (status != HAL_OK) return status;

    return HAL_I2C_Master_Receive(handle->hi2c, handle->sensor_addr << 1, val, 1, ARDUCAM_I2C_TIMEOUT);
}

HAL_StatusTypeDef arducam_wr_sensor_regs8_8(arducam_handle_t *handle, const arducam_sensor_reg_t *reg_list)
{
    HAL_StatusTypeDef status;
    const arducam_sensor_reg_t *next = reg_list;

    if (handle == NULL || reg_list == NULL) {
        return HAL_ERROR;
    }

    while ((next->reg != 0xFF) || (next->val != 0xFF)) {
        status = arducam_wr_sensor_reg8_8(handle, (uint8_t)next->reg, (uint8_t)next->val);
        if (status != HAL_OK) return status;
        next++;
    }

    return HAL_OK;
}

HAL_StatusTypeDef arducam_wr_sensor_regs16_8(arducam_handle_t *handle, const arducam_sensor_reg_t *reg_list)
{
    HAL_StatusTypeDef status;
    const arducam_sensor_reg_t *next = reg_list;

    if (handle == NULL || reg_list == NULL) {
        return HAL_ERROR;
    }

    while ((next->reg != 0xFFFF) || (next->val != 0xFF)) {
        status = arducam_wr_sensor_reg16_8(handle, next->reg, (uint8_t)next->val);
        if (status != HAL_OK) return status;
        next++;
    }

    return HAL_OK;
}

/*============================================================================
 * Utility Functions
 *============================================================================*/

HAL_StatusTypeDef arducam_is_capture_done(arducam_handle_t *handle, bool *done)
{
    uint8_t status_reg;
    HAL_StatusTypeDef status;

    if (handle == NULL || done == NULL) {
        return HAL_ERROR;
    }

    status = arducam_read_reg(handle, ARDUCHIP_TRIG, &status_reg);
    if (status == HAL_OK) {
        *done = (status_reg & CAP_DONE_MASK) != 0;
    }

    return status;
}

HAL_StatusTypeDef arducam_verify_spi(arducam_handle_t *handle)
{
    HAL_StatusTypeDef status;
    uint8_t test_val = 0x55;
    uint8_t read_val;

    if (handle == NULL) {
        return HAL_ERROR;
    }

    /* Write test value */
    status = arducam_write_reg(handle, ARDUCHIP_TEST1, test_val);
    if (status != HAL_OK) return status;

    /* Read back */
    status = arducam_read_reg(handle, ARDUCHIP_TEST1, &read_val);
    if (status != HAL_OK) return status;

    /* Verify */
    if (read_val != test_val) {
        return HAL_ERROR;
    }

    /* Try another value */
    test_val = 0xAA;
    status = arducam_write_reg(handle, ARDUCHIP_TEST1, test_val);
    if (status != HAL_OK) return status;

    status = arducam_read_reg(handle, ARDUCHIP_TEST1, &read_val);
    if (status != HAL_OK) return status;

    return (read_val == test_val) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef arducam_verify_i2c(arducam_handle_t *handle, uint16_t *chip_id)
{
    HAL_StatusTypeDef status;
    uint8_t id_high, id_low;
    uint16_t id;

    if (handle == NULL) {
        return HAL_ERROR;
    }

    switch (handle->sensor_model) {
        case ARDUCAM_OV2640:
            /* Select bank 1 */
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x01);
            if (status != HAL_OK) return status;

            /* Read chip ID */
            status = arducam_rd_sensor_reg8_8(handle, OV2640_CHIPID_HIGH, &id_high);
            if (status != HAL_OK) return status;

            status = arducam_rd_sensor_reg8_8(handle, OV2640_CHIPID_LOW, &id_low);
            if (status != HAL_OK) return status;

            id = ((uint16_t)id_high << 8) | id_low;

            if (chip_id != NULL) {
                *chip_id = id;
            }

            /* OV2640 chip ID should be 0x2640 or 0x2642 */
            if ((id_high == 0x26) && ((id_low == 0x40) || (id_low == 0x42))) {
                return HAL_OK;
            }
            break;

        case ARDUCAM_OV5642:
            /* Read chip ID */
            status = arducam_rd_sensor_reg16_8(handle, OV5642_CHIPID_HIGH, &id_high);
            if (status != HAL_OK) return status;

            status = arducam_rd_sensor_reg16_8(handle, OV5642_CHIPID_LOW, &id_low);
            if (status != HAL_OK) return status;

            id = ((uint16_t)id_high << 8) | id_low;

            if (chip_id != NULL) {
                *chip_id = id;
            }

            /* OV5642 chip ID should be 0x5642 */
            if ((id_high == 0x56) && (id_low == 0x42)) {
                return HAL_OK;
            }
            break;

        default:
            return HAL_ERROR;
    }

    return HAL_ERROR;
}

/*============================================================================
 * OV2640 Specific Functions
 *============================================================================*/

HAL_StatusTypeDef arducam_ov2640_set_jpeg_size(arducam_handle_t *handle, uint8_t size)
{
    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    switch (size) {
        case OV2640_160x120:
            return arducam_wr_sensor_regs8_8(handle, OV2640_160x120_JPEG);
        case OV2640_176x144:
            return arducam_wr_sensor_regs8_8(handle, OV2640_176x144_JPEG);
        case OV2640_320x240:
            return arducam_wr_sensor_regs8_8(handle, OV2640_320x240_JPEG);
        case OV2640_352x288:
            return arducam_wr_sensor_regs8_8(handle, OV2640_352x288_JPEG);
        case OV2640_640x480:
            return arducam_wr_sensor_regs8_8(handle, OV2640_640x480_JPEG);
        case OV2640_800x600:
            return arducam_wr_sensor_regs8_8(handle, OV2640_800x600_JPEG);
        case OV2640_1024x768:
            return arducam_wr_sensor_regs8_8(handle, OV2640_1024x768_JPEG);
        case OV2640_1280x1024:
            return arducam_wr_sensor_regs8_8(handle, OV2640_1280x1024_JPEG);
        case OV2640_1600x1200:
            return arducam_wr_sensor_regs8_8(handle, OV2640_1600x1200_JPEG);
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef arducam_ov2640_set_light_mode(arducam_handle_t *handle, uint8_t mode)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    switch (mode) {
        case LIGHT_MODE_AUTO:
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0xC7, 0x00);

        case LIGHT_MODE_SUNNY:
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xC7, 0x40);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCC, 0x5E);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCD, 0x41);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0xCE, 0x54);

        case LIGHT_MODE_CLOUDY:
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xC7, 0x40);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCC, 0x65);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCD, 0x41);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0xCE, 0x4F);

        case LIGHT_MODE_OFFICE:
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xC7, 0x40);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCC, 0x52);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCD, 0x41);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0xCE, 0x66);

        case LIGHT_MODE_HOME:
            status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xC7, 0x40);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCC, 0x42);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0xCD, 0x3F);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0xCE, 0x71);

        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef arducam_ov2640_set_saturation(arducam_handle_t *handle, uint8_t saturation)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x00);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x02);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x03);
    if (status != HAL_OK) return status;

    switch (saturation) {
        case SATURATION_PLUS_4:
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x68);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0x7D, 0x68);
        case SATURATION_DEFAULT:
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x28);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0x7D, 0x28);
        default:
            return HAL_OK;
    }
}

HAL_StatusTypeDef arducam_ov2640_set_brightness(arducam_handle_t *handle, uint8_t brightness)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x00);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x04);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x09);
    if (status != HAL_OK) return status;

    (void)brightness;  /* TODO: implement brightness levels */
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov2640_set_contrast(arducam_handle_t *handle, uint8_t contrast)
{
    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    (void)contrast;  /* TODO: implement contrast levels */
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov2640_set_special_effects(arducam_handle_t *handle, uint8_t effect)
{
    HAL_StatusTypeDef status;

    if (handle == NULL || handle->sensor_model != ARDUCAM_OV2640) {
        return HAL_ERROR;
    }

    status = arducam_wr_sensor_reg8_8(handle, 0xFF, 0x00);
    if (status != HAL_OK) return status;
    status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x00);
    if (status != HAL_OK) return status;

    switch (effect) {
        case EFFECT_NORMAL:
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x00);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x05);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x80);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0x7D, 0x80);

        case EFFECT_BW:
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x18);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x05);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x80);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0x7D, 0x80);

        case EFFECT_SEPIA:
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x18);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7C, 0x05);
            if (status != HAL_OK) return status;
            status = arducam_wr_sensor_reg8_8(handle, 0x7D, 0x40);
            if (status != HAL_OK) return status;
            return arducam_wr_sensor_reg8_8(handle, 0x7D, 0xA6);

        default:
            return HAL_OK;
    }
}

/*============================================================================
 * OV5642 Specific Functions (Stubs)
 *============================================================================*/

HAL_StatusTypeDef arducam_ov5642_set_jpeg_size(arducam_handle_t *handle, uint8_t size)
{
    if (handle == NULL || handle->sensor_model != ARDUCAM_OV5642) {
        return HAL_ERROR;
    }

    switch (size) {
        case OV5642_320x240:
            return arducam_wr_sensor_regs16_8(handle, ov5642_320x240);
        case OV5642_640x480:
            return arducam_wr_sensor_regs16_8(handle, ov5642_640x480);
        case OV5642_1024x768:
            return arducam_wr_sensor_regs16_8(handle, ov5642_1024x768);
        case OV5642_1280x960:
            return arducam_wr_sensor_regs16_8(handle, ov5642_1280x960);
        case OV5642_1600x1200:
            return arducam_wr_sensor_regs16_8(handle, ov5642_1600x1200);
        case OV5642_2048x1536:
            return arducam_wr_sensor_regs16_8(handle, ov5642_2048x1536);
        case OV5642_2592x1944:
            return arducam_wr_sensor_regs16_8(handle, ov5642_2592x1944);
        default:
        	return arducam_wr_sensor_regs16_8(handle, ov5642_320x240);
    }
}

HAL_StatusTypeDef arducam_ov5642_set_light_mode(arducam_handle_t *handle, uint8_t mode)
{
    (void)handle;
    (void)mode;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_saturation(arducam_handle_t *handle, uint8_t saturation)
{
    (void)handle;
    (void)saturation;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_brightness(arducam_handle_t *handle, uint8_t brightness)
{
    (void)handle;
    (void)brightness;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_contrast(arducam_handle_t *handle, uint8_t contrast)
{
    (void)handle;
    (void)contrast;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_special_effects(arducam_handle_t *handle, uint8_t effect)
{
    (void)handle;
    (void)effect;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_hue(arducam_handle_t *handle, uint8_t degree)
{
    (void)handle;
    (void)degree;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_exposure(arducam_handle_t *handle, uint8_t level)
{
    (void)handle;
    (void)level;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_sharpness(arducam_handle_t *handle, uint8_t sharpness)
{
    (void)handle;
    (void)sharpness;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_mirror_flip(arducam_handle_t *handle, uint8_t mode)
{
    (void)handle;
    (void)mode;
    return HAL_OK;
}

HAL_StatusTypeDef arducam_ov5642_set_compress_quality(arducam_handle_t *handle, uint8_t quality)
{
    if (handle == NULL || handle->sensor_model != ARDUCAM_OV5642) {
        return HAL_ERROR;
    }

    switch (quality) {
        case QUALITY_HIGH:
            return arducam_wr_sensor_reg16_8(handle, 0x4407, 0x02);
        case QUALITY_DEFAULT:
            return arducam_wr_sensor_reg16_8(handle, 0x4407, 0x04);
        case QUALITY_LOW:
            return arducam_wr_sensor_reg16_8(handle, 0x4407, 0x08);
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef arducam_ov5642_test_pattern(arducam_handle_t *handle, uint8_t pattern)
{
    (void)handle;
    (void)pattern;
    return HAL_OK;
}


