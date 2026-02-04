/*
 * LoRa-E5 Driver for STM32L5
 * Ported from Arduino LoRa-E5 library
 */

#include "lora-e5.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *region_str[] = {
    "EU434", "EU868", "US915", "US915HYBRID", "US915OLD",
    "AU915", "AS923", "CN470", "CN779", "KR920",
    "CN470PREQUEL", "STE920", "IN865", "RU864"
};

/* Private function prototypes */
static void LoRa_ClearBuffer(LoRa_Handle_t *hlora);
static void LoRa_SF_BW_ToBitrate(LoRa_Handle_t *hlora, LoRa_SpreadingFactor_t sf, 
                                  LoRa_Bandwidth_t bw);
static uint8_t LoRa_HexCharToValue(char c);

/*============================================================================
 * Initialization Functions
 *============================================================================*/

LoRa_Status_t LoRa_Init(LoRa_Handle_t *hlora, UART_HandleTypeDef *huart)
{
    uint32_t result;
    
    if (hlora == NULL || huart == NULL) {
        return LORA_ERROR;
    }
    
    /* Initialize handle */
    hlora->huart = huart;
    hlora->initialized = false;
    hlora->lowpower_auto = false;
    hlora->adaptive_dr = true;
    hlora->baud_rate = LORA_BR_9600;
    hlora->region = LORA_REGION_UNINIT;
    hlora->sf = LORA_SF_X;
    hlora->bw = LORA_BW_X;
    hlora->tx_power = 14;
    hlora->freq_band = 0;
    hlora->bit_rate = 0;
    hlora->tx_head_time = 0;
    hlora->rx_index = 0;
    hlora->rx_complete = false;
    
    /* Clear buffers */
    LoRa_ClearBuffer(hlora);
    
    /* Start UART reception in interrupt mode */
    HAL_UART_Receive_IT(hlora->huart, &hlora->rx_byte, 1);
    
    /* Wait for module to boot */
    HAL_Delay(2000);
    
    /* Disable low power auto mode first */
    hlora->lowpower_auto = true;
    LoRa_SetLowPowerAutoMode(hlora, false);
    hlora->lowpower_auto = false;
    
    /* Wake up the module */
    LoRa_WakeUp(hlora);
    
    char debug_response[256] = {0};

    /* Test communication with AT command */
    result = LoRa_SendCommand(hlora, "AT\r\n", "+AT: OK", 1000, debug_response);
    printf("LoRa response: [%s], result: %lu, rx_index: %u, callbacks: %lu, errors: %lu\r\n", 
           debug_response, result, hlora->rx_index, LoRa_GetRxCallbackCount(), LoRa_GetRxErrorCount());
    if (result == 0) {
        /* Try different baud rates */
        /* For now, assume 9600 works - can be extended */
        printf("LoRa-E5: Communication test failed\r\n");
        return LORA_ERROR;
    }
    
    hlora->initialized = true;
    printf("LoRa-E5: Initialized successfully\r\n");
    
    return LORA_OK;
}

LoRa_Status_t LoRa_DeInit(LoRa_Handle_t *hlora)
{
    if (hlora == NULL) {
        return LORA_ERROR;
    }
    
    hlora->initialized = false;
    return LORA_OK;
}

/*============================================================================
 * Core AT Command Functions
 *============================================================================*/

static void LoRa_ClearBuffer(LoRa_Handle_t *hlora)
{
    memset(hlora->recv_buf, 0, LORA_BUFFER_LENGTH_MAX);
    memset(hlora->cmd_buf, 0, LORA_CMD_BUFFER_MAX);
    memset(hlora->ack_buf, 0, 64);
    hlora->rx_index = 0;
}

static volatile uint32_t rx_callback_count = 0;
static volatile uint32_t rx_error_count = 0;
static volatile HAL_StatusTypeDef last_rx_status = HAL_OK;

void LoRa_UART_RxCallback(LoRa_Handle_t *hlora)
{
    HAL_StatusTypeDef status;
    
    if (hlora == NULL) return;
    
    rx_callback_count++;
    
    if (hlora->rx_index < LORA_BUFFER_LENGTH_MAX - 1) {
        hlora->recv_buf[hlora->rx_index++] = hlora->rx_byte;
    }
    
    /* Continue receiving */
    status = HAL_UART_Receive_IT(hlora->huart, &hlora->rx_byte, 1);
    last_rx_status = status;
    if (status != HAL_OK) {
        rx_error_count++;
    }
}

void LoRa_UART_ErrorCallback(LoRa_Handle_t *hlora)
{
    if (hlora == NULL) return;
    
    rx_error_count++;

    /* Clear error flags and restart reception */
    __HAL_UART_CLEAR_FLAG(hlora->huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    HAL_UART_Receive_IT(hlora->huart, &hlora->rx_byte, 1);
}

uint32_t LoRa_GetRxCallbackCount(void)
{
    return rx_callback_count;
}

uint32_t LoRa_GetRxErrorCount(void)
{
    return rx_error_count;
}

uint32_t LoRa_SendCommand(LoRa_Handle_t *hlora, const char *cmd, const char *ack, 
                          uint32_t timeout_ms, char *response)
{
    uint32_t start_time;
    uint32_t elapsed = 0;
    int ch;
    
    if (hlora == NULL) return 0;
    
    /* Clear receive buffer */
    memset(hlora->recv_buf, 0, LORA_BUFFER_LENGTH_MAX);
    hlora->rx_index = 0;
    
    /* Flush any pending data */
    while (__HAL_UART_GET_FLAG(hlora->huart, UART_FLAG_RXNE)) {
        ch = hlora->huart->Instance->RDR;
        (void)ch;
    }
    
    /* Send wake-up bytes if in low power auto mode */
    if (hlora->lowpower_auto) {
        uint8_t wake_bytes[] = {0xFF, 0xFF, 0xFF, 0xFF};
        HAL_UART_Transmit(hlora->huart, wake_bytes, 4, 100);
    }
    
    /* Send command if provided */
    if (cmd != NULL) {
        HAL_UART_Transmit(hlora->huart, (uint8_t*)cmd, strlen(cmd), 1000);
        /* Re-arm RX interrupt after blocking transmit */
        HAL_UART_Receive_IT(hlora->huart, &hlora->rx_byte, 1);
    }
    
    /* Check for NO_ACK mode */
    if (ack == NULL) {
        return 0;
    }
    
    if (strcmp(ack, LORA_AT_NO_ACK) == 0) {
        /* Just wait for timeout and return */
        HAL_Delay(timeout_ms);
        if (response != NULL) {
            strcpy(response, hlora->recv_buf);
        }
        return timeout_ms;
    }
    
    /* Wait for response */
    start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        /* Check if ACK received */
        if (strstr(hlora->recv_buf, ack) != NULL) {
            elapsed = HAL_GetTick() - start_time;
            if (response != NULL) {
                strcpy(response, hlora->recv_buf);
            }
            return elapsed > 0 ? elapsed : 1;
        }
        HAL_Delay(1);
    }
    
    /* Timeout - copy whatever we received */
    if (response != NULL) {
        strcpy(response, hlora->recv_buf);
    }
    
    return 0;
}

uint32_t LoRa_ReadBuffer(LoRa_Handle_t *hlora, char *buffer, uint16_t length, 
                         uint32_t timeout_ms)
{
    uint16_t index = 0;
    
    if (hlora == NULL || buffer == NULL) return 0;
    
    memset(buffer, 0, length);
    HAL_Delay(timeout_ms);
    
    /* Copy received data */
    while (index < hlora->rx_index && index < length) {
        buffer[index] = hlora->recv_buf[index];
        index++;
    }
    
    return index;
}

/*============================================================================
 * Device Information Functions
 *============================================================================*/

uint32_t LoRa_GetVersion(LoRa_Handle_t *hlora, char *buffer, uint32_t timeout)
{
    return LoRa_SendCommand(hlora, "AT+VER=?\r\n", LORA_AT_NO_ACK, timeout, buffer);
}

uint32_t LoRa_GetId(LoRa_Handle_t *hlora, char *buffer, LoRa_DeviceID_t id, uint32_t timeout)
{
    const char *cmd;
    
    switch (id) {
        case LORA_ID_DEV_ADDR:
            cmd = "AT+ID=DevAddr\r\n";
            break;
        case LORA_ID_DEV_EUI:
            cmd = "AT+ID=DevEui\r\n";
            break;
        case LORA_ID_APP_EUI:
            cmd = "AT+ID=AppEui\r\n";
            break;
        default:
            return 0;
    }
    
    return LoRa_SendCommand(hlora, cmd, LORA_AT_NO_ACK, timeout, buffer);
}

uint32_t LoRa_SetId(LoRa_Handle_t *hlora, const char *dev_addr, const char *dev_eui, 
                    const char *app_eui)
{
    uint32_t time_cmd = 0;
    
    if (dev_addr != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+ID=DevAddr,\"%s\"\r\n", dev_addr);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    if (dev_eui != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+ID=DevEui,\"%s\"\r\n", dev_eui);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    if (app_eui != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+ID=AppEui,\"%s\"\r\n", app_eui);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    return time_cmd;
}

uint32_t LoRa_SetKey(LoRa_Handle_t *hlora, const char *nwk_skey, const char *app_skey, 
                     const char *app_key)
{
    uint32_t time_cmd = 0;
    
    if (nwk_skey != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+KEY=NWKSKEY,\"%s\"\r\n", nwk_skey);
        snprintf(hlora->ack_buf, 64, "%s", nwk_skey);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    if (app_skey != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+KEY=APPSKEY,\"%s\"\r\n", app_skey);
        snprintf(hlora->ack_buf, 64, "%s", app_skey);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    if (app_key != NULL) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+KEY=APPKEY,\"%s\"\r\n", app_key);
        snprintf(hlora->ack_buf, 64, "%s", app_key);
        time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                     LORA_DEFAULT_TIMEWAIT * 2, NULL);
    }
    
    return time_cmd;
}

/*============================================================================
 * Configuration Functions
 *============================================================================*/

uint32_t LoRa_SetFrequencyBand(LoRa_Handle_t *hlora, LoRa_Region_t region)
{
    uint32_t time_cmd;
    
    if (region <= LORA_REGION_UNINIT || region >= LORA_REGION_UNDEF) {
        return 0;
    }
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+DR=%s\r\n", region_str[region]);
    snprintf(hlora->ack_buf, 64, "%s", region_str[region]);
    
    time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                LORA_DEFAULT_TIMEWAIT * 2, NULL);
    
    if (time_cmd > 0) {
        hlora->region = region;
        
        /* Set frequency band value */
        switch (region) {
            case LORA_REGION_EU434: hlora->freq_band = 434; break;
            case LORA_REGION_EU868: hlora->freq_band = 868; break;
            case LORA_REGION_US915:
            case LORA_REGION_US915HYBRID:
            case LORA_REGION_US915OLD:
            case LORA_REGION_AU915: hlora->freq_band = 915; break;
            case LORA_REGION_AS923: hlora->freq_band = 923; break;
            case LORA_REGION_CN470:
            case LORA_REGION_CN470PREQUEL: hlora->freq_band = 470; break;
            case LORA_REGION_KR920:
            case LORA_REGION_STE920: hlora->freq_band = 920; break;
            default: break;
        }
    }
    
    return time_cmd;
}

uint32_t LoRa_SetDataRate(LoRa_Handle_t *hlora, LoRa_DataRate_t dr, LoRa_Region_t region)
{
    uint32_t time_cmd = 0;
    
    if (dr > LORA_DR15) {
        return 0;
    }
    
    /* Set frequency band first if not set */
    if (hlora->freq_band == 0 && region > LORA_REGION_UNINIT) {
        time_cmd += LoRa_SetFrequencyBand(hlora, region);
    }
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+DR=%d\r\n", dr);
    time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                 LORA_DEFAULT_TIMEWAIT, NULL);
    
    return time_cmd;
}

static void LoRa_SF_BW_ToBitrate(LoRa_Handle_t *hlora, LoRa_SpreadingFactor_t sf, 
                                  LoRa_Bandwidth_t bw)
{
    uint8_t scale = 4;
    
    if (bw == LORA_BW_500) scale = 4;
    else if (bw == LORA_BW_250) scale = 2;
    else if (bw == LORA_BW_125) scale = 1;
    
    switch (sf) {
        case LORA_SF12:
            hlora->bit_rate = 250 * scale;
            hlora->tx_head_time = 1155.1f / scale;
            break;
        case LORA_SF11:
            hlora->bit_rate = 440 * scale;
            hlora->tx_head_time = 577.5f / scale;
            break;
        case LORA_SF10:
            hlora->bit_rate = 980 * scale;
            hlora->tx_head_time = 288.8f / scale;
            break;
        case LORA_SF9:
            hlora->bit_rate = 1760 * scale;
            hlora->tx_head_time = 164.9f / scale;
            break;
        case LORA_SF8:
            hlora->bit_rate = 3125 * scale;
            hlora->tx_head_time = 82.4f / scale;
            break;
        case LORA_SF7:
            hlora->bit_rate = 5470 * scale;
            hlora->tx_head_time = 46.3f / scale;
            break;
        default:
            break;
    }
    
    if (bw == LORA_BW_50KBPS) {
        hlora->bit_rate = 50000;
        hlora->tx_head_time = 0;
    }
}

uint32_t LoRa_SetSpreadFactor(LoRa_Handle_t *hlora, LoRa_SpreadingFactor_t sf, 
                              LoRa_Bandwidth_t bw, LoRa_Region_t region)
{
    LoRa_DataRate_t dr = LORA_DR_NONE;
    uint32_t time_cmd = 0;
    
    /* Set frequency band first if not set */
    if (hlora->freq_band == 0 && region > LORA_REGION_UNINIT) {
        time_cmd += LoRa_SetFrequencyBand(hlora, region);
    }
    
    /* Map SF/BW to DR based on region */
    if (region == LORA_REGION_EU868 || region == LORA_REGION_EU434 ||
        region == LORA_REGION_AS923 || region == LORA_REGION_IN865) {
        if (sf == LORA_SF12 && bw == LORA_BW_125) dr = LORA_DR0;
        else if (sf == LORA_SF11 && bw == LORA_BW_125) dr = LORA_DR1;
        else if (sf == LORA_SF10 && bw == LORA_BW_125) dr = LORA_DR2;
        else if (sf == LORA_SF9 && bw == LORA_BW_125) dr = LORA_DR3;
        else if (sf == LORA_SF8 && bw == LORA_BW_125) dr = LORA_DR4;
        else if (sf == LORA_SF7 && bw == LORA_BW_125) dr = LORA_DR5;
        else if (sf == LORA_SF7 && bw == LORA_BW_250) dr = LORA_DR6;
    } else if (region == LORA_REGION_US915 || region == LORA_REGION_AU915) {
        if (sf == LORA_SF10 && bw == LORA_BW_125) dr = LORA_DR0;
        else if (sf == LORA_SF9 && bw == LORA_BW_125) dr = LORA_DR1;
        else if (sf == LORA_SF8 && bw == LORA_BW_125) dr = LORA_DR2;
        else if (sf == LORA_SF7 && bw == LORA_BW_125) dr = LORA_DR3;
        else if (sf == LORA_SF8 && bw == LORA_BW_500) dr = LORA_DR4;
    }
    
    if (dr != LORA_DR_NONE) {
        time_cmd += LoRa_SetDataRate(hlora, dr, LORA_REGION_UNINIT);
        if (time_cmd > 0) {
            hlora->sf = sf;
            hlora->bw = bw;
            LoRa_SF_BW_ToBitrate(hlora, sf, bw);
        }
    }
    
    return time_cmd;
}

uint32_t LoRa_SetAdaptiveDataRate(LoRa_Handle_t *hlora, bool enable)
{
    uint32_t time_cmd;
    
    if (enable) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+ADR=ON\r\n");
        snprintf(hlora->ack_buf, 64, "+ADR: ON");
    } else {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+ADR=OFF\r\n");
        snprintf(hlora->ack_buf, 64, "+ADR: OFF");
    }
    
    time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                LORA_DEFAULT_TIMEWAIT, NULL);
    
    if (time_cmd > 0) {
        hlora->adaptive_dr = enable;
    }
    
    return time_cmd;
}

uint32_t LoRa_SetPower(LoRa_Handle_t *hlora, int16_t power)
{
    uint32_t time_cmd;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+POWER=%d\r\n", power);
    snprintf(hlora->ack_buf, 64, "+POWER: %d", power);
    
    time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                                LORA_DEFAULT_TIMEWAIT, NULL);
    
    if (time_cmd > 0) {
        hlora->tx_power = power;
    }
    
    return time_cmd;
}

uint32_t LoRa_SetPort(LoRa_Handle_t *hlora, uint8_t port)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+PORT=%d\r\n", port);
    snprintf(hlora->ack_buf, 64, "+PORT: %d", port);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetChannel(LoRa_Handle_t *hlora, uint8_t channel)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+CH=%d\r\n", channel);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetClassType(LoRa_Handle_t *hlora, LoRa_ClassType_t type)
{
    char class_char = (type == LORA_CLASS_C) ? 'C' : 'A';
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+CLASS=%c\r\n", class_char);
    snprintf(hlora->ack_buf, 64, "CLASS: %c", class_char);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetDeviceMode(LoRa_Handle_t *hlora, LoRa_DeviceMode_t mode)
{
    const char *mode_str;
    
    switch (mode) {
        case LORA_MODE_LWABP:
            mode_str = "LWABP";
            break;
        case LORA_MODE_LWOTAA:
            mode_str = "LWOTAA";
            break;
        case LORA_MODE_TEST:
            mode_str = "TEST";
            break;
        default:
            return 0;
    }
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+MODE=%s\r\n", mode_str);
    snprintf(hlora->ack_buf, 64, "+MODE: %s", mode_str);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 1000, NULL);
}

/*============================================================================
 * Join Network Functions
 *============================================================================*/

uint32_t LoRa_JoinOTAA(LoRa_Handle_t *hlora, LoRa_JoinCmd_t cmd, uint32_t timeout)
{
    (void)cmd; /* Both JOIN and FORCE use same command */
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+JOIN\r\n");
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "+JOIN: Network joined", 
                            timeout, NULL);
}

/*============================================================================
 * Data Transfer Functions - LoRaWAN
 *============================================================================*/

uint32_t LoRa_TransferPacket(LoRa_Handle_t *hlora, const char *buffer, uint32_t timeout)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+MSG=\"%s\"\r\n", buffer);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "Done", timeout, NULL);
}

uint32_t LoRa_TransferPacketBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                   uint8_t length, uint32_t timeout)
{
    int pos = 0;
    
    pos = snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+MSGHEX=\"");
    
    for (uint8_t i = 0; i < length && pos < LORA_CMD_BUFFER_MAX - 10; i++) {
        pos += snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "%02X", buffer[i]);
    }
    
    snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "\"\r\n");
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "Done", timeout, NULL);
}

uint32_t LoRa_TransferPacketConfirmed(LoRa_Handle_t *hlora, const char *buffer, 
                                       uint32_t timeout)
{
    uint32_t time_ret;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+CMSG=\"%s\"\r\n", buffer);
    
    time_ret = LoRa_SendCommand(hlora, hlora->cmd_buf, "Wait ACK", timeout, NULL);
    
    /* Wait for RX window */
    HAL_Delay(LORA_RXWIN1_DELAY);
    time_ret += LORA_RXWIN1_DELAY;
    
    /* Wait for Done */
    time_ret += LoRa_SendCommand(hlora, NULL, "Done", timeout, NULL);
    
    return time_ret;
}

uint32_t LoRa_TransferPacketConfirmedBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                            uint8_t length, uint32_t timeout)
{
    uint32_t time_ret;
    int pos = 0;
    
    pos = snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+CMSGHEX=\"");
    
    for (uint8_t i = 0; i < length && pos < LORA_CMD_BUFFER_MAX - 10; i++) {
        pos += snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "%02X", buffer[i]);
    }
    
    snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "\"\r\n");
    
    time_ret = LoRa_SendCommand(hlora, hlora->cmd_buf, "Wait ACK", timeout, NULL);
    
    /* Wait for RX window */
    HAL_Delay(LORA_RXWIN1_DELAY);
    time_ret += LORA_RXWIN1_DELAY;
    
    /* Wait for Done */
    time_ret += LoRa_SendCommand(hlora, NULL, "Done", timeout, NULL);
    
    return time_ret;
}

static uint8_t LoRa_HexCharToValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

int16_t LoRa_ReceivePacket(LoRa_Handle_t *hlora, char *buffer, int16_t length, 
                           int16_t *rssi, uint32_t timeout)
{
    char *ptr;
    int16_t number = 0;
    
    LoRa_ReadBuffer(hlora, hlora->recv_buf, LORA_BUFFER_LENGTH_MAX, timeout);
    
    /* Parse RSSI */
    ptr = strstr(hlora->recv_buf, "RSSI ");
    if (ptr) {
        *rssi = atoi(ptr + 5);
    } else {
        *rssi = -255;
    }
    
    /* Parse RX data */
    ptr = strstr(hlora->recv_buf, "RX: \"");
    if (ptr) {
        ptr += 5;
        for (int16_t i = 0; ; i++) {
            char temp[3] = {0};
            uint8_t result;
            
            temp[0] = *(ptr + i * 3);
            temp[1] = *(ptr + i * 3 + 1);
            
            result = (LoRa_HexCharToValue(temp[0]) << 4) | LoRa_HexCharToValue(temp[1]);
            
            if (i < length) buffer[i] = result;
            
            if (*(ptr + i * 3 + 3) == '\"') {
                number = i + 1;
                break;
            }
        }
    }
    
    return number;
}

/*============================================================================
 * P2P Mode Functions
 *============================================================================*/

uint32_t LoRa_InitP2PMode(LoRa_Handle_t *hlora, uint16_t frequency, 
                          LoRa_SpreadingFactor_t sf, LoRa_Bandwidth_t bw,
                          uint8_t tx_preamble, uint8_t rx_preamble, int16_t power)
{
    uint32_t time_cmd = 0;
    
    /* Set TEST mode */
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+MODE=TEST\r\n");
    time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, "TEST", LORA_DEFAULT_TIMEWAIT, NULL);
    
    /* Configure RF */
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, 
             "AT+TEST=RFCFG,%d,%d,%d,%d,%d,%d\r\n",
             frequency, sf, bw, tx_preamble, rx_preamble, power);
    time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                 LORA_DEFAULT_TIMEWAIT, NULL);
    
    /* Enable RX */
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+TEST=RXLRPKT\r\n");
    time_cmd += LoRa_SendCommand(hlora, hlora->cmd_buf, "RXLRPKT", 
                                 LORA_DEFAULT_TIMEWAIT, NULL);
    
    return time_cmd;
}

uint32_t LoRa_TransferPacketP2P(LoRa_Handle_t *hlora, const char *buffer)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+TEST=TXLRSTR,\"%s\"\r\n", buffer);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "Done", LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_TransferPacketP2PBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                      uint8_t length)
{
    int pos = 0;
    
    pos = snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+TEST=TXLRPKT,\"");
    
    for (uint8_t i = 0; i < length && pos < LORA_CMD_BUFFER_MAX - 10; i++) {
        pos += snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "%02X", buffer[i]);
    }
    
    snprintf(hlora->cmd_buf + pos, LORA_CMD_BUFFER_MAX - pos, "\"\r\n");
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "Done", LORA_DEFAULT_TIMEWAIT, NULL);
}

int16_t LoRa_ReceivePacketP2P(LoRa_Handle_t *hlora, uint8_t *buffer, int16_t length, 
                              int16_t *rssi, uint32_t timeout)
{
    char *ptr;
    int16_t number = 0;
    
    LoRa_ReadBuffer(hlora, hlora->recv_buf, LORA_BUFFER_LENGTH_MAX, timeout);
    
    /* Parse LEN */
    ptr = strstr(hlora->recv_buf, "LEN");
    if (ptr) {
        number = atoi(ptr + 4);
    }
    
    if (number <= 0) return 0;
    
    /* Parse RSSI */
    ptr = strstr(hlora->recv_buf, "RSSI:");
    if (ptr) {
        *rssi = atoi(ptr + 5);
    } else {
        *rssi = -255;
    }
    
    /* Parse RX data */
    ptr = strstr(hlora->recv_buf, "RX \"");
    if (ptr) {
        ptr += 4;
        for (int16_t i = 0; i < number && i < length; i++) {
            char temp[3] = {0};
            
            temp[0] = *(ptr + i * 2);
            temp[1] = *(ptr + i * 2 + 1);
            
            buffer[i] = (LoRa_HexCharToValue(temp[0]) << 4) | LoRa_HexCharToValue(temp[1]);
        }
    }
    
    return number;
}

/*============================================================================
 * Power Management Functions
 *============================================================================*/

uint32_t LoRa_SetLowPower(LoRa_Handle_t *hlora, uint32_t wakeup_time_ms)
{
    if (wakeup_time_ms == 0) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+LOWPOWER\r\n");
    } else {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+LOWPOWER=%lu\r\n", wakeup_time_ms);
    }
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "+LOWPOWER: SLEEP", 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetLowPowerAutoMode(LoRa_Handle_t *hlora, bool enable)
{
    uint32_t time_cmd;
    
    if (enable) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+LOWPOWER=AUTOON\r\n");
        time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, "AUTOON", 
                                    LORA_DEFAULT_TIMEWAIT, NULL);
    } else {
        hlora->lowpower_auto = true; /* Temporarily set to send wake bytes */
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+LOWPOWER=AUTOOFF\r\n");
        time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, "AUTOOFF", 
                                    LORA_DEFAULT_TIMEWAIT, NULL);
        hlora->lowpower_auto = false;
    }
    
    if (time_cmd > 0) {
        hlora->lowpower_auto = enable;
    }
    
    return time_cmd;
}

uint32_t LoRa_WakeUp(LoRa_Handle_t *hlora)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+\r\n");
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "WAKEUP", LORA_DEFAULT_TIMEWAIT, NULL);
}

/*============================================================================
 * Device Control Functions
 *============================================================================*/

uint32_t LoRa_Reset(LoRa_Handle_t *hlora)
{
    uint32_t time_cmd;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+RESET\r\n");
    time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, "+RESET: OK", 
                                LORA_DEFAULT_TIMEWAIT, NULL);
    
    /* Wait for module to restart */
    HAL_Delay(2000);
    
    return time_cmd;
}

uint32_t LoRa_FactoryReset(LoRa_Handle_t *hlora)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+FDEFAULT=RISINGHF\r\n");
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, "+FDEFAULT: OK", 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetDebugLevel(LoRa_Handle_t *hlora, LoRa_DebugLevel_t level)
{
    const char *level_str;
    
    switch (level) {
        case LORA_DEBUG_DEBUG: level_str = "DEBUG"; break;
        case LORA_DEBUG_INFO:  level_str = "INFO"; break;
        case LORA_DEBUG_WARN:  level_str = "WARN"; break;
        case LORA_DEBUG_ERROR: level_str = "ERROR"; break;
        case LORA_DEBUG_FATAL: level_str = "FATAL"; break;
        case LORA_DEBUG_PANIC: level_str = "PANIC"; break;
        case LORA_DEBUG_QUIET: level_str = "QUIET"; break;
        default: return 0;
    }
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+LOG=%s\r\n", level_str);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

/*============================================================================
 * Receive Window Configuration Functions
 *============================================================================*/

uint32_t LoRa_SetReceiveWindowFirst(LoRa_Handle_t *hlora, bool enable)
{
    if (enable) {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+RXWIN1=ON\r\n");
    } else {
        snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+RXWIN1=OFF\r\n");
    }
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetReceiveWindowSecond(LoRa_Handle_t *hlora, float frequency, 
                                      LoRa_DataRate_t dr)
{
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+RXWIN2=%d.%d,%d\r\n",
             (int)frequency, (int)(frequency * 10) % 10, dr);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetReceiveWindowDelay(LoRa_Handle_t *hlora, LoRa_WindowDelay_t type, 
                                     uint16_t delay_ms)
{
    const char *delay_type;
    
    switch (type) {
        case LORA_DELAY_RECEIVE1:      delay_type = "RX1"; break;
        case LORA_DELAY_RECEIVE2:      delay_type = "RX2"; break;
        case LORA_DELAY_JOIN_ACCEPT1:  delay_type = "JRX1"; break;
        case LORA_DELAY_JOIN_ACCEPT2:  delay_type = "JRX2"; break;
        default: return 0;
    }
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+DELAY=%s,%d\r\n", 
             delay_type, delay_ms);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

/*============================================================================
 * Retry Configuration Functions
 *============================================================================*/

uint32_t LoRa_SetUnconfirmedRepeatTime(LoRa_Handle_t *hlora, uint8_t times)
{
    if (times > 15) times = 15;
    if (times == 0) times = 1;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+REPT=%d\r\n", times);
    snprintf(hlora->ack_buf, 64, "+REPT=%d", times);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

uint32_t LoRa_SetConfirmedRetryTime(LoRa_Handle_t *hlora, uint8_t times)
{
    if (times > 15) times = 15;
    if (times == 0) times = 1;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+RETRY=%d\r\n", times);
    snprintf(hlora->ack_buf, 64, "+RETRY=%d", times);
    
    return LoRa_SendCommand(hlora, hlora->cmd_buf, hlora->ack_buf, 
                            LORA_DEFAULT_TIMEWAIT, NULL);
}

/*============================================================================
 * Utility Functions
 *============================================================================*/

uint32_t LoRa_GetBitRate(LoRa_Handle_t *hlora, uint32_t *bit_rate, float *tx_head_time)
{
    uint32_t time_cmd;
    LoRa_SpreadingFactor_t sf = LORA_SF_X;
    LoRa_Bandwidth_t bw = LORA_BW_X;
    
    snprintf(hlora->cmd_buf, LORA_CMD_BUFFER_MAX, "AT+DR\r\n");
    time_cmd = LoRa_SendCommand(hlora, hlora->cmd_buf, LORA_AT_NO_ACK, 
                                LORA_DEFAULT_TIMEWAIT, NULL);
    
    /* Parse response for SF and BW */
    if (strstr(hlora->recv_buf, "SF12") != NULL) sf = LORA_SF12;
    else if (strstr(hlora->recv_buf, "SF11") != NULL) sf = LORA_SF11;
    else if (strstr(hlora->recv_buf, "SF10") != NULL) sf = LORA_SF10;
    else if (strstr(hlora->recv_buf, "SF9") != NULL) sf = LORA_SF9;
    else if (strstr(hlora->recv_buf, "SF8") != NULL) sf = LORA_SF8;
    else if (strstr(hlora->recv_buf, "SF7") != NULL) sf = LORA_SF7;
    
    if (strstr(hlora->recv_buf, "BW500") != NULL) bw = LORA_BW_500;
    else if (strstr(hlora->recv_buf, "BW250") != NULL) bw = LORA_BW_250;
    else if (strstr(hlora->recv_buf, "BW125") != NULL) bw = LORA_BW_125;
    else if (strstr(hlora->recv_buf, "50kbps") != NULL) bw = LORA_BW_50KBPS;
    
    LoRa_SF_BW_ToBitrate(hlora, sf, bw);
    
    if (bit_rate != NULL) *bit_rate = hlora->bit_rate;
    if (tx_head_time != NULL) *tx_head_time = hlora->tx_head_time;
    
    return time_cmd;
}

float LoRa_GetTransmissionTime(LoRa_Handle_t *hlora, uint32_t payload_size)
{
    return hlora->tx_head_time + ((float)(payload_size) * 8.0f * 1000.0f / hlora->bit_rate);
}
