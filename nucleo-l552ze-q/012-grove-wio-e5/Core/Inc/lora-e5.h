/*
 * LoRa-E5 Driver for STM32L5
 * Ported from Arduino LoRa-E5 library
 * https://github.com/idreamsi/LoRaE5
 *
 */

#ifndef LORA_E5_H
#define LORA_E5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define LORA_BUFFER_LENGTH_MAX    512
#define LORA_CMD_BUFFER_MAX       556
#define LORA_DEFAULT_TIMEOUT      3000
#define LORA_DEFAULT_TIMEOUT_ReTx 1000  //milliseconds to max wait for a command to get a response for transferPacketWithConfirmedReTransmission command
#define LORA_DEFAULT_TIMEWAIT     100
#define LORA_kLOCAL_BUFF_MAX  64 // not used anywhere
#define LORA_RXWIN1_DELAY         1000
#define LORA_RXWIN2_DELAY         2000

#define    SLEEPPOWER_mA    0.021 //measured: power consuption when module is feed with 3.3V and is in sleep mode
#define    RXPOWER_mA       5.65 //measured mA when LoRa-E5 module is working as a receptor
#define    TXPOWER_00dBm_mA 41.0  //NOT MEASURED. STIMATED  based on table
#define    TXPOWER_02dBm_mA 41.5 //meassured 41.5 mA at 868 Mhz
#define    TXPOWER_04dBm_mA 48.1 //meassured 48.1 mA at 868 Mhz
#define    TXPOWER_06dBm_mA 53.5 //meassured 53.5 mA at 868 Mhz
#define    TXPOWER_08dBm_mA 60.6 //meassured 60.6 mA at 868 Mhz
#define    TXPOWER_10dBm_mA 68.3 //meassured 68.3 mA at 868 Mhz
#define    TXPOWER_12dBm_mA 77.3 //meassured 77.3 mA at 868 Mhz
#define    TXPOWER_14dBm_mA 86.8 //meassured 86.8 mA at 868 Mhz
#define    TXPOWER_16dBm_mA 86.8 //Module says that 16 dBm were set up properlty, meassured 86.8 mA at 868 Mhz. Not checked if the effective TX

/* AT Command Response Markers */
#define LORA_AT_NO_ACK            "NO_ACK"
#define LORA_MAC_COMMAND_FLAG     "MACCMD:"

typedef enum {
    LORA_ID_DEV_ADDR = 0,
    LORA_ID_DEV_EUI,
    LORA_ID_APP_EUI
} LoRa_DeviceID_t;

typedef enum {
    LORA_BR_9600   = 9600,
    LORA_BR_38400  = 38400,
    LORA_BR_115200 = 115200
} LoRa_BaudRate_t;

typedef enum {
    LORA_DEBUG_DEBUG = 0,
    LORA_DEBUG_INFO,
    LORA_DEBUG_WARN,
    LORA_DEBUG_ERROR,
    LORA_DEBUG_FATAL,
    LORA_DEBUG_PANIC,
    LORA_DEBUG_QUIET
} LoRa_DebugLevel_t;

/* LoRaWAN Class Types */
typedef enum {
    LORA_CLASS_A = 0,
    LORA_CLASS_C
} LoRa_ClassType_t;

/* Physical/Regional Types */
typedef enum {
    LORA_REGION_UNINIT = -1,
    LORA_REGION_EU434 = 0,
    LORA_REGION_EU868,
    LORA_REGION_US915,
    LORA_REGION_US915HYBRID,
    LORA_REGION_US915OLD,
    LORA_REGION_AU915,
    LORA_REGION_AS923,
    LORA_REGION_CN470,
    LORA_REGION_CN779,
    LORA_REGION_KR920,
    LORA_REGION_CN470PREQUEL,
    LORA_REGION_STE920,
    LORA_REGION_IN865,
    LORA_REGION_RU864,
    LORA_REGION_UNDEF
} LoRa_Region_t;

typedef enum {
    LORA_MODE_LWABP = 0,
    LORA_MODE_LWOTAA,
    LORA_MODE_TEST
} LoRa_DeviceMode_t;

/* OTAA Join Commands */
typedef enum {
    LORA_JOIN_JOIN = 0,
    LORA_JOIN_FORCE
} LoRa_JoinCmd_t;

typedef enum {
    LORA_DELAY_RECEIVE1 = 0,
    LORA_DELAY_RECEIVE2,
    LORA_DELAY_JOIN_ACCEPT1,
    LORA_DELAY_JOIN_ACCEPT2
} LoRa_WindowDelay_t;

typedef enum {
    LORA_BW_X = 0,
    LORA_BW_50KBPS = 50,
    LORA_BW_125 = 125,
    LORA_BW_250 = 250,
    LORA_BW_500 = 500
} LoRa_Bandwidth_t;

typedef enum {
    LORA_SF_X = 0,
    LORA_SF7 = 7,
    LORA_SF8 = 8,
    LORA_SF9 = 9,
    LORA_SF10 = 10,
    LORA_SF11 = 11,
    LORA_SF12 = 12
} LoRa_SpreadingFactor_t;

typedef enum {
    LORA_DR0 = 0,
    LORA_DR1,
    LORA_DR2,
    LORA_DR3,
    LORA_DR4,
    LORA_DR5,
    LORA_DR6,
    LORA_DR7,
    LORA_DR8,
    LORA_DR9,
    LORA_DR10,
    LORA_DR11,
    LORA_DR12,
    LORA_DR13,
    LORA_DR14,
    LORA_DR15,
    LORA_DR_NONE
} LoRa_DataRate_t;

/* Return Status */
typedef enum {
    LORA_OK = 0,
    LORA_ERROR,
    LORA_TIMEOUT,
    LORA_BUSY,
    LORA_NOT_JOINED
} LoRa_Status_t;

/* LoRa-E5 Handle Structure */
typedef struct {
    UART_HandleTypeDef *huart;          /* UART handle for LoRa-E5 communication */
    
    /* Internal state */
    bool initialized;
    bool lowpower_auto;
    bool adaptive_dr; // adaptive data rate.
    LoRa_BaudRate_t baud_rate;
    LoRa_Region_t region;
    LoRa_SpreadingFactor_t sf;
    LoRa_Bandwidth_t bw;
    int16_t tx_power;
    float freq_band;
    uint32_t bit_rate;
    float tx_head_time;
    
    /* Buffers */
    char recv_buf[LORA_BUFFER_LENGTH_MAX];
    char cmd_buf[LORA_CMD_BUFFER_MAX];
    char ack_buf[64];
    
    /* RX handling */
    uint8_t rx_byte;
    volatile uint16_t rx_index;
    volatile bool rx_complete;
} LoRa_Handle_t;

/* Initialization */
LoRa_Status_t LoRa_Init(LoRa_Handle_t *hlora, UART_HandleTypeDef *huart);
LoRa_Status_t LoRa_DeInit(LoRa_Handle_t *hlora);

/* Core AT Command Functions */
uint32_t LoRa_SendCommand(LoRa_Handle_t *hlora, const char *cmd, const char *ack, 
                          uint32_t timeout_ms, char *response);
uint32_t LoRa_ReadBuffer(LoRa_Handle_t *hlora, char *buffer, uint16_t length, 
                         uint32_t timeout_ms);

/* Device Information */
uint32_t LoRa_GetVersion(LoRa_Handle_t *hlora, char *buffer, uint32_t timeout);
uint32_t LoRa_GetId(LoRa_Handle_t *hlora, char *buffer, LoRa_DeviceID_t id, uint32_t timeout);
uint32_t LoRa_SetId(LoRa_Handle_t *hlora, const char *dev_addr, const char *dev_eui, 
                    const char *app_eui);
uint32_t LoRa_SetKey(LoRa_Handle_t *hlora, const char *nwk_skey, const char *app_skey, 
                     const char *app_key);

/* Configuration */
uint32_t LoRa_SetFrequencyBand(LoRa_Handle_t *hlora, LoRa_Region_t region);
uint32_t LoRa_SetDataRate(LoRa_Handle_t *hlora, LoRa_DataRate_t dr, LoRa_Region_t region);
uint32_t LoRa_SetSpreadFactor(LoRa_Handle_t *hlora, LoRa_SpreadingFactor_t sf, 
                              LoRa_Bandwidth_t bw, LoRa_Region_t region);
uint32_t LoRa_SetAdaptiveDataRate(LoRa_Handle_t *hlora, bool enable);
uint32_t LoRa_SetPower(LoRa_Handle_t *hlora, int16_t power);
uint32_t LoRa_SetPort(LoRa_Handle_t *hlora, uint8_t port);
uint32_t LoRa_SetChannel(LoRa_Handle_t *hlora, uint8_t channel);
uint32_t LoRa_SetClassType(LoRa_Handle_t *hlora, LoRa_ClassType_t type);
uint32_t LoRa_SetDeviceMode(LoRa_Handle_t *hlora, LoRa_DeviceMode_t mode);

/* Join Network */
uint32_t LoRa_JoinOTAA(LoRa_Handle_t *hlora, LoRa_JoinCmd_t cmd, uint32_t timeout);

/* Data Transfer - LoRaWAN */
uint32_t LoRa_TransferPacket(LoRa_Handle_t *hlora, const char *buffer, uint32_t timeout);
uint32_t LoRa_TransferPacketBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                   uint8_t length, uint32_t timeout);
uint32_t LoRa_TransferPacketConfirmed(LoRa_Handle_t *hlora, const char *buffer, 
                                       uint32_t timeout);
uint32_t LoRa_TransferPacketConfirmedBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                            uint8_t length, uint32_t timeout);
int16_t LoRa_ReceivePacket(LoRa_Handle_t *hlora, char *buffer, int16_t length, 
                           int16_t *rssi, uint32_t timeout);

/* P2P Mode */
uint32_t LoRa_InitP2PMode(LoRa_Handle_t *hlora, uint16_t frequency, 
                          LoRa_SpreadingFactor_t sf, LoRa_Bandwidth_t bw,
                          uint8_t tx_preamble, uint8_t rx_preamble, int16_t power);
uint32_t LoRa_TransferPacketP2P(LoRa_Handle_t *hlora, const char *buffer);
uint32_t LoRa_TransferPacketP2PBytes(LoRa_Handle_t *hlora, const uint8_t *buffer, 
                                      uint8_t length);
int16_t LoRa_ReceivePacketP2P(LoRa_Handle_t *hlora, uint8_t *buffer, int16_t length, 
                              int16_t *rssi, uint32_t timeout);

/* Power Management */
uint32_t LoRa_SetLowPower(LoRa_Handle_t *hlora, uint32_t wakeup_time_ms);
uint32_t LoRa_SetLowPowerAutoMode(LoRa_Handle_t *hlora, bool enable);
uint32_t LoRa_WakeUp(LoRa_Handle_t *hlora);

/* Device Control */
uint32_t LoRa_Reset(LoRa_Handle_t *hlora);
uint32_t LoRa_FactoryReset(LoRa_Handle_t *hlora);
uint32_t LoRa_SetDebugLevel(LoRa_Handle_t *hlora, LoRa_DebugLevel_t level);

/* Receive Window Configuration */
uint32_t LoRa_SetReceiveWindowFirst(LoRa_Handle_t *hlora, bool enable);
uint32_t LoRa_SetReceiveWindowSecond(LoRa_Handle_t *hlora, float frequency, 
                                      LoRa_DataRate_t dr);
uint32_t LoRa_SetReceiveWindowDelay(LoRa_Handle_t *hlora, LoRa_WindowDelay_t type, 
                                     uint16_t delay_ms);

/* Retry Configuration */
uint32_t LoRa_SetUnconfirmedRepeatTime(LoRa_Handle_t *hlora, uint8_t times);
uint32_t LoRa_SetConfirmedRetryTime(LoRa_Handle_t *hlora, uint8_t times);

/* Utility */
uint32_t LoRa_GetBitRate(LoRa_Handle_t *hlora, uint32_t *bit_rate, float *tx_head_time);
float LoRa_GetTransmissionTime(LoRa_Handle_t *hlora, uint32_t payload_size);

/* UART Callback - call from HAL_UART_RxCpltCallback */
void LoRa_UART_RxCallback(LoRa_Handle_t *hlora);

// TODO
/*
	unsigned int getChannel(void);
unsigned int setChannel(unsigned char channel);
	unsigned int setChannel(unsigned char channel, float frequency);
	unsigned int setChannel(unsigned char channel, float frequency,
			_data_rate_t dataRata);
	unsigned int setChannel(unsigned char channel, float frequency,
			_data_rate_t dataRataMin, _data_rate_t dataRataMax);

 */
#ifdef __cplusplus
}
#endif

#endif /* LORA_E5_H */
