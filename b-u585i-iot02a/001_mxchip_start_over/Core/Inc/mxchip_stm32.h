/*
 * mxchip_stm32.h
 *
 *  Created on: Oct 4, 2025
 *      Author: fahmad
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "mx_wifi.h"

/* Network scan result structure */
typedef struct {
    char ssid[33];              /* SSID of the AP (null-terminated) */
    uint8_t bssid[6];           /* BSSID (MAC address) of the AP */
    int32_t rssi;               /* Signal strength in dBm */
    int32_t channel;            /* Channel number */
    uint8_t security;           /* Security type */
    const char* security_str;   /* Human-readable security type */
} MXCHIP_NetworkInfo_t;

/* Network scan results container */
typedef struct {
    MXCHIP_NetworkInfo_t networks[MX_WIFI_MAX_DETECTED_AP];
    uint8_t count;              /* Number of networks found */
    uint32_t scan_time_ms;      /* Time taken for scan in milliseconds */
} MXCHIP_ScanResults_t;

/* Function prototypes */
MX_WIFI_STATUS_T MXCHIP_Init(MX_WIFIObject_t *wifi_obj);
MX_WIFI_STATUS_T MXCHIP_ScanNetworks(MX_WIFIObject_t *wifi_obj, MXCHIP_ScanResults_t *results);
MX_WIFI_STATUS_T MXCHIP_ScanSpecificNetwork(MX_WIFIObject_t *wifi_obj, const char *ssid, MXCHIP_ScanResults_t *results);
void MXCHIP_PrintScanResults(const MXCHIP_ScanResults_t *results);
const char* MXCHIP_GetSecurityString(uint8_t security_type);
void MXCHIP_PrintNetworkInfo(const MXCHIP_NetworkInfo_t *network, uint8_t index);
void MXCHIP_PrintStrongestNetwork(const MXCHIP_ScanResults_t *results);
void MXCHIP_PrintNetworkStatistics(const MXCHIP_ScanResults_t *results);
