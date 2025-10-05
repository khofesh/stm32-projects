/*
 * mx_wifi_stm32.c
 *
 *  Created on: Oct 5, 2025
 *      Author: fahmad
 */

#include "mx_wifi_stm32.h"
#include "mx_wifi_io.h"

/* External UART handle for debug output */
extern UART_HandleTypeDef huart1;
/* External SPI handle for MXCHIP communication */
extern SPI_HandleTypeDef hspi2;

/* Private function prototypes */
static void MXCHIP_Reset(void);

/**
 * @brief Initialize the MXCHIP WiFi module
 * @param wifi_obj: Pointer to WiFi object
 * @retval MX_WIFI_STATUS_T: Status of initialization
 */
MX_WIFI_STATUS_T MXCHIP_Init(MX_WIFIObject_t *wifi_obj)
{
    MX_WIFI_STATUS_T status;

    printf("\r\n=== MXCHIP WiFi Module Initialization ===\r\n");

    /* First, probe and register the IO functions */
    printf("Probing MXCHIP IO interface...\r\n");
    void *ll_drv_context = NULL;
    if (mxwifi_probe(&ll_drv_context) != 0) {
        printf("Failed to probe MXCHIP IO interface\r\n");
        return MX_WIFI_STATUS_ERROR;
    }
    
    /* Get the WiFi object with registered IO functions */
    wifi_obj = wifi_obj_get();
    if (wifi_obj == NULL) {
        printf("Failed to get WiFi object\r\n");
        return MX_WIFI_STATUS_ERROR;
    }

    /* Reset the MXCHIP module */
//    printf("Resetting MXCHIP module...\r\n");
//    MXCHIP_Reset();

    /* Initialize the WiFi module */
    printf("Initializing WiFi module...\r\n");
    status = MX_WIFI_Init(wifi_obj);

    if (status == MX_WIFI_STATUS_OK) {
        printf("MXCHIP WiFi module initialized successfully\r\n");

        /* Get module information */
        char version[64];
        if (MX_WIFI_GetVersion(wifi_obj, (uint8_t*)version, sizeof(version)) == MX_WIFI_STATUS_OK) {
            printf("Module version: %s\r\n", version);
        }

        char mac[18];
        uint8_t mac_addr[6];
        if (MX_WIFI_GetMACAddress(wifi_obj, mac_addr) == MX_WIFI_STATUS_OK) {
            snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                    mac_addr[0], mac_addr[1], mac_addr[2],
                    mac_addr[3], mac_addr[4], mac_addr[5]);
            printf("MAC Address: %s\r\n", mac);
        }
    } else {
        printf("Failed to initialize MXCHIP WiFi module (error: %d)\r\n", status);
    }

    printf("=== Initialization Complete ===\r\n\r\n");
    return status;
}

/**
 * @brief Scan for available WiFi networks
 * @param wifi_obj: Pointer to WiFi object
 * @param results: Pointer to scan results structure
 * @retval MX_WIFI_STATUS_T: Status of scan operation
 */
MX_WIFI_STATUS_T MXCHIP_ScanNetworks(MX_WIFIObject_t *wifi_obj, MXCHIP_ScanResults_t *results)
{
    MX_WIFI_STATUS_T status;
    uint32_t start_time;
    int8_t ap_count;
    mwifi_ap_info_t mx_aps[MX_WIFI_MAX_DETECTED_AP];

    if (!wifi_obj || !results) {
        return MX_WIFI_STATUS_PARAM_ERROR;
    }

    /* Initialize results structure */
    memset(results, 0, sizeof(MXCHIP_ScanResults_t));
    memset(mx_aps, 0, sizeof(mx_aps));

    printf("🔍 Starting WiFi network scan...\r\n");
    start_time = HAL_GetTick();

    /* Start passive scan */
    status = MX_WIFI_Scan(wifi_obj, MC_SCAN_PASSIVE, NULL, 0);

    if (status != MX_WIFI_STATUS_OK) {
        printf("Failed to start WiFi scan (error: %d)\r\n", status);
        return status;
    }

    /* Wait for scan to complete and get results */
    printf("Scanning in progress...\r\n");
    HAL_Delay(3000); /* Wait for scan to complete */

    /* Get scan results */
    ap_count = MX_WIFI_Get_scan_result(wifi_obj, (uint8_t*)mx_aps, MX_WIFI_MAX_DETECTED_AP);

    if (ap_count < 0) {
        printf("Failed to get scan results (error: %d)\r\n", ap_count);
        return MX_WIFI_STATUS_ERROR;
    }

    /* Record scan time */
    results->scan_time_ms = HAL_GetTick() - start_time;
    results->count = (uint8_t)ap_count;

    printf("Scan completed in %lu ms\r\n", results->scan_time_ms);
    printf("Found %d networks\r\n\r\n", results->count);

    /* Convert results to our format */
    for (int i = 0; i < ap_count && i < MX_WIFI_MAX_DETECTED_AP; i++) {
        MXCHIP_NetworkInfo_t *network = &results->networks[i];

        /* Copy SSID (ensure null termination) */
        strncpy(network->ssid, mx_aps[i].ssid, sizeof(network->ssid) - 1);
        network->ssid[sizeof(network->ssid) - 1] = '\0';

        /* Copy BSSID */
        memcpy(network->bssid, mx_aps[i].bssid, 6);

        /* Copy other parameters */
        network->rssi = mx_aps[i].rssi;
        network->channel = mx_aps[i].channel;
        network->security = mx_aps[i].security;
        network->security_str = MXCHIP_GetSecurityString(network->security);
    }

    return MX_WIFI_STATUS_OK;
}

/**
 * @brief Scan for a specific WiFi network
 * @param wifi_obj: Pointer to WiFi object
 * @param ssid: SSID to search for
 * @param results: Pointer to scan results structure
 * @retval MX_WIFI_STATUS_T: Status of scan operation
 */
MX_WIFI_STATUS_T MXCHIP_ScanSpecificNetwork(MX_WIFIObject_t *wifi_obj, const char *ssid, MXCHIP_ScanResults_t *results)
{
    MX_WIFI_STATUS_T status;
    uint32_t start_time;
    int8_t ap_count;
    mwifi_ap_info_t mx_aps[MX_WIFI_MAX_DETECTED_AP];

    if (!wifi_obj || !ssid || !results) {
        return MX_WIFI_STATUS_PARAM_ERROR;
    }

    /* Initialize results structure */
    memset(results, 0, sizeof(MXCHIP_ScanResults_t));
    memset(mx_aps, 0, sizeof(mx_aps));

    printf("Scanning for network: '%s'...\r\n", ssid);
    start_time = HAL_GetTick();

    /* Start active scan for specific SSID */
    status = MX_WIFI_Scan(wifi_obj, MC_SCAN_ACTIVE, (char*)ssid, strlen(ssid));

    if (status != MX_WIFI_STATUS_OK) {
        printf("Failed to start WiFi scan for '%s' (error: %d)\r\n", ssid, status);
        return status;
    }

    /* Wait for scan to complete */
    printf("Scanning in progress...\r\n");
    HAL_Delay(2000); /* Active scan typically faster */

    /* Get scan results */
    ap_count = MX_WIFI_Get_scan_result(wifi_obj, (uint8_t*)mx_aps, MX_WIFI_MAX_DETECTED_AP);

    if (ap_count < 0) {
        printf("Failed to get scan results (error: %d)\r\n", ap_count);
        return MX_WIFI_STATUS_ERROR;
    }

    /* Record scan time */
    results->scan_time_ms = HAL_GetTick() - start_time;

    /* Filter results to match requested SSID */
    int matched_count = 0;
    for (int i = 0; i < ap_count && i < MX_WIFI_MAX_DETECTED_AP; i++) {
        if (strcmp(mx_aps[i].ssid, ssid) == 0) {
            MXCHIP_NetworkInfo_t *network = &results->networks[matched_count];

            /* Copy SSID */
            strncpy(network->ssid, mx_aps[i].ssid, sizeof(network->ssid) - 1);
            network->ssid[sizeof(network->ssid) - 1] = '\0';

            /* Copy BSSID */
            memcpy(network->bssid, mx_aps[i].bssid, 6);

            /* Copy other parameters */
            network->rssi = mx_aps[i].rssi;
            network->channel = mx_aps[i].channel;
            network->security = mx_aps[i].security;
            network->security_str = MXCHIP_GetSecurityString(network->security);

            matched_count++;
        }
    }

    results->count = matched_count;

    printf("✓ Scan completed in %lu ms\r\n", results->scan_time_ms);
    printf("✓ Found %d instances of network '%s'\r\n\r\n", results->count, ssid);

    return MX_WIFI_STATUS_OK;
}

/**
 * @brief Convert security type to human-readable string
 * @param security_type: Security type value
 * @retval const char*: Security type string
 */
const char* MXCHIP_GetSecurityString(uint8_t security_type)
{
    switch (security_type) {
        case MX_WIFI_SEC_NONE:       return "Open";
        case MX_WIFI_SEC_WEP:        return "WEP";
        case MX_WIFI_SEC_WPA_TKIP:   return "WPA-TKIP";
        case MX_WIFI_SEC_WPA_AES:    return "WPA-AES";
        case MX_WIFI_SEC_WPA2_TKIP:  return "WPA2-TKIP";
        case MX_WIFI_SEC_WPA2_AES:   return "WPA2-AES";
        case MX_WIFI_SEC_WPA2_MIXED: return "WPA2-Mixed";
        case MX_WIFI_SEC_AUTO:       return "Auto";
        default:                     return "Unknown";
    }
}

/**
 * @brief Print detailed information about a single network
 * @param network: Pointer to network info structure
 * @param index: Index number for display
 */
void MXCHIP_PrintNetworkInfo(const MXCHIP_NetworkInfo_t *network, uint8_t index)
{
    if (!network) return;

    printf("  [%2d] %-32s", index, network->ssid);
    printf(" | Ch:%2ld", network->channel);
    printf(" | RSSI:%4ld dBm", network->rssi);
    printf(" | %s", network->security_str);
    printf(" | BSSID: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           network->bssid[0], network->bssid[1], network->bssid[2],
           network->bssid[3], network->bssid[4], network->bssid[5]);
}

/**
 * @brief Print complete scan results
 * @param results: Pointer to scan results structure
 */
void MXCHIP_PrintScanResults(const MXCHIP_ScanResults_t *results)
{
    if (!results) return;

    printf("📊 === WiFi Scan Results ===\r\n");
    printf("Scan time: %lu ms\r\n", results->scan_time_ms);
    printf("Networks found: %d\r\n\r\n", results->count);

    if (results->count == 0) {
        printf("No networks found.\r\n\r\n");
        return;
    }

    printf("  #   SSID                             | Ch | RSSI      | Security     | BSSID\r\n");
    printf("  --- -------------------------------- | -- | --------- | ------------ | -----------------\r\n");

    for (int i = 0; i < results->count; i++) {
        MXCHIP_PrintNetworkInfo(&results->networks[i], i + 1);
    }

    printf("\r\n");
}

/**
 * @brief Print information about the strongest network found
 * @param results: Pointer to scan results structure
 */
void MXCHIP_PrintStrongestNetwork(const MXCHIP_ScanResults_t *results)
{
    if (!results || results->count == 0) {
        printf("No networks available for analysis.\r\n\r\n");
        return;
    }

    /* Find strongest signal */
    const MXCHIP_NetworkInfo_t *strongest = &results->networks[0];
    for (int i = 1; i < results->count; i++) {
        if (results->networks[i].rssi > strongest->rssi) {
            strongest = &results->networks[i];
        }
    }

    printf("=== Strongest Signal ===\r\n");
    printf("SSID: %s\r\n", strongest->ssid);
    printf("RSSI: %ld dBm\r\n", strongest->rssi);
    printf("Channel: %ld\r\n", strongest->channel);
    printf("Security: %s\r\n", strongest->security_str);
    printf("BSSID: %02X:%02X:%02X:%02X:%02X:%02X\r\n\r\n",
           strongest->bssid[0], strongest->bssid[1], strongest->bssid[2],
           strongest->bssid[3], strongest->bssid[4], strongest->bssid[5]);
}

/**
 * @brief Print network statistics
 * @param results: Pointer to scan results structure
 */
void MXCHIP_PrintNetworkStatistics(const MXCHIP_ScanResults_t *results)
{
    if (!results || results->count == 0) {
        printf("No networks available for statistics.\r\n\r\n");
        return;
    }

    /* Calculate statistics */
    int32_t total_rssi = 0;
    int32_t min_rssi = results->networks[0].rssi;
    int32_t max_rssi = results->networks[0].rssi;

    int security_counts[8] = {0}; /* Count for each security type */
    int channel_counts[14] = {0}; /* Count for channels 1-14 */

    for (int i = 0; i < results->count; i++) {
        const MXCHIP_NetworkInfo_t *network = &results->networks[i];

        /* RSSI statistics */
        total_rssi += network->rssi;
        if (network->rssi < min_rssi) min_rssi = network->rssi;
        if (network->rssi > max_rssi) max_rssi = network->rssi;

        /* Security type count */
        if (network->security < 8) {
            security_counts[network->security]++;
        }

        /* Channel count */
        if (network->channel >= 1 && network->channel <= 14) {
            channel_counts[network->channel - 1]++;
        }
    }

    int32_t avg_rssi = total_rssi / results->count;

    printf("=== Network Statistics ===\r\n");
    printf("Total networks: %d\r\n", results->count);
    printf("RSSI - Min: %ld dBm, Max: %ld dBm, Avg: %ld dBm\r\n",
           min_rssi, max_rssi, avg_rssi);

    printf("\nSecurity Distribution:\r\n");
    const char* security_names[] = {"Open", "WEP", "WPA-TKIP", "WPA-AES",
                                   "WPA2-TKIP", "WPA2-AES", "WPA2-Mixed", "Auto"};
    for (int i = 0; i < 8; i++) {
        if (security_counts[i] > 0) {
            printf("  %s: %d\r\n", security_names[i], security_counts[i]);
        }
    }

    printf("\nChannel Distribution:\r\n");
    for (int i = 0; i < 14; i++) {
        if (channel_counts[i] > 0) {
            printf("  Channel %d: %d\r\n", i + 1, channel_counts[i]);
        }
    }

    printf("\r\n");
}

/* Private Functions */

/**
 * @brief Reset the MXCHIP module using hardware reset pin
 */
static void MXCHIP_Reset(void)
{
    /* Pull reset pin low */
    HAL_GPIO_WritePin(MXCHIP_RESET_GPIO_Port, MXCHIP_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    /* Release reset pin */
    HAL_GPIO_WritePin(MXCHIP_RESET_GPIO_Port, MXCHIP_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1000); /* Wait for module to boot */
}

