/*
 * net_interface.h
 *
 *  Created on: Oct 4, 2025
 *      Author: fahmad
 */

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "net_connect.h"
#include "net_wifi.h"
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>

typedef struct
{
  const char *ssid;
  const char *pwd;
} ap_t;

void NetWifiGetDefaultStation(net_wifi_credentials_t *WifiCreds, const ap_t net_wifi_registred_hotspot[]);
net_if_handle_t *NetInterfaceOn(net_if_driver_init_func driver_init, net_if_notify_func notify_func);
void NetInterfaceConnect(net_if_handle_t *netif, bool dhcp_mode, void *credential, net_wifi_mode_t mode);
void NetInterfaceDisconnect(net_if_handle_t *netif);
void NetInterfaceOff(net_if_handle_t *netif);
int32_t scan_cmd(int32_t argc, char **argv);


/* The network interface in use. */
extern net_if_handle_t *Netif;

#ifdef __cplusplus
}
#endif
