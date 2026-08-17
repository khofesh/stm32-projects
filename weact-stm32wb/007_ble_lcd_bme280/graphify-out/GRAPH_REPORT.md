# Graph Report - .  (2026-08-17)

## Corpus Check
- 112 files · ~180,857 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 956 nodes · 2318 edges · 46 communities (38 shown, 8 thin omitted)
- Extraction: 71% EXTRACTED · 29% INFERRED · 0% AMBIGUOUS · INFERRED: 680 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- HCI LE Command Layer
- SSD1315 OLED Driver
- SHCI System Commands
- HCI Transport Layer
- GAP ACI Commands
- GATT/ATT ACI Commands
- IPCC Hardware Mailbox
- TL Mailbox Core
- App Entry & Init
- Main & Peripheral Init
- BME280 Sensor Driver
- Debug Trace Output
- RTC Timer Server
- HAL ACI & Stack Config
- Event Processing & Lists
- BLE Service Controller
- Newlib Syscalls
- Legacy BLE Aliases
- L2CAP ACI Commands
- Low Power Manager
- IPCC Command Send Path
- UART Ring Buffer
- P2P Service Definition
- P2P Server Application
- LLD Tests Channel
- 802.15.4 MAC Channel
- Zigbee Channel
- BLE Channel Init
- CLI Command Channel
- LLD Tests CLI Send
- OpenThread Send
- Zigbee M4 Ack
- Zigbee M4 Request

## God Nodes (most connected - your core abstractions)
1. `hci_send_req()` - 211 edges
2. `Osal_MemSet()` - 206 edges
3. `Osal_MemCpy()` - 91 edges
4. `shci_send()` - 36 edges
5. `a_ssd1315_multiple_write_byte()` - 26 edges
6. `SVCCTL_SvcInit()` - 21 edges
7. `a_ssd1315_write_byte()` - 17 edges
8. `main()` - 16 edges
9. `Error_Handler()` - 16 edges
10. `bme280_get_regs()` - 13 edges

## Surprising Connections (you probably didn't know these)
- `Adv_Cancel()` --calls--> `aci_gap_set_non_discoverable()`  [INFERRED]
  STM32_WPAN/App/app_ble.c → Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c
- `TL_MM_EvtDone()` --calls--> `HW_IPCC_MM_SendFreeBuf()`  [INFERRED]
  Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c → STM32_WPAN/Target/hw_ipcc.c
- `APPD_EnableCPU2()` --calls--> `SHCI_C2_DEBUG_Init()`  [INFERRED]
  Core/Src/app_debug.c → Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci/shci.c
- `APPD_EnableCPU2()` --calls--> `TL_TRACES_Init()`  [INFERRED]
  Core/Src/app_debug.c → Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c
- `Config_HSE()` --calls--> `OTP_Read()`  [INFERRED]
  Core/Src/app_entry.c → Middlewares/ST/STM32_WPAN/utilities/otp.c

## Import Cycles
- None detected.

## Communities (46 total, 8 thin omitted)

### Community 0 - "HCI LE Command Layer"
Cohesion: 0.07
Nodes (86): Host_Nb_Of_Completed_Pkt_Pair_t, Adv_Set_t, Init_Param_Phy_t, Scan_Param_Phy_t, tBleStatus, hci_disconnect(), hci_host_buffer_size(), hci_host_number_of_completed_packets() (+78 more)

### Community 1 - "SSD1315 OLED Driver"
Cohesion: 0.06
Nodes (76): a_ssd1315_gram_draw_point(), a_ssd1315_gram_show_char(), a_ssd1315_multiple_write_byte(), a_ssd1315_write_byte(), ssd1315_activate_scroll(), ssd1315_clear(), ssd1315_deactivate_scroll(), ssd1315_deinit() (+68 more)

### Community 2 - "SHCI System Commands"
Cohesion: 0.06
Nodes (63): shci_notify_asynch_evt(), SHCI_C2_802_15_4_DeInit(), SHCI_C2_BLE_Init(), SHCI_C2_BLE_LLD_Init(), SHCI_C2_CONCURRENT_EnableNext_802154_EvtNotification(), SHCI_C2_CONCURRENT_GetNextBleEvtTime(), SHCI_C2_CONCURRENT_SetMode(), SHCI_C2_Config() (+55 more)

### Community 3 - "HCI Transport Layer"
Cohesion: 0.06
Nodes (53): APP_BLE_ConnStatus_t, shci_cmd_resp_release(), shci_cmd_resp_wait(), HCI_TL_CmdStatus_t, TL_CmdPacket_t, TL_EvtPacket_t, __WEAK, hci_cmd_resp_release() (+45 more)

### Community 4 - "GAP ACI Commands"
Cohesion: 0.07
Nodes (59): Bonded_Device_Entry_t, List_Entry_t, aci_gap_add_devices_to_list(), aci_gap_additional_beacon_set_data(), aci_gap_additional_beacon_start(), aci_gap_additional_beacon_stop(), aci_gap_adv_clear_sets(), aci_gap_adv_remove_set() (+51 more)

### Community 5 - "GATT/ATT ACI Commands"
Cohesion: 0.08
Nodes (50): Char_Desc_Uuid_t, Char_UUID_t, Handle_Entry_t, Include_UUID_t, aci_att_execute_write_req(), aci_att_find_by_type_value_req(), aci_att_find_info_req(), aci_att_prepare_write_req() (+42 more)

### Community 6 - "IPCC Hardware Mailbox"
Cohesion: 0.07
Nodes (42): TL_LLDTESTS_SendCliRspAck(), TL_LLDTESTS_SendM0CmdAck(), TL_MAC_802_15_4_SendCmd(), __weak, HW_IPCC_BLE_AclDataAckNot(), HW_IPCC_BLE_AclDataEvtHandler(), HW_IPCC_BLE_EvtHandler(), HW_IPCC_BLE_RxEvtNot() (+34 more)

### Community 7 - "TL Mailbox Core"
Cohesion: 0.07
Nodes (40): TL_CmdPacket_t, TL_EvtPacket_t, __WEAK, HW_IPCC_BLE_LLD_ReceiveCliRsp(), HW_IPCC_BLE_LLD_ReceiveM0Cmd(), HW_IPCC_LLDTESTS_ReceiveCliRsp(), HW_IPCC_LLDTESTS_ReceiveM0Cmd(), HW_IPCC_MAC_802_15_4_CmdEvtNot() (+32 more)

### Community 8 - "App Entry & Init"
Cohesion: 0.08
Nodes (32): APPD_EnableCPU2(), APPE_SysEvtError(), APPE_SysEvtReadyProcessing(), APPE_SysStatusNot(), APPE_SysUserEvtRx(), appe_Tl_Init(), SHCI_TL_CmdStatus_t, Config_HSE() (+24 more)

### Community 9 - "Main & Peripheral Init"
Cohesion: 0.10
Nodes (32): ADC_HandleTypeDef, Error_Handler(), main(), MX_ADC1_Init(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init(), MX_I2C3_Init() (+24 more)

### Community 10 - "BME280 Sensor Driver"
Cohesion: 0.13
Nodes (32): are_settings_changed(), bme280_compensate_data(), bme280_get_regs(), bme280_get_sensor_data(), bme280_get_sensor_mode(), bme280_get_sensor_settings(), bme280_init(), bme280_set_regs() (+24 more)

### Community 11 - "Debug Trace Output"
Cohesion: 0.10
Nodes (22): APPD_BleDtbCfg(), APPD_Init(), APPD_SetCPU2GpioConfig(), DbgOutputInit(), DbgOutputTraces(), FILE, DbgTrace_TxCpltCallback(), DbgTraceInit() (+14 more)

### Community 12 - "RTC Timer Server"
Cohesion: 0.12
Nodes (27): RTC_HandleTypeDef, __weak, HW_TS_Create(), HW_TS_Delete(), HW_TS_Init(), HW_TS_RTC_Int_AppNot(), HW_TS_RTC_ReadLeftTicksToCount(), HW_TS_RTC_Wakeup_Handler() (+19 more)

### Community 13 - "HAL ACI & Stack Config"
Cohesion: 0.19
Nodes (26): aci_gap_init(), aci_gap_set_authentication_requirement(), aci_gap_set_io_capability(), aci_gatt_init(), aci_gatt_update_char_value(), aci_hal_ead_encrypt_decrypt(), aci_hal_get_anchor_period(), aci_hal_get_link_status() (+18 more)

### Community 14 - "Event Processing & Lists"
Cohesion: 0.14
Nodes (26): hci_user_evt_proc(), shci_user_evt_proc(), HW_IPCC_BLE_RxEvtNot(), HW_IPCC_SYS_EvtNot(), HW_IPCC_TRACES_EvtNot(), SendFreeBuf(), TL_MM_EvtDone(), TL_MM_Init() (+18 more)

### Community 15 - "BLE Service Controller"
Cohesion: 0.20
Nodes (25): BAS_Init(), BLS_Init(), BVOPUS_STM_Init(), __WEAK, CRS_STM_Init(), DIS_Init(), EDS_STM_Init(), HIDS_Init() (+17 more)

### Community 21 - "Legacy BLE Aliases"
Cohesion: 0.27
Nodes (12): Identity_Entry_t, aci_gap_check_bonded_device(), aci_gatt_permit_read(), aci_gap_add_devices_to_resolving_list(), aci_gap_is_device_bonded(), aci_gap_resolve_private_addr(), aci_gatt_allow_read(), aci_gatt_deny_read() (+4 more)

### Community 22 - "L2CAP ACI Commands"
Cohesion: 0.33
Nodes (10): aci_l2cap_coc_connect(), aci_l2cap_coc_connect_confirm(), aci_l2cap_coc_disconnect(), aci_l2cap_coc_flow_control(), aci_l2cap_coc_reconf(), aci_l2cap_coc_reconf_confirm(), aci_l2cap_coc_tx_data(), aci_l2cap_connection_parameter_update_req() (+2 more)

### Community 23 - "Low Power Manager"
Cohesion: 0.29
Nodes (6): EnterLowPower(), ExitLowPower(), PWR_EnterOffMode(), PWR_EnterStopMode(), PWR_ExitStopMode(), Switch_On_HSI()

### Community 24 - "IPCC Command Send Path"
Cohesion: 0.20
Nodes (10): HW_IPCC_BLE_AclDataAckNot(), HW_IPCC_SYS_CmdEvtNot(), OutputDbgTrace(), TL_BLE_SendAclData(), TL_BLE_SendCmd(), TL_SYS_SendCmd(), HW_IPCC_BLE_SendAclData(), HW_IPCC_BLE_SendCmd() (+2 more)

### Community 26 - "UART Ring Buffer"
Cohesion: 0.62
Nodes (6): RingBuffer_GetDataLength(), RingBuffer_GetFreeSpace(), RingBuffer_Init(), RingBuffer_Read(), RingBuffer_Write(), RingBuffer

### Community 28 - "P2P Server Application"
Cohesion: 0.29
Nodes (5): PeerToPeer_Event_Handler(), P2PS_STM_App_Notification_evt_t, P2PS_APP_Init(), P2PS_STM_App_Notification(), SVCCTL_EvtAckStatus_t

### Community 30 - "LLD Tests Channel"
Cohesion: 0.67
Nodes (3): TL_LLDTESTS_Init(), HW_IPCC_LLDTESTS_Init(), TL_LLD_tests_Config_t

### Community 31 - "802.15.4 MAC Channel"
Cohesion: 0.67
Nodes (3): TL_MAC_802_15_4_Init(), HW_IPCC_MAC_802_15_4_Init(), TL_MAC_802_15_4_Config_t

### Community 32 - "Zigbee Channel"
Cohesion: 0.67
Nodes (3): TL_ZIGBEE_Init(), HW_IPCC_ZIGBEE_Init(), TL_ZIGBEE_Config_t

## Knowledge Gaps
- **8 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `hci_send_req()` connect `HCI LE Command Layer` to `HCI Transport Layer`, `GAP ACI Commands`, `GATT/ATT ACI Commands`, `HAL ACI & Stack Config`, `Event Processing & Lists`, `Legacy BLE Aliases`, `L2CAP ACI Commands`?**
  _High betweenness centrality (0.217) - this node is a cross-community bridge._
- **Why does `APP_BLE_Init()` connect `GAP ACI Commands` to `SHCI System Commands`, `HCI Transport Layer`, `App Entry & Init`, `Main & Peripheral Init`, `RTC Timer Server`, `HAL ACI & Stack Config`, `BLE Service Controller`, `P2P Server Application`?**
  _High betweenness centrality (0.117) - this node is a cross-community bridge._
- **Why does `Osal_MemSet()` connect `HAL ACI & Stack Config` to `HCI LE Command Layer`, `GAP ACI Commands`, `GATT/ATT ACI Commands`, `BLE Core Definitions`, `Legacy BLE Aliases`, `L2CAP ACI Commands`?**
  _High betweenness centrality (0.077) - this node is a cross-community bridge._
- **Are the 207 inferred relationships involving `hci_send_req()` (e.g. with `aci_gap_add_devices_to_list()` and `aci_gap_additional_beacon_set_data()`) actually correct?**
  _`hci_send_req()` has 207 INFERRED edges - model-reasoned connections that need verification._
- **Are the 205 inferred relationships involving `Osal_MemSet()` (e.g. with `aci_gap_add_devices_to_list()` and `aci_gap_additional_beacon_set_data()`) actually correct?**
  _`Osal_MemSet()` has 205 INFERRED edges - model-reasoned connections that need verification._
- **Are the 90 inferred relationships involving `Osal_MemCpy()` (e.g. with `aci_gap_add_devices_to_list()` and `aci_gap_additional_beacon_set_data()`) actually correct?**
  _`Osal_MemCpy()` has 90 INFERRED edges - model-reasoned connections that need verification._
- **Should `HCI LE Command Layer` be split into smaller, more focused modules?**
  _Cohesion score 0.06522320235231222 - nodes in this community are weakly interconnected._