# Graph Report - .  (2026-08-22)

## Corpus Check
- 134 files · ~202,089 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1318 nodes · 3027 edges · 113 communities (64 shown, 49 thin omitted)
- Extraction: 73% EXTRACTED · 27% INFERRED · 0% AMBIGUOUS · INFERRED: 813 edges (avg confidence: 0.8)
- Token cost: 71,273 input · 0 output

## Community Hubs (Navigation)
- HCI LE Command Layer
- System Command Interface (SHCI)
- GAP Advertising and Bonding
- BME280 Sensor Application Layer
- HCI Transport and BLE App Core
- GATT and ATT Command API
- BLE Public Header Surface
- IPCC Hardware Channel Layer
- Transport Layer Mailbox
- Debug Trace and UART Output
- Application Entry and Startup
- ATT/GATT Response Event Handlers
- Main Init and Peripheral Setup
- BLE Service Controller Registry
- Event Processing and Memory Manager
- SSD1315 Core Display Driver
- SSD1315 Register Configuration
- GATT Indication and Pairing Events
- Timer Server and Env Scheduling
- HAL Radio and Test Commands
- Custom BME Service and App
- Newlib Syscall Stubs
- Python BLE Test Client
- BLE Advertising Policy
- SSD1315 Basic Drawing API
- Display Application Layer
- SSD1315 Init and Power Modes
- Adaptive Sampling Architecture
- Display Power and Button Design
- Event Callbacks and State Wiring
- Environment State Machine
- L2CAP Connection API
- BLE Advertising Design Notes
- Debug Trace Failure Investigation
- Display Wake and Repaint Logic
- Low Power Mode Interface
- Transport Send Path
- STOP2 Power Design Notes
- Ring Buffer Utility
- Generic ACI Config Commands
- Event-Driven Design Principles
- LE Advertising Report Event
- ATT Find By Type Event
- Directed Advertising Report Event
- SSD1315 Address Pin Config
- SSD1315 Interface Selection
- SSD1315 Charge Pump Config
- SSD1315 COM Pin Config
- GATT Read Multi Permit Event
- Completed Packets Event
- LLD Tests Transport Init
- 802.15.4 MAC Transport Init
- Zigbee Transport Init
- ATT Exec Write Response Event
- ATT Find Info Response Event
- ATT Read Blob Response Event
- ATT Read By Group Type Event
- ATT Read Multiple Response Event
- GAP Address Not Resolved Event
- GAP Authorization Request Event
- GAP Keypress Notification Event
- GAP Limited Discoverable Event
- GAP Pairing Request Event
- GAP Passkey Request Event
- GATT Attribute Modified Event
- GATT Discover Char By UUID Event
- GATT Error Response Event
- GATT Indication Extended Event
- GATT Notification Event
- GATT Notification Extended Event
- GATT Procedure Complete Event
- GATT Procedure Timeout Event
- GATT Read Extended Event
- GATT Server Confirmation Event
- HAL End Of Radio Activity Event
- HAL Scan Request Report Event
- L2CAP CoC Connect Event
- L2CAP CoC Disconnect Event
- L2CAP CoC Flow Control Event
- L2CAP CoC Reconfig Confirm Event
- L2CAP CoC TX Pool Event
- L2CAP Command Reject Event
- L2CAP Connection Update Event
- L2CAP Procedure Timeout Event
- ACI Warning Event
- HCI Disconnection Complete Event
- HCI Hardware Error Event
- LE Channel Selection Event
- LE Connection Update Event
- LE Generate DHKey Event
- LE Long Term Key Request Event
- LE Read Remote Features Event
- LE Scan Request Received Event
- Remote Version Information Event
- BLE Transport Init
- CLI Command Send Path
- LLD Tests CLI Command Path
- OpenThread Command Send Path
- Zigbee M4 Ack Path
- Zigbee M4 Request Path
- Scripts Directory

## God Nodes (most connected - your core abstractions)
1. `hci_send_req()` - 215 edges
2. `Osal_MemSet()` - 210 edges
3. `Osal_MemCpy()` - 95 edges
4. `shci_send()` - 36 edges
5. `ssd1315_basic_init()` - 33 edges
6. `a_ssd1315_multiple_write_byte()` - 26 edges
7. `SVCCTL_SvcInit()` - 21 edges
8. `main()` - 20 edges
9. `a_ssd1315_write_byte()` - 17 edges
10. `Error_Handler()` - 16 edges

## Surprising Connections (you probably didn't know these)
- `shci_cmd_resp_wait()` --calls--> `UTIL_SEQ_WaitEvt()`  [INFERRED]
  Core/Src/app_entry.c → Utilities/sequencer/stm32_seq.c
- `ssd1315_basic_init()` --calls--> `ssd1315_interface_debug_print()`  [INFERRED]
  Drivers/SSD1315/driver_ssd1315_basic.c → Core/Src/driver_ssd1315_interface.c
- `Adv_Cancel()` --calls--> `aci_gap_set_non_discoverable()`  [INFERRED]
  STM32_WPAN/App/app_ble.c → Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c
- `TL_MM_EvtDone()` --calls--> `HW_IPCC_MM_SendFreeBuf()`  [INFERRED]
  Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl/tl_mbox.c → STM32_WPAN/Target/hw_ipcc.c
- `app_ble_policy_publish_task()` --calls--> `aci_gap_update_adv_data()`  [INFERRED]
  Core/Src/app_ble_policy.c → Middlewares/ST/STM32_WPAN/ble/core/auto/ble_gap_aci.c

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **WeEnv Advertising Silence Investigation** — problem_weenv_advertising_failure, problem_cfg_tx_power, problem_appd_init_never_called, problem_pa9_stlink_vcp_dead, problem_radio_activity_mask_useless, problem_no_central_observer_role [EXTRACTED 1.00]
- **STABLE / ACTIVE / INTERACTIVE State Machine** — todo_app_state_stable, todo_app_state_active, todo_app_state_interactive, todo_app_environment, todo_environment_has_changed, todo_app_config [EXTRACTED 1.00]
- **STOP2 Sleep/Wake Chain** — todo_stop2_low_power, todo_clock_restore_on_wake, todo_button_debounce_timer_server, todo_ble_survives_sleep, todo_tick_wraparound, todo_cfg_debugger_supported [EXTRACTED 1.00]

## Communities (113 total, 49 thin omitted)

### Community 0 - "HCI LE Command Layer"
Cohesion: 0.07
Nodes (86): Host_Nb_Of_Completed_Pkt_Pair_t, Adv_Set_t, Init_Param_Phy_t, Scan_Param_Phy_t, tBleStatus, hci_disconnect(), hci_host_buffer_size(), hci_host_number_of_completed_packets() (+78 more)

### Community 1 - "System Command Interface (SHCI)"
Cohesion: 0.06
Nodes (63): shci_notify_asynch_evt(), SHCI_C2_802_15_4_DeInit(), SHCI_C2_BLE_Init(), SHCI_C2_BLE_LLD_Init(), SHCI_C2_CONCURRENT_EnableNext_802154_EvtNotification(), SHCI_C2_CONCURRENT_GetNextBleEvtTime(), SHCI_C2_CONCURRENT_SetMode(), SHCI_C2_Config() (+55 more)

### Community 2 - "GAP Advertising and Bonding"
Cohesion: 0.07
Nodes (65): Bonded_Device_Entry_t, List_Entry_t, aci_gap_add_devices_to_list(), aci_gap_additional_beacon_set_data(), aci_gap_additional_beacon_start(), aci_gap_additional_beacon_stop(), aci_gap_adv_clear_sets(), aci_gap_adv_remove_set() (+57 more)

### Community 3 - "BME280 Sensor Application Layer"
Cohesion: 0.06
Nodes (46): BME280_INTF_RET_TYPE, HAL_Delay(), BME280_APP_GetData(), BME280_APP_GetMeasurementDelayMs(), BME280_APP_Init(), BME280_APP_ReadMeasurement(), BME280_APP_StartMeasurement(), bme280_delay_us() (+38 more)

### Community 4 - "HCI Transport and BLE App Core"
Cohesion: 0.06
Nodes (50): APP_BLE_ConnStatus_t, MX_APPE_Process(), shci_cmd_resp_release(), HCI_TL_CmdStatus_t, TL_CmdPacket_t, TL_EvtPacket_t, __WEAK, hci_cmd_resp_release() (+42 more)

### Community 5 - "GATT and ATT Command API"
Cohesion: 0.10
Nodes (52): Char_Desc_Uuid_t, Char_UUID_t, Handle_Entry_t, Include_UUID_t, aci_att_execute_write_req(), aci_att_find_by_type_value_req(), aci_att_find_info_req(), aci_att_prepare_write_req() (+44 more)

### Community 6 - "BLE Public Header Surface"
Cohesion: 0.05
Nodes (12): Identity_Entry_t, aci_gap_check_bonded_device(), aci_gatt_permit_read(), aci_gap_add_devices_to_resolving_list(), aci_gap_is_device_bonded(), aci_gap_resolve_private_addr(), aci_gatt_allow_read(), aci_gatt_deny_read() (+4 more)

### Community 7 - "IPCC Hardware Channel Layer"
Cohesion: 0.07
Nodes (42): TL_LLDTESTS_SendCliRspAck(), TL_LLDTESTS_SendM0CmdAck(), TL_MAC_802_15_4_SendCmd(), __weak, HW_IPCC_BLE_AclDataAckNot(), HW_IPCC_BLE_AclDataEvtHandler(), HW_IPCC_BLE_EvtHandler(), HW_IPCC_BLE_RxEvtNot() (+34 more)

### Community 8 - "Transport Layer Mailbox"
Cohesion: 0.07
Nodes (40): TL_CmdPacket_t, TL_EvtPacket_t, __WEAK, HW_IPCC_BLE_LLD_ReceiveCliRsp(), HW_IPCC_BLE_LLD_ReceiveM0Cmd(), HW_IPCC_LLDTESTS_ReceiveCliRsp(), HW_IPCC_LLDTESTS_ReceiveM0Cmd(), HW_IPCC_MAC_802_15_4_CmdEvtNot() (+32 more)

### Community 9 - "Debug Trace and UART Output"
Cohesion: 0.08
Nodes (31): APPD_BleDtbCfg(), APPD_Init(), APPD_SetCPU2GpioConfig(), DbgOutputInit(), DbgOutputTraces(), UART_HandleTypeDef, HAL_UART_RxCpltCallback(), HAL_UART_TxCpltCallback() (+23 more)

### Community 10 - "Application Entry and Startup"
Cohesion: 0.07
Nodes (37): APPD_EnableCPU2(), APPE_SysEvtError(), APPE_SysEvtReadyProcessing(), APPE_SysStatusNot(), APPE_SysUserEvtRx(), appe_Tl_Init(), SHCI_TL_CmdStatus_t, Config_HSE() (+29 more)

### Community 11 - "ATT/GATT Response Event Handlers"
Cohesion: 0.07
Nodes (40): aci_att_prepare_write_resp_event(), aci_att_prepare_write_resp_event_process(), aci_att_read_by_type_resp_event(), aci_att_read_by_type_resp_event_process(), aci_att_read_resp_event(), aci_att_read_resp_event_process(), aci_gap_pairing_complete_event(), aci_gap_pairing_complete_event_process() (+32 more)

### Community 12 - "Main Init and Peripheral Setup"
Cohesion: 0.10
Nodes (33): ADC_HandleTypeDef, button_init(), Error_Handler(), main(), MX_ADC1_Init(), MX_DMA_Init(), MX_GPIO_Init(), MX_I2C1_Init() (+25 more)

### Community 13 - "BLE Service Controller Registry"
Cohesion: 0.19
Nodes (26): BAS_Init(), BLS_Init(), BVOPUS_STM_Init(), __WEAK, CRS_STM_Init(), DIS_Init(), EDS_STM_Init(), HIDS_Init() (+18 more)

### Community 14 - "Event Processing and Memory Manager"
Cohesion: 0.14
Nodes (26): hci_user_evt_proc(), shci_user_evt_proc(), HW_IPCC_BLE_RxEvtNot(), HW_IPCC_SYS_EvtNot(), HW_IPCC_TRACES_EvtNot(), SendFreeBuf(), TL_MM_EvtDone(), TL_MM_Init() (+18 more)

### Community 15 - "SSD1315 Core Display Driver"
Cohesion: 0.12
Nodes (21): a_ssd1315_write_byte(), ssd1315_basic_write_point(), ssd1315_activate_scroll(), ssd1315_deactivate_scroll(), ssd1315_gram_read_point(), ssd1315_gram_write_point(), ssd1315_info(), ssd1315_set_display_mode() (+13 more)

### Community 16 - "SSD1315 Register Configuration"
Cohesion: 0.18
Nodes (22): a_ssd1315_multiple_write_byte(), ssd1315_handle_t, ssd1315_set_column_address_range(), ssd1315_set_contrast(), ssd1315_set_display_clock(), ssd1315_set_display_offset(), ssd1315_set_left_horizontal_scroll(), ssd1315_set_left_horizontal_scroll_one_column() (+14 more)

### Community 18 - "GATT Indication and Pairing Events"
Cohesion: 0.10
Nodes (21): aci_att_exchange_mtu_resp_event(), aci_att_exchange_mtu_resp_event_process(), aci_gap_bond_lost_event(), aci_gap_bond_lost_event_process(), aci_gap_numeric_comparison_value_event(), aci_gap_numeric_comparison_value_event_process(), aci_gatt_indication_event(), aci_gatt_indication_event_process() (+13 more)

### Community 19 - "Timer Server and Env Scheduling"
Cohesion: 0.22
Nodes (19): app_env_measure_task(), app_env_schedule_next(), __weak, HW_TS_Delete(), HW_TS_RTC_Int_AppNot(), HW_TS_RTC_ReadLeftTicksToCount(), HW_TS_RTC_Wakeup_Handler(), HW_TS_Start() (+11 more)

### Community 20 - "HAL Radio and Test Commands"
Cohesion: 0.19
Nodes (19): aci_hal_ead_encrypt_decrypt(), aci_hal_get_anchor_period(), aci_hal_get_link_status(), aci_hal_le_tx_test_packet_number(), aci_hal_read_config_data(), aci_hal_read_radio_reg(), aci_hal_read_raw_rssi(), aci_hal_read_rssi() (+11 more)

### Community 21 - "Custom BME Service and App"
Cohesion: 0.16
Nodes (15): APP_BLE_POLICY_OnNotifyEnabled(), Custom_STM_App_Notification_evt_t, Custom_STM_Char_Opcode_t, Custom_APP_Init(), Custom_Bme_c_Send_Notification(), Custom_Bme_c_Update_Char(), Custom_STM_App_Notification(), tBleStatus (+7 more)

### Community 23 - "Python BLE Test Client"
Cohesion: 0.17
Nodes (17): AdvertisementData, BLEDevice, decode_adv(), decode_payload(), describe(), find_device(), main(), matches() (+9 more)

### Community 24 - "BLE Advertising Policy"
Cohesion: 0.18
Nodes (15): app_ble_policy_build_adv(), APP_BLE_POLICY_Init(), app_ble_policy_led_blink(), app_ble_policy_led_init(), APP_BLE_POLICY_OnReading(), app_ble_policy_pack(), app_ble_policy_publish_task(), app_ble_policy_worth_notifying() (+7 more)

### Community 25 - "SSD1315 Basic Drawing API"
Cohesion: 0.15
Nodes (15): a_ssd1315_gram_draw_point(), a_ssd1315_gram_show_char(), ssd1315_font_t, ssd1315_basic_clear(), ssd1315_basic_picture(), ssd1315_basic_read_point(), ssd1315_basic_rect(), ssd1315_basic_string() (+7 more)

### Community 26 - "Display Application Layer"
Cohesion: 0.17
Nodes (14): app_display_push_task(), bme280_app_data_t, DISPLAY_APP_HasPendingFrame(), DISPLAY_APP_Init(), DISPLAY_APP_PowerOn(), DISPLAY_APP_Process(), DISPLAY_APP_ShowMeasurement(), display_push_page() (+6 more)

### Community 27 - "SSD1315 Init and Power Modes"
Cohesion: 0.13
Nodes (15): ssd1315_address_t, ssd1315_interface_t, ssd1315_basic_deinit(), ssd1315_basic_init(), ssd1315_deinit(), ssd1315_init(), ssd1315_set_deselect_level(), ssd1315_set_fade_blinking_mode() (+7 more)

### Community 28 - "Adaptive Sampling Architecture"
Cohesion: 0.19
Nodes (13): Adaptive Sampling Strategy, app_environment Module, APP_STATE_ACTIVE, APP_STATE_STABLE, Thresholded BLE Notifications, bme280_driver Module, BME280 Forced-Mode Strategy, Definition of Done (+5 more)

### Community 29 - "Display Power and Button Design"
Cohesion: 0.17
Nodes (13): app_config.h Central Configuration, app_display Module, APP_STATE_INTERACTIVE, EXTI-Mask Debounce via Timer Server, Button Interrupt Handling, CFG_DEBUGGER_SUPPORTED Trade-Off, oled_driver Module, OLED Power Abstraction (+5 more)

### Community 30 - "Event Callbacks and State Wiring"
Cohesion: 0.17
Nodes (12): APP_BLE_POLICY_OnConnected(), APP_BLE_POLICY_OnDisconnected(), APP_BLE_POLICY_OnStateChanged(), AppState, app_display_timeout_cb(), app_env_conv_timer_cb(), APP_ENV_OnUserInteraction(), app_env_sample_timer_cb() (+4 more)

### Community 31 - "Environment State Machine"
Cohesion: 0.32
Nodes (11): app_env_classify(), APP_ENV_GetReading(), APP_ENV_GetState(), app_env_has_changed(), app_env_interact_task(), APP_ENV_OnDisplayTimeout(), app_env_read_task(), app_env_sample_interval_ms() (+3 more)

### Community 32 - "L2CAP Connection API"
Cohesion: 0.33
Nodes (10): aci_l2cap_coc_connect(), aci_l2cap_coc_connect_confirm(), aci_l2cap_coc_disconnect(), aci_l2cap_coc_flow_control(), aci_l2cap_coc_reconf(), aci_l2cap_coc_reconf_confirm(), aci_l2cap_coc_tx_data(), aci_l2cap_connection_parameter_update_req() (+2 more)

### Community 33 - "BLE Advertising Design Notes"
Cohesion: 0.25
Nodes (11): Advertising Buffer Overflow in app_ble_policy.c, APP_BLE_AD_FIELD() Macro, CFG_TX_POWER Override (0x18 -> 0x1F), Weak RF Path (still open), app_ble_policy Module, BLE Advertising Policy, BLE Connection Parameter Policy, 12-Byte WB_BME280 Characteristic Payload (+3 more)

### Community 34 - "Debug Trace Failure Investigation"
Cohesion: 0.18
Nodes (11): APPD_Init() Never Called, Building from Debug/ Requires ST Toolchain, CLK48 HSEM Lock Before Stack Start, Clocks Ruled Out (HSE/LSE verified), STM32_Programmer_CLI -coreReg Leaves Core Halted, DbgOutputInit() Link Error on MX_USART1_UART_Init, Debug Trace Path, Scan HCI Commands Unsupported (no central/observer role) (+3 more)

### Community 35 - "Display Wake and Repaint Logic"
Cohesion: 0.29
Nodes (8): app_display_needs_repaint(), APP_DISPLAY_OnReading(), app_display_render(), app_display_timeout_task(), APP_DISPLAY_Wake(), bme280_app_data_t, DISPLAY_APP_PowerOff(), ssd1315_basic_display_off()

### Community 36 - "Low Power Mode Interface"
Cohesion: 0.29
Nodes (6): EnterLowPower(), ExitLowPower(), PWR_EnterOffMode(), PWR_EnterStopMode(), PWR_ExitStopMode(), Switch_On_HSI()

### Community 37 - "Transport Send Path"
Cohesion: 0.20
Nodes (10): HW_IPCC_BLE_AclDataAckNot(), HW_IPCC_SYS_CmdEvtNot(), OutputDbgTrace(), TL_BLE_SendAclData(), TL_BLE_SendCmd(), TL_SYS_SendCmd(), HW_IPCC_BLE_SendAclData(), HW_IPCC_BLE_SendCmd() (+2 more)

### Community 39 - "STOP2 Power Design Notes"
Cohesion: 0.29
Nodes (8): Trace and STOP2 Are Mutually Exclusive, app_power Module, BLE Connection Must Survive MCU Sleep, HSE Clock Restore on STOP2 Exit, Build-Time Logging Control, Phase 6 - STM32 Low Power, Power Measurement Instrumentation GPIOs, STOP2 Low-Power Requirement

### Community 40 - "Ring Buffer Utility"
Cohesion: 0.62
Nodes (6): RingBuffer_GetDataLength(), RingBuffer_GetFreeSpace(), RingBuffer_Init(), RingBuffer_Read(), RingBuffer_Write(), RingBuffer

### Community 41 - "Generic ACI Config Commands"
Cohesion: 0.53
Nodes (5): aci_get_information(), aci_read_config_data(), aci_reset(), aci_write_config_data(), tBleStatus

### Community 43 - "Event-Driven Design Principles"
Cohesion: 0.40
Nodes (5): AppEvent Bit-Flag Event Model, Event-Driven Design Philosophy, Avoid Busy Waiting, Important Non-Goals, Power Optimization Rule

### Community 44 - "LE Advertising Report Event"
Cohesion: 0.67
Nodes (3): Advertising_Report_t, hci_le_advertising_report_event(), hci_le_advertising_report_event_process()

### Community 45 - "ATT Find By Type Event"
Cohesion: 0.67
Nodes (3): Attribute_Group_Handle_Pair_t, aci_att_find_by_type_value_resp_event(), aci_att_find_by_type_value_resp_event_process()

### Community 47 - "Directed Advertising Report Event"
Cohesion: 0.67
Nodes (3): Direct_Advertising_Report_t, hci_le_directed_advertising_report_event(), hci_le_directed_advertising_report_event_process()

### Community 48 - "SSD1315 Address Pin Config"
Cohesion: 0.67
Nodes (3): ssd1315_address_t, ssd1315_get_addr_pin(), ssd1315_set_addr_pin()

### Community 49 - "SSD1315 Interface Selection"
Cohesion: 0.67
Nodes (3): ssd1315_interface_t, ssd1315_get_interface(), ssd1315_set_interface()

### Community 50 - "SSD1315 Charge Pump Config"
Cohesion: 0.67
Nodes (3): ssd1315_set_charge_pump(), ssd1315_charge_pump_mode_t, ssd1315_charge_pump_t

### Community 51 - "SSD1315 COM Pin Config"
Cohesion: 0.67
Nodes (3): ssd1315_set_com_pins_hardware_conf(), ssd1315_left_right_remap_t, ssd1315_pin_conf_t

### Community 52 - "GATT Read Multi Permit Event"
Cohesion: 0.67
Nodes (3): Handle_Item_t, aci_gatt_read_multi_permit_req_event(), aci_gatt_read_multi_permit_req_event_process()

### Community 53 - "Completed Packets Event"
Cohesion: 0.67
Nodes (3): Handle_Packets_Pair_Entry_t, hci_number_of_completed_packets_event(), hci_number_of_completed_packets_event_process()

### Community 54 - "LLD Tests Transport Init"
Cohesion: 0.67
Nodes (3): TL_LLDTESTS_Init(), HW_IPCC_LLDTESTS_Init(), TL_LLD_tests_Config_t

### Community 55 - "802.15.4 MAC Transport Init"
Cohesion: 0.67
Nodes (3): TL_MAC_802_15_4_Init(), HW_IPCC_MAC_802_15_4_Init(), TL_MAC_802_15_4_Config_t

### Community 56 - "Zigbee Transport Init"
Cohesion: 0.67
Nodes (3): TL_ZIGBEE_Init(), HW_IPCC_ZIGBEE_Init(), TL_ZIGBEE_Config_t

## Knowledge Gaps
- **10 isolated node(s):** `scripts`, `Weak RF Path (still open)`, `APP_BLE_AD_FIELD() Macro`, `Battery-Powered Environmental Monitor`, `oled_driver Module` (+5 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **49 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Osal_MemCpy()` connect `GAP Advertising and Bonding` to `HCI LE Command Layer`, `L2CAP Connection API`, `GATT and ATT Command API`, `BLE Public Header Surface`, `Generic ACI Config Commands`, `LE Advertising Report Event`, `HAL Radio and Test Commands`?**
  _High betweenness centrality (0.250) - this node is a cross-community bridge._
- **Why does `hci_le_advertising_report_event_process()` connect `LE Advertising Report Event` to `GAP Advertising and Bonding`, `ATT/GATT Response Event Handlers`?**
  _High betweenness centrality (0.240) - this node is a cross-community bridge._
- **Why does `hci_send_req()` connect `GATT and ATT Command API` to `HCI LE Command Layer`, `L2CAP Connection API`, `GAP Advertising and Bonding`, `HCI Transport and BLE App Core`, `BLE Public Header Surface`, `Generic ACI Config Commands`, `Event Processing and Memory Manager`, `HAL Radio and Test Commands`?**
  _High betweenness centrality (0.182) - this node is a cross-community bridge._
- **Are the 211 inferred relationships involving `hci_send_req()` (e.g. with `aci_gap_add_devices_to_list()` and `aci_gap_additional_beacon_set_data()`) actually correct?**
  _`hci_send_req()` has 211 INFERRED edges - model-reasoned connections that need verification._
- **Are the 209 inferred relationships involving `Osal_MemSet()` (e.g. with `aci_gap_add_devices_to_list()` and `aci_gap_additional_beacon_set_data()`) actually correct?**
  _`Osal_MemSet()` has 209 INFERRED edges - model-reasoned connections that need verification._
- **Are the 94 inferred relationships involving `Osal_MemCpy()` (e.g. with `hci_le_advertising_report_event_process()` and `aci_gap_add_devices_to_list()`) actually correct?**
  _`Osal_MemCpy()` has 94 INFERRED edges - model-reasoned connections that need verification._
- **What connects `scripts`, `Weak RF Path (still open)`, `APP_BLE_AD_FIELD() Macro` to the rest of the system?**
  _10 weakly-connected nodes found - possible documentation gaps or missing edges._