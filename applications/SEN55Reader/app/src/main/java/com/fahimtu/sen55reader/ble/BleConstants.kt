package com.fahimtu.sen55reader.ble

import java.util.UUID

object BleConstants {
    // Service and characteristic UUIDs from the Go implementation
    val SEN55_SERVICE_UUID: UUID = UUID.fromString("0000fe40-cc7a-482a-984a-7f2ed5b3e58f")
    val LED_CHAR_UUID: UUID = UUID.fromString("0000fe41-8e22-4541-9d4c-21edae82ed19")
    val SEN55_CHAR_UUID: UUID = UUID.fromString("0000fe42-8e22-4541-9d4c-21edae82ed19")

    // LED control commands
    val LED_ON_COMMAND = byteArrayOf(0x00, 0x01)
    val LED_OFF_COMMAND = byteArrayOf(0x00, 0x00)

    // BLE scan settings
    val SCAN_TIMEOUT_MS = 10000L // 10 seconds
    val CONNECTION_TIMEOUT_MS = 5000L // 5 seconds
}