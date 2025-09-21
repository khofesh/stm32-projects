package com.fahimtu.sen55reader.model

import android.bluetooth.BluetoothDevice

data class BleDevice(
    val device: BluetoothDevice? = null, // Optional for Kable compatibility
    val rssi: Int,
    val name: String?,
    val address: String = device?.address ?: "" // Explicit address for Kable
) {
    val displayName: String get() = name ?: "Unknown Device"

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is BleDevice) return false
        return address == other.address
    }

    override fun hashCode(): Int {
        return address.hashCode()
    }
}