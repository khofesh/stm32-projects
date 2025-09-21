package com.fahimtu.sen55reader.ble

import android.Manifest
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.pm.PackageManager
import android.location.LocationManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.annotation.RequiresPermission
import androidx.core.app.ActivityCompat
import com.fahimtu.sen55reader.model.SEN55Data
import com.fahimtu.sen55reader.model.BleDevice
import java.util.UUID


class BleManager(private val context: Context) {
    companion object {
        private const val TAG = "BleManager"
    }

    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter = bluetoothManager.adapter
    private val bluetoothLeScanner = bluetoothAdapter.bluetoothLeScanner
    private val handler = Handler(Looper.getMainLooper())

    private var bluetoothGatt: BluetoothGatt? = null
    private var sen55Characteristic: BluetoothGattCharacteristic? = null
    private var ledCharacteristic: BluetoothGattCharacteristic? = null

    // callbacks
    var onDeviceFound: ((BleDevice) -> Unit)? = null
    var onScanComplete: (() -> Unit)? = null
    var onConnectionStateChanged: ((Boolean, String?) -> Unit)? = null
    var onSensorDataReceived: ((SEN55Data) -> Unit)? = null
    var onError: ((String) -> Unit)? = null

    private var isScanning = false
    private var isConnected = false
    private val discoveredDevices = mutableSetOf<BleDevice>()

    // BLE scan callback
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                Log.w(TAG, "Bluetooth connect permission not granted")
                return
            }

            // Log ALL discovered devices for debugging
            val deviceName = result.device.name ?: "Unknown"
            val deviceAddress = result.device.address
            val rssi = result.rssi
            val scanRecord = result.scanRecord
            
            Log.i(TAG, "=== BLE DEVICE DISCOVERED ===")
            Log.i(TAG, "Name: $deviceName")
            Log.i(TAG, "Address: $deviceAddress")
            Log.i(TAG, "RSSI: $rssi dBm")
            Log.i(TAG, "Callback Type: $callbackType")
            
            // Log scan record details
            scanRecord?.let { record ->
                Log.i(TAG, "Device Name (from scan record): ${record.deviceName}")
                Log.i(TAG, "Advertised Services: ${record.serviceUuids}")
                Log.i(TAG, "Manufacturer Data: ${record.manufacturerSpecificData}")
                Log.i(TAG, "Service Data: ${record.serviceData}")
                Log.i(TAG, "Tx Power Level: ${record.txPowerLevel}")
                Log.i(TAG, "Advertise Flags: ${record.advertiseFlags}")
            }
            Log.i(TAG, "=============================")

            val device = BleDevice(
                device = result.device,
                rssi = result.rssi,
                name = result.device.name
            )

            // Show ALL devices (no filtering for debugging)
            if (discoveredDevices.add(device)) {
                Log.d(TAG, "Added to device list: ${device.displayName} (${device.address}) RSSI: ${device.rssi}")
                onDeviceFound?.invoke(device)
            } else {
                Log.d(TAG, "Device already in list: ${device.displayName} (${device.address})")
            }
        }

        override fun onScanFailed(errorCode: Int) {
            val errorMessage = when (errorCode) {
                ScanCallback.SCAN_FAILED_ALREADY_STARTED -> "Scan already started"
                ScanCallback.SCAN_FAILED_APPLICATION_REGISTRATION_FAILED -> "App registration failed"
                ScanCallback.SCAN_FAILED_FEATURE_UNSUPPORTED -> "BLE feature unsupported"
                ScanCallback.SCAN_FAILED_INTERNAL_ERROR -> "Internal error"
                ScanCallback.SCAN_FAILED_OUT_OF_HARDWARE_RESOURCES -> "Out of hardware resources"
                else -> "Unknown error ($errorCode)"
            }
            Log.e(TAG, "Scan failed: $errorMessage (code: $errorCode)")
            isScanning = false
            onError?.invoke("Scan failed: $errorMessage")
        }
    }

    // GATT callback for connection and data handling
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                return
            }

            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.d(TAG, "Connected to GATT server")
                    isConnected = true
                    handler.post {
                        onConnectionStateChanged?.invoke(true, null)
                    }
                    // Discover services
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.d(TAG, "Disconnected from GATT server")
                    isConnected = false
                    handler.post {
                        onConnectionStateChanged?.invoke(false, null)
                    }
                    cleanup()
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d(TAG, "Services discovered")

                // Find SEN55 service and characteristics
                val sen55Service = gatt.getService(BleConstants.SEN55_SERVICE_UUID)
                if (sen55Service != null) {
                    Log.d(TAG, "Found SEN55 service")

                    sen55Characteristic = sen55Service.getCharacteristic(BleConstants.SEN55_CHAR_UUID)
                    ledCharacteristic = sen55Service.getCharacteristic(BleConstants.LED_CHAR_UUID)

                    if (sen55Characteristic != null) {
                        Log.d(TAG, "Found SEN55 data characteristic")
                        enableNotifications()
                    } else {
                        handler.post {
                            onError?.invoke("SEN55 data characteristic not found")
                        }
                    }

                    if (ledCharacteristic != null) {
                        Log.d(TAG, "Found LED control characteristic")
                    }
                } else {
                    handler.post {
                        onError?.invoke("SEN55 service not found")
                    }
                }
            } else {
                Log.e(TAG, "Service discovery failed with status: $status")
                handler.post {
                    onError?.invoke("Service discovery failed")
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == BleConstants.SEN55_CHAR_UUID) {
                Log.d(TAG, "Received SEN55 data: ${value.size} bytes")

                val sensorData = SEN55Data.fromByteArray(value)
                if (sensorData != null) {
                    handler.post {
                        onSensorDataReceived?.invoke(sensorData)
                    }
                } else {
                    Log.e(TAG, "Failed to parse sensor data")
                }
            }
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.d(TAG, "Characteristic write successful for ${characteristic.uuid}")
            } else {
                Log.e(TAG, "Characteristic write failed with status: $status for ${characteristic.uuid}")
            }
        }
    }

    fun isBluetoothEnabled(): Boolean {
        return bluetoothAdapter.isEnabled
    }

    fun startScan() {
        Log.i(TAG, "=== STARTING BLE SCAN ===")
        
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Log.e(TAG, "Bluetooth scan permission not granted")
            onError?.invoke("Bluetooth scan permission not granted")
            return
        }

        if (!bluetoothAdapter.isEnabled) {
            Log.e(TAG, "Bluetooth is not enabled")
            onError?.invoke("Bluetooth is not enabled")
            return
        }

        // Check location services (required for BLE scanning on Android)
        val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
        val isLocationEnabled = locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) || 
                               locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
        
        Log.i(TAG, "Location services enabled: $isLocationEnabled")
        if (!isLocationEnabled) {
            Log.w(TAG, "Location services disabled - this may affect BLE scanning")
            // Don't return here, just warn - some devices might still work
        }

        if (isScanning) {
            Log.w(TAG, "Scan already in progress")
            return
        }

        discoveredDevices.clear()
        isScanning = true

        // Try different scan settings for better compatibility
        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
            .setMatchMode(ScanSettings.MATCH_MODE_AGGRESSIVE)
            .setNumOfMatches(ScanSettings.MATCH_NUM_MAX_ADVERTISEMENT)
            .setReportDelay(0L) // Report immediately
            .build()

        Log.i(TAG, "Scan settings: Low Latency mode, All matches callback")

        // Check if there are any bonded devices (paired devices)
        try {
            val bondedDevices = bluetoothAdapter.bondedDevices
            Log.i(TAG, "Bonded devices count: ${bondedDevices?.size ?: 0}")
            bondedDevices?.forEach { device ->
                Log.i(TAG, "Bonded device: ${device.name} (${device.address}) - Type: ${device.type}")
            }
        } catch (e: SecurityException) {
            Log.w(TAG, "Cannot access bonded devices: ${e.message}")
        }

        try {
            // Start scan without filters to see all devices
            bluetoothLeScanner.startScan(emptyList(), scanSettings, scanCallback)
            Log.i(TAG, "BLE scan started successfully with advanced settings")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start BLE scan with advanced settings: ${e.message}")
            
            // Try fallback with basic settings
            Log.i(TAG, "Trying fallback scan with basic settings...")
            try {
                val basicScanSettings = ScanSettings.Builder()
                    .setScanMode(ScanSettings.SCAN_MODE_LOW_POWER)
                    .build()
                
                bluetoothLeScanner.startScan(emptyList(), basicScanSettings, scanCallback)
                Log.i(TAG, "BLE scan started successfully with basic settings")
            } catch (e2: Exception) {
                Log.e(TAG, "Failed to start BLE scan with basic settings: ${e2.message}")
                
                // Try legacy scan (no settings)
                Log.i(TAG, "Trying legacy scan (no settings)...")
                try {
                    bluetoothLeScanner.startScan(scanCallback)
                    Log.i(TAG, "BLE scan started successfully with legacy method")
                } catch (e3: Exception) {
                    Log.e(TAG, "All scan methods failed: ${e3.message}")
                    isScanning = false
                    onError?.invoke("Failed to start scan: ${e3.message}")
                    return
                }
            }
        }

        // Stop scan after timeout
        handler.postDelayed({
            Log.i(TAG, "Scan timeout reached, stopping scan")
            stopScan()
        }, BleConstants.SCAN_TIMEOUT_MS)

        Log.i(TAG, "BLE scan will run for ${BleConstants.SCAN_TIMEOUT_MS}ms")
        
        // Add a periodic check to see if scan is actually working
        handler.postDelayed({
            Log.i(TAG, "=== 5-second scan progress check ===")
            Log.i(TAG, "Devices found so far: ${discoveredDevices.size}")
            Log.i(TAG, "Scan still active: $isScanning")
            if (discoveredDevices.isEmpty()) {
                Log.w(TAG, "No devices found after 5 seconds - this is unusual")
                Log.w(TAG, "Possible issues:")
                Log.w(TAG, "1. No BLE devices in range (very unlikely)")
                Log.w(TAG, "2. Phone's BLE scanning is restricted")
                Log.w(TAG, "3. Android system-level BLE issue")
                Log.w(TAG, "Try: Enable Developer Options > Bluetooth HCI snoop log")
            }
        }, 5000)
    }

    fun stopScan() {
        Log.i(TAG, "=== STOPPING BLE SCAN ===")
        
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Log.w(TAG, "Bluetooth scan permission not granted for stop scan")
            return
        }

        if (!isScanning) {
            Log.w(TAG, "Scan not in progress, nothing to stop")
            return
        }

        isScanning = false
        
        try {
            bluetoothLeScanner.stopScan(scanCallback)
            Log.i(TAG, "BLE scan stopped successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping BLE scan: ${e.message}")
        }
        
        Log.i(TAG, "Total devices discovered: ${discoveredDevices.size}")
        discoveredDevices.forEach { device ->
            Log.i(TAG, "Device in list: ${device.displayName} (${device.address})")
        }
        
        onScanComplete?.invoke()
        Log.i(TAG, "========================")
    }

    fun connect(device: BleDevice) {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            onError?.invoke("Bluetooth connect permission not granted")
            return
        }

        if (isConnected) {
            disconnect()
        }

        Log.d(TAG, "Connecting to device: ${device.address}")
        bluetoothGatt = device.device?.connectGatt(context, false, gattCallback)
    }

    fun disconnect() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            return
        }

        bluetoothGatt?.disconnect()
    }

    /**
     * Control LED on the device
     */
    fun controlLed(turnOn: Boolean) {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            return
        }

        val characteristic = ledCharacteristic
        val gatt = bluetoothGatt

        if (characteristic != null && gatt != null && isConnected) {
            val command = if (turnOn) BleConstants.LED_ON_COMMAND else BleConstants.LED_OFF_COMMAND
            val writeResult = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                gatt.writeCharacteristic(characteristic, command, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            } else {
                // For older Android versions, set the write type on the characteristic first
                @Suppress("DEPRECATION")
                characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                @Suppress("DEPRECATION")
                characteristic.value = command
                @Suppress("DEPRECATION")
                gatt.writeCharacteristic(characteristic)
            }
            if (writeResult == BluetoothGatt.GATT_SUCCESS) {
                Log.d(TAG, "LED ${if (turnOn) "ON" else "OFF"} command sent")
            } else {
                Log.e(TAG, "Failed to send LED command, result: $writeResult")
                onError?.invoke("Failed to send LED command")
            }
        } else {
            onError?.invoke("LED control not available")
        }
    }

    private fun enableNotifications() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            return
        }

        val characteristic = sen55Characteristic
        val gatt = bluetoothGatt

        if (characteristic != null && gatt != null) {
            gatt.setCharacteristicNotification(characteristic, true)

            // Enable notifications on the remote device
            val descriptor = characteristic.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
            if (descriptor != null) {
                val writeResult = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    gatt.writeDescriptor(descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
                } else {
                    TODO("VERSION.SDK_INT < TIRAMISU")
                }
                if (writeResult == BluetoothGatt.GATT_SUCCESS) {
                    Log.d(TAG, "Notifications enabled for SEN55 data")
                } else {
                    Log.e(TAG, "Failed to enable notifications, result: $writeResult")
                    handler.post {
                        onError?.invoke("Failed to enable notifications")
                    }
                }
            } else {
                Log.e(TAG, "Client Characteristic Configuration descriptor not found")
                handler.post {
                    onError?.invoke("Notification descriptor not found")
                }
            }
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun cleanup() {
        sen55Characteristic = null
        ledCharacteristic = null
        bluetoothGatt?.close()
        bluetoothGatt = null
    }

    fun isConnected(): Boolean = isConnected

    fun isScanning(): Boolean = isScanning
}