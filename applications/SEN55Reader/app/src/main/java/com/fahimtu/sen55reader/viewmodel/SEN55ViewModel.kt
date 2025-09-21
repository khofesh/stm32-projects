package com.fahimtu.sen55reader.viewmodel

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import com.fahimtu.sen55reader.ble.BleManager
import com.fahimtu.sen55reader.model.BleDevice
import com.fahimtu.sen55reader.model.SEN55Data

/**
 * ViewModel for managing SEN55 sensor data and BLE connection state
 */
class SEN55ViewModel(application: Application) : AndroidViewModel(application) {
    
    private val bleManager = BleManager(application)
    
    // UI State
    private val _uiState = MutableLiveData(SEN55UiState())
    val uiState: LiveData<SEN55UiState> = _uiState
    
    // Device list
    private val _devices = MutableLiveData<List<BleDevice>>(emptyList())
    val devices: LiveData<List<BleDevice>> = _devices
    
    // Current sensor data
    private val _sensorData = MutableLiveData<SEN55Data?>()
    val sensorData: LiveData<SEN55Data?> = _sensorData
    
    // Connection status
    private val _connectionStatus = MutableLiveData<String>("")
    val connectionStatus: LiveData<String> = _connectionStatus
    
    // Error messages
    private val _errorMessage = MutableLiveData<String?>()
    val errorMessage: LiveData<String?> = _errorMessage
    
    private var selectedDevice: BleDevice? = null
    
    init {
        setupBleCallbacks()
    }
    
    private fun setupBleCallbacks() {
        bleManager.onDeviceFound = { device ->
            val currentDevices = _devices.value?.toMutableList() ?: mutableListOf()
            if (!currentDevices.any { it.address == device.address }) {
                currentDevices.add(device)
                _devices.postValue(currentDevices)
            }
        }
        
        bleManager.onScanComplete = {
            _uiState.postValue(_uiState.value?.copy(isScanning = false))
        }
        
        bleManager.onConnectionStateChanged = { connected, error ->
            if (connected) {
                _uiState.postValue(_uiState.value?.copy(
                    isConnected = true,
                    currentScreen = Screen.SENSOR_DATA
                ))
                _connectionStatus.postValue("Connected to ${selectedDevice?.displayName}")
            } else {
                _uiState.postValue(_uiState.value?.copy(
                    isConnected = false,
                    currentScreen = Screen.SCAN
                ))
                _connectionStatus.postValue("")
                _sensorData.postValue(null)
                if (error != null) {
                    _errorMessage.postValue("Connection error: $error")
                }
            }
        }
        
        bleManager.onSensorDataReceived = { data ->
            _sensorData.postValue(data)
        }
        
        bleManager.onError = { error ->
            _errorMessage.postValue(error)
        }
    }
    
    fun startScan() {
        if (!bleManager.isBluetoothEnabled()) {
            _errorMessage.postValue("Bluetooth is not enabled")
            return
        }
        
        _devices.postValue(emptyList())
        selectedDevice = null
        _uiState.postValue(_uiState.value?.copy(
            isScanning = true,
            selectedDeviceIndex = -1
        ))
        
        bleManager.startScan()
    }
    
    fun stopScan() {
        bleManager.stopScan()
        _uiState.postValue(_uiState.value?.copy(isScanning = false))
    }
    
    fun selectDevice(device: BleDevice) {
        selectedDevice = device
        val deviceIndex = _devices.value?.indexOf(device) ?: -1
        _uiState.postValue(_uiState.value?.copy(selectedDeviceIndex = deviceIndex))
    }
    
    fun connectToSelectedDevice() {
        selectedDevice?.let { device ->
            _uiState.postValue(_uiState.value?.copy(isConnecting = true))
            bleManager.connect(device)
        } ?: run {
            _errorMessage.postValue("No device selected")
        }
    }
    
    fun disconnect() {
        bleManager.disconnect()
        _uiState.postValue(_uiState.value?.copy(isConnecting = false))
    }
    
    fun controlLed(turnOn: Boolean) {
        bleManager.controlLed(turnOn)
    }
    
    fun clearError() {
        _errorMessage.postValue(null)
    }
    
    fun isBluetoothEnabled(): Boolean = bleManager.isBluetoothEnabled()
    
    override fun onCleared() {
        super.onCleared()
        bleManager.disconnect()
    }
}

/**
 * UI State for the SEN55 application
 */
data class SEN55UiState(
    val currentScreen: Screen = Screen.SCAN,
    val isScanning: Boolean = false,
    val isConnecting: Boolean = false,
    val isConnected: Boolean = false,
    val selectedDeviceIndex: Int = -1
)

/**
 * Screen navigation states
 */
enum class Screen {
    SCAN,
    SENSOR_DATA
}
