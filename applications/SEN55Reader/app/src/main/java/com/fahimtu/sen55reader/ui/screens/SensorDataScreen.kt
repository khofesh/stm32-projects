package com.fahimtu.sen55reader.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import com.fahimtu.sen55reader.model.SEN55Data
import com.fahimtu.sen55reader.viewmodel.SEN55ViewModel

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SensorDataScreen(
    viewModel: SEN55ViewModel,
    modifier: Modifier = Modifier
) {
    val sensorData by viewModel.sensorData.observeAsState()
    val connectionStatus by viewModel.connectionStatus.observeAsState("")
    val scrollState = rememberScrollState()
    
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(scrollState)
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        // Header with connection status
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(
                containerColor = MaterialTheme.colorScheme.primaryContainer
            )
        ) {
            Column(
                modifier = Modifier.padding(16.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Icon(
                    imageVector = Icons.Default.Sensors,
                    contentDescription = "Sensor",
                    modifier = Modifier.size(48.dp),
                    tint = MaterialTheme.colorScheme.primary
                )
                Spacer(modifier = Modifier.height(8.dp))
                Text(
                    text = "SEN55 Sensor Data",
                    style = MaterialTheme.typography.headlineSmall,
                    fontWeight = FontWeight.Bold
                )
                Text(
                    text = connectionStatus,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
        
        // Particulate Matter Group
        SensorGroup(
            title = "Particulate Matter (PM)",
            icon = Icons.Default.Air,
            iconColor = Color(0xFF9C27B0) // Purple
        ) {
            sensorData?.let { data ->
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    SensorCard(
                        label = "PM1.0",
                        value = "%.1f".format(data.getPM1_0()),
                        unit = "µg/m³",
                        modifier = Modifier.weight(1f)
                    )
                    SensorCard(
                        label = "PM2.5",
                        value = "%.1f".format(data.getPM2_5()),
                        unit = "µg/m³",
                        modifier = Modifier.weight(1f)
                    )
                }
                Spacer(modifier = Modifier.height(8.dp))
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    SensorCard(
                        label = "PM4.0",
                        value = "%.1f".format(data.getPM4_0()),
                        unit = "µg/m³",
                        modifier = Modifier.weight(1f)
                    )
                    SensorCard(
                        label = "PM10",
                        value = "%.1f".format(data.getPM10()),
                        unit = "µg/m³",
                        modifier = Modifier.weight(1f)
                    )
                }
            } ?: run {
                NoDataCard("No PM data available")
            }
        }
        
        // Temperature & Humidity Group
        SensorGroup(
            title = "Temperature & Humidity",
            icon = Icons.Default.Thermostat,
            iconColor = Color(0xFF2196F3) // Blue
        ) {
            sensorData?.let { data ->
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    val temperature = data.getTemperature()
                    val humidity = data.getHumidity()
                    
                    SensorCard(
                        label = "Temperature",
                        value = temperature?.let { "%.1f".format(it) } ?: "n/a",
                        unit = "°C",
                        modifier = Modifier.weight(1f),
                        isValid = temperature != null
                    )
                    SensorCard(
                        label = "Humidity",
                        value = humidity?.let { "%.1f".format(it) } ?: "n/a",
                        unit = "%RH",
                        modifier = Modifier.weight(1f),
                        isValid = humidity != null
                    )
                }
            } ?: run {
                NoDataCard("No temperature/humidity data available")
            }
        }
        
        // Air Quality Indices Group
        SensorGroup(
            title = "Air Quality Indices",
            icon = Icons.Default.Eco,
            iconColor = Color(0xFF4CAF50) // Green
        ) {
            sensorData?.let { data ->
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    val vocIndex = data.getVOCIndex()
                    val noxIndex = data.getNOxIndex()
                    
                    SensorCard(
                        label = "VOC Index",
                        value = vocIndex?.let { "%.0f".format(it) } ?: "n/a",
                        unit = "",
                        modifier = Modifier.weight(1f),
                        isValid = vocIndex != null
                    )
                    SensorCard(
                        label = "NOx Index",
                        value = noxIndex?.let { "%.0f".format(it) } ?: "n/a",
                        unit = "",
                        modifier = Modifier.weight(1f),
                        isValid = noxIndex != null
                    )
                }
            } ?: run {
                NoDataCard("No air quality data available")
            }
        }
        
        // LED Control Section
        Card(
            modifier = Modifier.fillMaxWidth()
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Icon(
                        imageVector = Icons.Default.Lightbulb,
                        contentDescription = "LED Control",
                        tint = Color(0xFFFF9800) // Orange
                    )
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = "LED Control",
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.SemiBold
                    )
                }
                Spacer(modifier = Modifier.height(12.dp))
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Button(
                        onClick = { viewModel.controlLed(true) },
                        modifier = Modifier.weight(1f),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = Color(0xFF4CAF50)
                        )
                    ) {
                        Icon(
                            imageVector = Icons.Default.LightMode,
                            contentDescription = null,
                            modifier = Modifier.size(16.dp)
                        )
                        Spacer(modifier = Modifier.width(4.dp))
                        Text("LED ON")
                    }
                    Button(
                        onClick = { viewModel.controlLed(false) },
                        modifier = Modifier.weight(1f),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = Color(0xFFF44336)
                        )
                    ) {
                        Icon(
                            imageVector = Icons.Default.LightMode,
                            contentDescription = null,
                            modifier = Modifier.size(16.dp)
                        )
                        Spacer(modifier = Modifier.width(4.dp))
                        Text("LED OFF")
                    }
                }
            }
        }
        
        // Disconnect button
        OutlinedButton(
            onClick = { viewModel.disconnect() },
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.outlinedButtonColors(
                contentColor = MaterialTheme.colorScheme.error
            )
        ) {
            Icon(
                imageVector = Icons.Default.BluetoothDisabled,
                contentDescription = null,
                modifier = Modifier.size(20.dp)
            )
            Spacer(modifier = Modifier.width(8.dp))
            Text("Disconnect")
        }
    }
}

@Composable
fun SensorGroup(
    title: String,
    icon: ImageVector,
    iconColor: Color,
    content: @Composable () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth()
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    imageVector = icon,
                    contentDescription = title,
                    tint = iconColor
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = title,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.SemiBold
                )
            }
            Spacer(modifier = Modifier.height(12.dp))
            content()
        }
    }
}

@Composable
fun SensorCard(
    label: String,
    value: String,
    unit: String,
    modifier: Modifier = Modifier,
    isValid: Boolean = true
) {
    Card(
        modifier = modifier,
        colors = CardDefaults.cardColors(
            containerColor = if (isValid) {
                MaterialTheme.colorScheme.surfaceVariant
            } else {
                MaterialTheme.colorScheme.errorContainer.copy(alpha = 0.3f)
            }
        )
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(12.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = label,
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                textAlign = TextAlign.Center
            )
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = value,
                style = MaterialTheme.typography.headlineSmall,
                fontWeight = FontWeight.Bold,
                color = if (isValid) {
                    MaterialTheme.colorScheme.onSurface
                } else {
                    MaterialTheme.colorScheme.onErrorContainer
                },
                textAlign = TextAlign.Center
            )
            if (unit.isNotEmpty()) {
                Text(
                    text = unit,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    textAlign = TextAlign.Center
                )
            }
        }
    }
}

@Composable
fun NoDataCard(message: String) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        )
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .padding(24.dp),
            contentAlignment = Alignment.Center
        ) {
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Icon(
                    imageVector = Icons.Default.SensorOccupied,
                    contentDescription = null,
                    modifier = Modifier.size(32.dp),
                    tint = MaterialTheme.colorScheme.onSurfaceVariant
                )
                Spacer(modifier = Modifier.height(8.dp))
                Text(
                    text = message,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    textAlign = TextAlign.Center
                )
            }
        }
    }
}
