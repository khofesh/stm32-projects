package com.fahimtu.sen55reader.model

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * SEN55 sensor data model based on the STM32WB55 BLE implementation
 * Data format matches the Go implementation in sen55-reader
 */
data class SEN55Data(
    val pm1_0: Int,       // PM1.0 concentration (µg/m³ * 10)
    val pm2_5: Int,       // PM2.5 concentration (µg/m³ * 10)
    val pm4_0: Int,       // PM4.0 concentration (µg/m³ * 10)
    val pm10: Int,        // PM10 concentration (µg/m³ * 10)
    val temperature: Short, // Temperature (°C * 200)
    val humidity: Short,    // Humidity (%RH * 100)
    val vocIndex: Short,    // VOC index (* 10)
    val noxIndex: Short     // NOx index (* 10)
) {
    companion object {
        const val INVALID_READING = 0x7fff.toShort()

        /**
         * Parse raw bytes from BLE notification into SEN55Data
         * Data format: 16 bytes, little-endian
         */
        fun fromByteArray(data: ByteArray): SEN55Data? {
            if (data.size < 16) {
                return null
            }

            val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)

            return SEN55Data(
                pm1_0 = buffer.getShort(0).toInt() and 0xFFFF,
                pm2_5 = buffer.getShort(2).toInt() and 0xFFFF,
                pm4_0 = buffer.getShort(4).toInt() and 0xFFFF,
                pm10 = buffer.getShort(6).toInt() and 0xFFFF,
                temperature = buffer.getShort(8),
                humidity = buffer.getShort(10),
                vocIndex = buffer.getShort(12),
                noxIndex = buffer.getShort(14)
            )
        }
    }

    // Convert raw sensor data to human-readable values
    fun getPM1_0(): Float = pm1_0 / 10.0f
    fun getPM2_5(): Float = pm2_5 / 10.0f
    fun getPM4_0(): Float = pm4_0 / 10.0f
    fun getPM10(): Float = pm10 / 10.0f

    fun getTemperature(): Float? {
        return if (temperature == INVALID_READING) {
            null // Invalid reading
        } else {
            temperature / 200.0f
        }
    }

    fun getHumidity(): Float? {
        return if (humidity == INVALID_READING) {
            null // Invalid reading
        } else {
            humidity / 100.0f
        }
    }

    fun getVOCIndex(): Float? {
        return if (vocIndex == INVALID_READING) {
            null // Invalid reading
        } else {
            vocIndex / 10.0f
        }
    }

    fun getNOxIndex(): Float? {
        return if (noxIndex == INVALID_READING) {
            null // Invalid reading
        } else {
            noxIndex / 10.0f
        }
    }

    override fun toString(): String {
        val temp = getTemperature()
        val hum = getHumidity()
        val voc = getVOCIndex()
        val nox = getNOxIndex()

        val tempStr = temp?.let { "%.1f °C".format(it) } ?: "n/a"
        val humStr = hum?.let { "%.1f %%RH".format(it) } ?: "n/a"
        val vocStr = voc?.let { "%.1f".format(it) } ?: "n/a"
        val noxStr = nox?.let { "%.1f".format(it) } ?: "n/a"

        return """
            PM1.0: %.1f µg/m³, PM2.5: %.1f µg/m³, PM4.0: %.1f µg/m³, PM10: %.1f µg/m³
            Temperature: %s, Humidity: %s
            VOC Index: %s, NOx Index: %s
        """.trimIndent().format(
            getPM1_0(), getPM2_5(), getPM4_0(), getPM10(),
            tempStr, humStr, vocStr, noxStr
        )
    }
}