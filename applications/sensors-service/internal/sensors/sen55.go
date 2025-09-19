//go:build linux || darwin || windows
// +build linux darwin windows

package sensors

import (
	"encoding/binary"
	"fmt"

	"sensors-service/internal/data"
)

const (
	SEN55_SERVICE_UUID = "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"
	SEN55_CHAR_UUID    = "0000fe42-8e22-4541-9d4c-21edae82ed19"
)

// SEN55Data represents raw sensor data from the SEN55 sensor
type SEN55Data struct {
	PM1_0       uint16 // PM1.0 concentration (µg/m³ * 10)
	PM2_5       uint16 // PM2.5 concentration (µg/m³ * 10)
	PM4_0       uint16 // PM4.0 concentration (µg/m³ * 10)
	PM10        uint16 // PM10 concentration (µg/m³ * 10)
	Temperature int16  // Temperature (°C * 200)
	Humidity    int16  // Humidity (%RH * 100)
	VOCIndex    int16  // VOC index (* 10)
	NOxIndex    int16  // NOx index (* 10)
}

// Convert raw sensor data to human-readable values
func (s *SEN55Data) GetPM1_0() float32 { return float32(s.PM1_0) / 10.0 }
func (s *SEN55Data) GetPM2_5() float32 { return float32(s.PM2_5) / 10.0 }
func (s *SEN55Data) GetPM4_0() float32 { return float32(s.PM4_0) / 10.0 }
func (s *SEN55Data) GetPM10() float32  { return float32(s.PM10) / 10.0 }

func (s *SEN55Data) GetTemperature() *float32 {
	if s.Temperature == 0x7fff {
		return nil // invalid reading
	}
	val := float32(s.Temperature) / 200.0
	return &val
}

func (s *SEN55Data) GetHumidity() *float32 {
	if s.Humidity == 0x7fff {
		return nil // invalid reading
	}
	val := float32(s.Humidity) / 100.0
	return &val
}

func (s *SEN55Data) GetVOCIndex() *float32 {
	if s.VOCIndex == 0x7fff {
		return nil // invalid reading
	}
	val := float32(s.VOCIndex) / 10.0
	return &val
}

func (s *SEN55Data) GetNOxIndex() *float32 {
	if s.NOxIndex == 0x7fff {
		return nil // invalid reading
	}
	val := float32(s.NOxIndex) / 10.0
	return &val
}

// String returns a human-readable representation of the sensor data
func (s *SEN55Data) String() string {
	temp := s.GetTemperature()
	hum := s.GetHumidity()
	voc := s.GetVOCIndex()
	nox := s.GetNOxIndex()

	tempStr := "n/a"
	humStr := "n/a"
	vocStr := "n/a"
	noxStr := "n/a"

	if temp != nil {
		tempStr = fmt.Sprintf("%.1f °C", *temp)
	}
	if hum != nil {
		humStr = fmt.Sprintf("%.1f %%RH", *hum)
	}
	if voc != nil {
		vocStr = fmt.Sprintf("%.1f", *voc)
	}
	if nox != nil {
		noxStr = fmt.Sprintf("%.1f", *nox)
	}

	return fmt.Sprintf(
		"PM1.0: %.1f µg/m³, PM2.5: %.1f µg/m³, PM4.0: %.1f µg/m³, PM10: %.1f µg/m³\n"+
			"Temperature: %s, Humidity: %s\n"+
			"VOC Index: %s, NOx Index: %s",
		s.GetPM1_0(), s.GetPM2_5(), s.GetPM4_0(), s.GetPM10(),
		tempStr, humStr, vocStr, noxStr)
}

// ParseSEN55Data parses raw BLE data into SEN55Data struct
func ParseSEN55Data(data []byte) (*SEN55Data, error) {
	if len(data) < 16 {
		return nil, fmt.Errorf("insufficient data length: got %d, need 16", len(data))
	}

	sensor := &SEN55Data{
		PM1_0:       binary.LittleEndian.Uint16(data[0:2]),
		PM2_5:       binary.LittleEndian.Uint16(data[2:4]),
		PM4_0:       binary.LittleEndian.Uint16(data[4:6]),
		PM10:        binary.LittleEndian.Uint16(data[6:8]),
		Temperature: int16(binary.LittleEndian.Uint16(data[8:10])),
		Humidity:    int16(binary.LittleEndian.Uint16(data[10:12])),
		VOCIndex:    int16(binary.LittleEndian.Uint16(data[12:14])),
		NOxIndex:    int16(binary.LittleEndian.Uint16(data[14:16])),
	}

	return sensor, nil
}

// ToMeasurement converts SEN55Data to data.Measurement for database storage
func (s *SEN55Data) ToMeasurement() *data.Measurement {
	return &data.Measurement{
		PM1_0:       floatPtr(s.GetPM1_0()),
		PM2_5:       floatPtr(s.GetPM2_5()),
		PM4_0:       floatPtr(s.GetPM4_0()),
		PM10:        floatPtr(s.GetPM10()),
		Temperature: convertFloat32PtrToFloat64Ptr(s.GetTemperature()),
		Humidity:    convertFloat32PtrToFloat64Ptr(s.GetHumidity()),
		VOCIndex:    convertFloat32PtrToIntPtr(s.GetVOCIndex()),
		NOxIndex:    convertFloat32PtrToIntPtr(s.GetNOxIndex()),
	}
}

// Helper functions for type conversions

// floatPtr converts float32 to *float64
func floatPtr(f float32) *float64 {
	val := float64(f)
	return &val
}

// convertFloat32PtrToFloat64Ptr converts *float32 to *float64
func convertFloat32PtrToFloat64Ptr(f *float32) *float64 {
	if f == nil {
		return nil
	}
	val := float64(*f)
	return &val
}

// convertFloat32PtrToIntPtr converts *float32 to *int (for VOC/NOx indices)
func convertFloat32PtrToIntPtr(f *float32) *int {
	if f == nil {
		return nil
	}
	val := int(*f)
	return &val
}
