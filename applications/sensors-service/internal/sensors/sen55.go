//go:build linux || darwin || windows
// +build linux darwin windows

package sensors

import (
	"encoding/binary"
	"fmt"
)

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

// convert raw sensor data to human-readable values
func (s *SEN55Data) GetPM1_0() float32 { return float32(s.PM1_0) / 10.0 }
func (s *SEN55Data) GetPM2_5() float32 { return float32(s.PM2_5) / 10.0 }
func (s *SEN55Data) GetPM4_0() float32 { return float32(s.PM4_0) / 10.0 }
func (s *SEN55Data) GetPM10() float32  { return float32(s.PM10) / 10.0 }

func (s *SEN55Data) GetTemperature() float32 {
	if s.Temperature == 0x7fff {
		// invalid reading marker
		return -999
	}
	return float32(s.Temperature) / 200.0
}

func (s *SEN55Data) GetHumidity() float32 {
	if s.Humidity == 0x7fff {
		// invalid reading marker
		return -999
	}
	return float32(s.Humidity) / 100.0
}

func (s *SEN55Data) GetVOCIndex() float32 {
	if s.VOCIndex == 0x7fff {
		// invalid reading marker
		return -999
	}
	return float32(s.VOCIndex) / 10.0
}

func (s *SEN55Data) GetNOxIndex() float32 {
	if s.NOxIndex == 0x7fff {
		// invalid reading marker
		return -999
	}
	return float32(s.NOxIndex) / 10.0
}

func (s *SEN55Data) String() string {
	temp := s.GetTemperature()
	hum := s.GetHumidity()
	voc := s.GetVOCIndex()
	nox := s.GetNOxIndex()

	tempStr := "n/a"
	humStr := "n/a"
	vocStr := "n/a"
	noxStr := "n/a"

	if temp != -999 {
		tempStr = fmt.Sprintf("%.1f °C", temp)
	}
	if hum != -999 {
		humStr = fmt.Sprintf("%.1f %%RH", hum)
	}
	if voc != -999 {
		vocStr = fmt.Sprintf("%.1f", voc)
	}
	if nox != -999 {
		noxStr = fmt.Sprintf("%.1f", nox)
	}

	return fmt.Sprintf(
		"PM1.0: %.1f µg/m³, PM2.5: %.1f µg/m³, PM4.0: %.1f µg/m³, PM10: %.1f µg/m³\n"+
			"Temperature: %s, Humidity: %s\n"+
			"VOC Index: %s, NOx Index: %s",
		s.GetPM1_0(), s.GetPM2_5(), s.GetPM4_0(), s.GetPM10(),
		tempStr, humStr, vocStr, noxStr)
}

// ParseSEN55Data parses raw bytes from BLE notification into SEN55Data
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
