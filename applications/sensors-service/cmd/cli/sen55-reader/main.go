//go:build linux || darwin || windows
// +build linux darwin windows

// how to:
// go run main.go 00:80:E1:26:06:2E

package main

import (
	"encoding/binary"
	"fmt"
	"os"
	"time"

	"tinygo.org/x/bluetooth"
)

var adapter = bluetooth.DefaultAdapter

const (
	SEN55_SERVICE_UUID = "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"
	LED_CHAR_UUID      = "0000fe41-8e22-4541-9d4c-21edae82ed19"
	SEN55_CHAR_UUID    = "0000fe42-8e22-4541-9d4c-21edae82ed19"
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

// parse raw bytes from BLE notification into SEN55Data
func parseSEN55Data(data []byte) (*SEN55Data, error) {
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

func main() {

	println("enabling")

	// enable BLE interface.
	must("enable BLE stack", adapter.Enable())

	ch := make(chan bluetooth.ScanResult, 1)

	// start scanning.
	println("scanning...")
	err := adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		println("found device:", result.Address.String(), result.RSSI, result.LocalName())
		if result.Address.String() == connectAddress() {
			adapter.StopScan()
			ch <- result
		}
	})

	var device bluetooth.Device
	select {
	case result := <-ch:
		device, err = adapter.Connect(result.Address, bluetooth.ConnectionParams{})
		if err != nil {
			println(err.Error())
			return
		}

		println("connected to ", result.Address.String())
	}

	// get services
	println("discovering services/characteristics")
	srvcs, err := device.DiscoverServices(nil)
	must("discover services", err)

	// find SEN55 service and characteristics
	// var sen55Service bluetooth.DeviceService
	var ledChar, sen55Char bluetooth.DeviceCharacteristic

	for _, srvc := range srvcs {
		println("- service", srvc.UUID().String())

		if srvc.UUID().String() == SEN55_SERVICE_UUID {
			// sen55Service = srvc
			println("  *** Found SEN55 Service! ***")
		}

		chars, err := srvc.DiscoverCharacteristics(nil)
		if err != nil {
			println("  error discovering characteristics:", err)
			continue
		}

		for _, char := range chars {
			println("-- characteristic", char.UUID().String())

			switch char.UUID().String() {
			case LED_CHAR_UUID:
				ledChar = char
				println("   *** LED Control Characteristic ***")
			case SEN55_CHAR_UUID:
				sen55Char = char
				println("   *** SEN55 Data Characteristic ***")
			}

			mtu, err := char.GetMTU()
			if err != nil {
				println("    mtu: error:", err.Error())
			} else {
				println("    mtu:", mtu)
			}
		}
	}

	// test LED control
	if ledChar.UUID().String() == LED_CHAR_UUID {
		println("\n=== Testing LED Control ===")

		println("Turning LED ON...")
		ledOn := []byte{0x00, 0x01}
		ledRes, err := ledChar.WriteWithoutResponse(ledOn)
		if err != nil {
			println("Error turning LED on:", err.Error())
		} else {
			println("LED turned ON successfully!")
		}

		time.Sleep(2 * time.Second)

		println("Turning LED OFF...")
		ledOff := []byte{0x00, 0x00}
		ledRes, err = ledChar.WriteWithoutResponse(ledOff)
		if err != nil {
			println("Error turning LED off:", err.Error())
		} else {
			println("LED turned OFF successfully!")
		}

		println("ledRes: ", ledRes)
	}

	// Set up sensor data notifications
	if sen55Char.UUID().String() == SEN55_CHAR_UUID {
		println("\n=== Setting up SEN55 Data Notifications ===")

		err := sen55Char.EnableNotifications(func(buf []byte) {
			println("\n*** Received SEN55 Data Notification ***")
			println("Raw data length:", len(buf))
			print("Raw bytes: ")
			for i, b := range buf {
				if i < 16 { // Only show first 16 bytes
					print(fmt.Sprintf("%02X ", b))
				}
			}
			println()

			if len(buf) >= 16 {
				sensorData, err := parseSEN55Data(buf)
				if err != nil {
					println("Error parsing sensor data:", err.Error())
				} else {
					println("-----------------------------------")
					println(sensorData.String())
					println("-----------------------------------")
				}
			}
		})

		if err != nil {
			println("Error enabling notifications:", err.Error())
		} else {
			println("Notifications enabled successfully!")
			println("Waiting for sensor data (should arrive every ~5 seconds)...")
			println("Press Ctrl+C to exit")

			// run for to receive notifications
			for i := 0; i < 60; i++ {
				time.Sleep(5 * time.Second)
				println("Waiting for data... (", i*5, "seconds elapsed)")
			}
		}
	} else {
		println("SEN55 characteristic not found!")
	}

	err = device.Disconnect()
	if err != nil {
		println(err)
	}

	fmt.Println("done")
}

func must(action string, err error) {
	if err != nil {
		panic("failed to " + action + ": " + err.Error())
	}
}

func connectAddress() string {
	if len(os.Args) < 2 {
		println("usage: discover [address]")
		os.Exit(1)
	}

	// look for device with specific name
	address := os.Args[1]

	return address
}
