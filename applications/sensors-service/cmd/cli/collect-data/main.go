//go:build linux || darwin || windows
// +build linux darwin windows

package main

import (
	"database/sql"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"sensors-service/internal/config"
	"time"

	_ "github.com/lib/pq"
	"tinygo.org/x/bluetooth"
)

const (
	SEN55_SERVICE_UUID = "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"
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

func (s *SEN55Data) GetVOCIndex() *int16 {
	if s.VOCIndex == 0x7fff {
		return nil // invalid reading
	}
	val := s.VOCIndex / 10
	return &val
}

func (s *SEN55Data) GetNOxIndex() *int16 {
	if s.NOxIndex == 0x7fff {
		return nil // invalid reading
	}
	val := s.NOxIndex / 10
	return &val
}

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

func loadConfig() (*config.Config, error) {
	configFile := "config.json"
	if len(os.Args) > 1 {
		configFile = os.Args[1]
	}

	data, err := os.ReadFile(configFile)
	if err != nil {
		return nil, fmt.Errorf("failed to read config file: %w", err)
	}

	var config config.Config
	if err := json.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("failed to parse config file: %w", err)
	}

	// Set default collection duration if not specified
	if config.CollectionDuration == 0 {
		config.CollectionDuration = 2 * time.Minute
	} else {
		config.CollectionDuration = config.CollectionDuration * time.Second
	}

	return &config, nil
}

func connectToDatabase(config *config.Config) (*sql.DB, error) {
	dsn := fmt.Sprintf("host=%s port=%d user=%s password=%s dbname=%s sslmode=disable",
		config.Database.Host, config.Database.Port, config.Database.User,
		config.Database.Password, config.Database.DBName)

	db, err := sql.Open("postgres", dsn)
	if err != nil {
		return nil, fmt.Errorf("failed to open database: %w", err)
	}

	if err := db.Ping(); err != nil {
		return nil, fmt.Errorf("failed to connect to database: %w", err)
	}

	return db, nil
}

func insertSensorData(db *sql.DB, data *SEN55Data) error {
	query := `
		INSERT INTO public.measurements (time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index)
		VALUES (NOW(), $1, $2, $3, $4, $5, $6, $7, $8)
	`

	_, err := db.Exec(query,
		data.GetPM1_0(),
		data.GetPM2_5(),
		data.GetPM4_0(),
		data.GetPM10(),
		data.GetTemperature(),
		data.GetHumidity(),
		data.GetVOCIndex(),
		data.GetNOxIndex(),
	)

	return err
}

func main() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	log.Println("Starting SEN55 data collector...")

	// Load configuration
	config, err := loadConfig()
	if err != nil {
		log.Fatalf("Configuration error: %v", err)
	}

	log.Printf("Loaded config: BLE Address=%s, DB Host=%s:%d",
		config.BLEAddress, config.Database.Host, config.Database.Port)

	// Connect to database
	db, err := connectToDatabase(config)
	if err != nil {
		log.Fatalf("Database connection error: %v", err)
	}
	defer db.Close()

	log.Println("Connected to TimescaleDB successfully")

	// Enable BLE adapter
	adapter := bluetooth.DefaultAdapter
	if err := adapter.Enable(); err != nil {
		log.Fatalf("Failed to enable BLE adapter: %v", err)
	}

	log.Println("BLE adapter enabled, scanning for device...")

	// Scan for the specific device
	ch := make(chan bluetooth.ScanResult, 1)
	err = adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		if result.Address.String() == config.BLEAddress {
			log.Printf("Found target device: %s (RSSI: %d)", result.Address.String(), result.RSSI)
			adapter.StopScan()
			ch <- result
		}
	})

	if err != nil {
		log.Fatalf("Failed to start BLE scan: %v", err)
	}

	// Wait for device discovery with timeout
	var device bluetooth.Device
	select {
	case result := <-ch:
		device, err = adapter.Connect(result.Address, bluetooth.ConnectionParams{})
		if err != nil {
			log.Fatalf("Failed to connect to device: %v", err)
		}
		log.Printf("Connected to %s", result.Address.String())
	case <-time.After(30 * time.Second):
		log.Fatalf("Device discovery timeout: %s not found", config.BLEAddress)
	}

	defer func() {
		if err := device.Disconnect(); err != nil {
			log.Printf("Error disconnecting: %v", err)
		}
	}()

	// Discover services and characteristics
	log.Println("Discovering services and characteristics...")
	services, err := device.DiscoverServices(nil)
	if err != nil {
		log.Fatalf("Failed to discover services: %v", err)
	}

	var sen55Char bluetooth.DeviceCharacteristic
	found := false

	for _, service := range services {
		if service.UUID().String() == SEN55_SERVICE_UUID {
			log.Println("Found SEN55 service")
			chars, err := service.DiscoverCharacteristics(nil)
			if err != nil {
				log.Printf("Error discovering characteristics: %v", err)
				continue
			}

			for _, char := range chars {
				if char.UUID().String() == SEN55_CHAR_UUID {
					sen55Char = char
					found = true
					log.Println("Found SEN55 data characteristic")
					break
				}
			}
		}
	}

	if !found {
		log.Fatalf("SEN55 data characteristic not found")
	}

	// Set up data collection
	log.Printf("Starting data collection for %v...", config.CollectionDuration)
	dataCount := 0
	startTime := time.Now()

	err = sen55Char.EnableNotifications(func(buf []byte) {
		if len(buf) >= 16 {
			sensorData, err := parseSEN55Data(buf)
			if err != nil {
				log.Printf("Error parsing sensor data: %v", err)
				return
			}

			// Insert data into database
			if err := insertSensorData(db, sensorData); err != nil {
				log.Printf("Error inserting data: %v", err)
				return
			}

			dataCount++
			log.Printf("Data point %d: PM2.5=%.1f µg/m³, Temp=%.1f°C, Humidity=%.1f%%RH",
				dataCount, sensorData.GetPM2_5(),
				func() float32 {
					if t := sensorData.GetTemperature(); t != nil {
						return *t
					}
					return -999
				}(),
				func() float32 {
					if h := sensorData.GetHumidity(); h != nil {
						return *h
					}
					return -999
				}())
		}
	})

	if err != nil {
		log.Fatalf("Failed to enable notifications: %v", err)
	}

	// Run for the configured duration
	for time.Since(startTime) < config.CollectionDuration {
		time.Sleep(1 * time.Second)
	}

	log.Printf("Data collection completed. Collected %d data points in %v", dataCount, config.CollectionDuration)
}
