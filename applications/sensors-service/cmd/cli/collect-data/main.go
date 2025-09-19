//go:build linux || darwin || windows
// +build linux darwin windows

package main

import (
	"log"
	"os"
	"sensors-service/internal/config"
	"sensors-service/internal/data"
	"sensors-service/internal/db"
	"sensors-service/internal/sensors"
	"time"

	_ "github.com/lib/pq"
	"tinygo.org/x/bluetooth"
)


func main() {
	log.SetFlags(log.LstdFlags | log.Lshortfile)
	log.Println("Starting SEN55 data collector...")

	// Load configuration
	configFile := "config.json"
	if len(os.Args) > 1 {
		configFile = os.Args[1]
	}

	config, err := config.LoadConfig(configFile)
	if err != nil {
		log.Fatalf("Configuration error: %v", err)
	}

	log.Printf("Loaded config: BLE Address=%s, DB Host=%s:%d",
		config.BLEAddress, config.Database.Host, config.Database.Port)

	// Connect to database using shared db package
	dbConfig := db.DBConfig{
		Dsn:          config.GetDSN(),
		MaxOpenConns: 25,
		MaxIdleConns: 25,
		MaxIdleTime:  15 * time.Minute,
	}

	database, err := db.OpenDB(dbConfig)
	if err != nil {
		log.Fatalf("Database connection error: %v", err)
	}
	defer database.Close()

	log.Println("Connected to TimescaleDB successfully")

	// Initialize data models
	models := data.NewModels(database)

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
		if service.UUID().String() == sensors.SEN55_SERVICE_UUID {
			log.Println("Found SEN55 service")
			chars, err := service.DiscoverCharacteristics(nil)
			if err != nil {
				log.Printf("Error discovering characteristics: %v", err)
				continue
			}

			for _, char := range chars {
				if char.UUID().String() == sensors.SEN55_CHAR_UUID {
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
	collectionDuration := config.GetCollectionDuration()
	log.Printf("Starting data collection for %v...", collectionDuration)
	dataCount := 0
	startTime := time.Now()

	err = sen55Char.EnableNotifications(func(buf []byte) {
		if len(buf) >= 16 {
			sensorData, err := sensors.ParseSEN55Data(buf)
			if err != nil {
				log.Printf("Error parsing sensor data: %v", err)
				return
			}

			// Convert to measurement and insert into database
			measurement := sensorData.ToMeasurement()
			if err := models.Measurements.Insert(measurement); err != nil {
				log.Printf("Error inserting data: %v", err)
				return
			}

			dataCount++
			log.Printf("Data point %d: PM2.5=%.1f µg/m³, Temp=%.1f°C, Humidity=%.1f%%RH, VOC=%.1f, NOx=%.1f",
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
				}(),
				func() float32 {
					if v := sensorData.GetVOCIndex(); v != nil {
						return *v
					}
					return -999
				}(),
				func() float32 {
					if n := sensorData.GetNOxIndex(); n != nil {
						return *n
					}
					return -999
				}())
		}
	})

	if err != nil {
		log.Fatalf("Failed to enable notifications: %v", err)
	}

	// Run for the configured duration
	for time.Since(startTime) < collectionDuration {
		time.Sleep(1 * time.Second)
	}

	log.Printf("Data collection completed. Collected %d data points in %v", dataCount, collectionDuration)
}
