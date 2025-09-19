package config

import "time"

type Config struct {
	BLEAddress string `json:"ble_address"`
	Database   struct {
		Host     string `json:"host"`
		Port     int    `json:"port"`
		User     string `json:"user"`
		Password string `json:"password"`
		DBName   string `json:"dbname"`
	} `json:"database"`
	CollectionDuration time.Duration `json:"collection_duration_seconds"`
}

type ServiceConfig struct {
	BLEAddress                string         `json:"ble_address"`
	Database                  DatabaseConfig `json:"database"`
	CollectionDurationSeconds int            `json:"collection_duration_seconds"`
}

type DatabaseConfig struct {
	Host     string `json:"host"`
	Port     int    `json:"port"`
	User     string `json:"user"`
	Password string `json:"password"`
	DBName   string `json:"dbname"`
}
