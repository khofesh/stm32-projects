package config

import (
	"encoding/json"
	"fmt"
	"os"
	"time"
)

// DatabaseConfig holds database connection configuration
type DatabaseConfig struct {
	Host     string `json:"host"`
	Port     int    `json:"port"`
	User     string `json:"user"`
	Password string `json:"password"`
	DBName   string `json:"dbname"`
}

// Config holds the unified configuration for both CLI and service
type Config struct {
	BLEAddress                string          `json:"ble_address"`
	Database                  DatabaseConfig  `json:"database"`
	CollectionDurationSeconds int             `json:"collection_duration_seconds"`
	WebServer                 WebServerConfig `json:"web_server"`
	MQTT                      MQTTConfig      `json:"mqtt"`
}

// WebServerConfig holds web server specific configuration
type WebServerConfig struct {
	Port int `json:"port"`
}

type MQTTConfig struct {
	Username string `json:"username"`
	Password string `json:"password"`
}

// GetCollectionDuration returns the collection duration as time.Duration
func (c *Config) GetCollectionDuration() time.Duration {
	if c.CollectionDurationSeconds == 0 {
		return 2 * time.Minute // default
	}
	return time.Duration(c.CollectionDurationSeconds) * time.Second
}

// GetDSN returns the database connection string
func (c *Config) GetDSN() string {
	return fmt.Sprintf("host=%s port=%d user=%s password=%s dbname=%s sslmode=disable",
		c.Database.Host, c.Database.Port, c.Database.User,
		c.Database.Password, c.Database.DBName)
}

// LoadConfig loads configuration from a JSON file
func LoadConfig(filename string) (*Config, error) {
	if filename == "" {
		filename = "config.json"
	}

	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to read config file: %w", err)
	}

	var config Config
	if err := json.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("failed to parse config file: %w", err)
	}

	// Set default web server port if not specified
	if config.WebServer.Port == 0 {
		config.WebServer.Port = 8080
	}

	return &config, nil
}
