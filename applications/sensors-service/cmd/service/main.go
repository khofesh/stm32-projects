package main

import (
	"database/sql"
	"encoding/json"
	"fmt"
	"html/template"
	"log"
	"net/http"
	"os"
	"strconv"
	"time"

	"sensors-service/internal/db"

	_ "github.com/lib/pq"
)

type Config struct {
	BLEAddress                 string `json:"ble_address"`
	Database                   DatabaseConfig `json:"database"`
	CollectionDurationSeconds int `json:"collection_duration_seconds"`
}

type DatabaseConfig struct {
	Host     string `json:"host"`
	Port     int    `json:"port"`
	User     string `json:"user"`
	Password string `json:"password"`
	DBName   string `json:"dbname"`
}

type Measurement struct {
	Time        time.Time `json:"time"`
	PM1_0       *float64  `json:"pm1_0"`
	PM2_5       *float64  `json:"pm2_5"`
	PM4_0       *float64  `json:"pm4_0"`
	PM10        *float64  `json:"pm10"`
	Temperature *float64  `json:"temperature"`
	Humidity    *float64  `json:"humidity"`
	VOCIndex    *int      `json:"voc_index"`
	NOxIndex    *int      `json:"nox_index"`
}

type Server struct {
	db       *sql.DB
	templates *template.Template
}

func main() {
	// Load configuration
	config, err := loadConfig("config.json")
	if err != nil {
		log.Fatalf("Failed to load config: %v", err)
	}

	// Connect to database
	dsn := fmt.Sprintf("host=%s port=%d user=%s password=%s dbname=%s sslmode=disable",
		config.Database.Host, config.Database.Port, config.Database.User,
		config.Database.Password, config.Database.DBName)

	dbConfig := db.DBConfig{
		Dsn:          dsn,
		MaxOpenConns: 25,
		MaxIdleConns: 25,
		MaxIdleTime:  15 * time.Minute,
	}

	database, err := db.OpenDB(dbConfig)
	if err != nil {
		log.Fatalf("Failed to connect to database: %v", err)
	}
	defer database.Close()

	log.Println("Connected to database successfully")

	// Parse templates
	templates, err := template.ParseGlob("web/templates/*.html")
	if err != nil {
		log.Fatalf("Failed to parse templates: %v", err)
	}

	// Create server
	server := &Server{
		db:        database,
		templates: templates,
	}

	// Setup routes
	http.HandleFunc("/", server.handleHome)
	http.HandleFunc("/api/measurements", server.handleMeasurements)
	http.HandleFunc("/api/latest", server.handleLatest)
	http.HandleFunc("/api/stats", server.handleStats)
	http.Handle("/static/", http.StripPrefix("/static/", http.FileServer(http.Dir("web/static/"))))

	port := ":8080"
	log.Printf("Starting server on port %s", port)
	log.Fatal(http.ListenAndServe(port, nil))
}

func loadConfig(filename string) (*Config, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	var config Config
	err = json.NewDecoder(file).Decode(&config)
	return &config, err
}

func (s *Server) handleHome(w http.ResponseWriter, r *http.Request) {
	err := s.templates.ExecuteTemplate(w, "index.html", nil)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
}

func (s *Server) handleMeasurements(w http.ResponseWriter, r *http.Request) {
	// Parse query parameters
	limitStr := r.URL.Query().Get("limit")
	offsetStr := r.URL.Query().Get("offset")
	hoursStr := r.URL.Query().Get("hours")

	limit := 100 // default
	if limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 && l <= 1000 {
			limit = l
		}
	}

	offset := 0 // default
	if offsetStr != "" {
		if o, err := strconv.Atoi(offsetStr); err == nil && o >= 0 {
			offset = o
		}
	}

	hours := 24 // default to last 24 hours
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 && h <= 168 { // max 1 week
			hours = h
		}
	}

	// Query database
	query := `
		SELECT time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index
		FROM public.measurements
		WHERE time >= NOW() - INTERVAL '%d hours'
		ORDER BY time DESC
		LIMIT $1 OFFSET $2
	`

	rows, err := s.db.Query(fmt.Sprintf(query, hours), limit, offset)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var measurements []Measurement
	for rows.Next() {
		var m Measurement
		err := rows.Scan(&m.Time, &m.PM1_0, &m.PM2_5, &m.PM4_0, &m.PM10,
			&m.Temperature, &m.Humidity, &m.VOCIndex, &m.NOxIndex)
		if err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
		measurements = append(measurements, m)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(measurements)
}

func (s *Server) handleLatest(w http.ResponseWriter, r *http.Request) {
	query := `
		SELECT time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index
		FROM public.measurements
		ORDER BY time DESC
		LIMIT 1
	`

	var m Measurement
	err := s.db.QueryRow(query).Scan(&m.Time, &m.PM1_0, &m.PM2_5, &m.PM4_0, &m.PM10,
		&m.Temperature, &m.Humidity, &m.VOCIndex, &m.NOxIndex)
	if err != nil {
		if err == sql.ErrNoRows {
			w.Header().Set("Content-Type", "application/json")
			json.NewEncoder(w).Encode(map[string]string{"error": "No data available"})
			return
		}
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(m)
}

func (s *Server) handleStats(w http.ResponseWriter, r *http.Request) {
	hoursStr := r.URL.Query().Get("hours")
	hours := 24 // default to last 24 hours
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 && h <= 168 {
			hours = h
		}
	}

	query := `
		SELECT 
			COUNT(*) as total_records,
			AVG(pm2_5) as avg_pm2_5,
			MAX(pm2_5) as max_pm2_5,
			MIN(pm2_5) as min_pm2_5,
			AVG(temperature) as avg_temperature,
			MAX(temperature) as max_temperature,
			MIN(temperature) as min_temperature,
			AVG(humidity) as avg_humidity,
			MAX(humidity) as max_humidity,
			MIN(humidity) as min_humidity
		FROM public.measurements
		WHERE time >= NOW() - INTERVAL '%d hours'
	`

	type Stats struct {
		TotalRecords    int      `json:"total_records"`
		AvgPM2_5       *float64 `json:"avg_pm2_5"`
		MaxPM2_5       *float64 `json:"max_pm2_5"`
		MinPM2_5       *float64 `json:"min_pm2_5"`
		AvgTemperature *float64 `json:"avg_temperature"`
		MaxTemperature *float64 `json:"max_temperature"`
		MinTemperature *float64 `json:"min_temperature"`
		AvgHumidity    *float64 `json:"avg_humidity"`
		MaxHumidity    *float64 `json:"max_humidity"`
		MinHumidity    *float64 `json:"min_humidity"`
	}

	var stats Stats
	err := s.db.QueryRow(fmt.Sprintf(query, hours)).Scan(
		&stats.TotalRecords, &stats.AvgPM2_5, &stats.MaxPM2_5, &stats.MinPM2_5,
		&stats.AvgTemperature, &stats.MaxTemperature, &stats.MinTemperature,
		&stats.AvgHumidity, &stats.MaxHumidity, &stats.MinHumidity)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(stats)
}
