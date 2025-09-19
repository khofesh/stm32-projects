package main

import (
	"encoding/json"
	"fmt"
	"html/template"
	"log"
	"net/http"
	"strconv"
	"time"

	"sensors-service/internal/config"
	"sensors-service/internal/data"
	"sensors-service/internal/db"

	_ "github.com/lib/pq"
)

type Server struct {
	models    *data.Models
	templates *template.Template
}

func main() {
	// Load configuration
	config, err := config.LoadConfig("config.json")
	if err != nil {
		log.Fatalf("Failed to load config: %v", err)
	}

	// Connect to database
	dbConfig := db.DBConfig{
		Dsn:          config.GetDSN(),
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

	// Initialize data models
	models := data.NewModels(database)

	// Parse templates
	templates, err := template.ParseGlob("web/templates/*.html")
	if err != nil {
		log.Fatalf("Failed to parse templates: %v", err)
	}

	// Create server
	server := &Server{
		models:    models,
		templates: templates,
	}

	// Setup routes
	http.HandleFunc("/", server.handleHome)
	http.HandleFunc("/api/measurements", server.handleMeasurements)
	http.HandleFunc("/api/latest", server.handleLatest)
	http.HandleFunc("/api/stats", server.handleStats)
	http.Handle("/static/", http.StripPrefix("/static/", http.FileServer(http.Dir("web/static/"))))

	port := fmt.Sprintf(":%d", config.WebServer.Port)
	log.Printf("Starting server on port %s", port)
	log.Fatal(http.ListenAndServe(port, nil))
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

	// Get measurements from repository
	measurements, err := s.models.Measurements.GetMeasurements(hours, limit, offset)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(measurements)
}

func (s *Server) handleLatest(w http.ResponseWriter, r *http.Request) {
	m, err := s.models.Measurements.GetLatest()
	if err != nil {
		if err == data.ErrRecordNotFound {
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

	stats, err := s.models.Measurements.GetStats(hours)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(stats)
}
