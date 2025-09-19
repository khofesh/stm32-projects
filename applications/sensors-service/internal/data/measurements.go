package data

import (
	"database/sql"
	"fmt"
	"time"
)

// Measurement represents a sensor measurement record
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

// Stats represents statistical data for measurements
type Stats struct {
	TotalRecords   int      `json:"total_records"`
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

// MeasurementRepository handles database operations for measurements
type MeasurementRepository struct {
	DB *sql.DB
}

// Insert adds a new measurement to the database
func (r MeasurementRepository) Insert(m *Measurement) error {
	query := `
		INSERT INTO public.measurements (time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index)
		VALUES (NOW(), $1, $2, $3, $4, $5, $6, $7, $8)
	`

	_, err := r.DB.Exec(query,
		m.PM1_0, m.PM2_5, m.PM4_0, m.PM10,
		m.Temperature, m.Humidity, m.VOCIndex, m.NOxIndex,
	)

	return err
}

// GetLatest retrieves the most recent measurement
func (r MeasurementRepository) GetLatest() (*Measurement, error) {
	query := `
		SELECT time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index
		FROM public.measurements
		ORDER BY time DESC
		LIMIT 1
	`

	var m Measurement
	err := r.DB.QueryRow(query).Scan(&m.Time, &m.PM1_0, &m.PM2_5, &m.PM4_0, &m.PM10,
		&m.Temperature, &m.Humidity, &m.VOCIndex, &m.NOxIndex)

	if err != nil {
		if err == sql.ErrNoRows {
			return nil, ErrRecordNotFound
		}
		return nil, err
	}

	return &m, nil
}

// GetMeasurements retrieves measurements with pagination and time filtering
func (r MeasurementRepository) GetMeasurements(hours, limit, offset int) ([]*Measurement, error) {
	query := `
		SELECT time, pm1_0, pm2_5, pm4_0, pm10, temperature, humidity, voc_index, nox_index
		FROM public.measurements
		WHERE time >= NOW() - INTERVAL '%d hours'
		ORDER BY time DESC
		LIMIT $1 OFFSET $2
	`

	rows, err := r.DB.Query(fmt.Sprintf(query, hours), limit, offset)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	var measurements []*Measurement
	for rows.Next() {
		var m Measurement
		err := rows.Scan(&m.Time, &m.PM1_0, &m.PM2_5, &m.PM4_0, &m.PM10,
			&m.Temperature, &m.Humidity, &m.VOCIndex, &m.NOxIndex)
		if err != nil {
			return nil, err
		}
		measurements = append(measurements, &m)
	}

	return measurements, nil
}

// GetStats retrieves statistical data for measurements within a time range
func (r MeasurementRepository) GetStats(hours int) (*Stats, error) {
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

	var stats Stats
	err := r.DB.QueryRow(fmt.Sprintf(query, hours)).Scan(
		&stats.TotalRecords, &stats.AvgPM2_5, &stats.MaxPM2_5, &stats.MinPM2_5,
		&stats.AvgTemperature, &stats.MaxTemperature, &stats.MinTemperature,
		&stats.AvgHumidity, &stats.MaxHumidity, &stats.MinHumidity)

	if err != nil {
		return nil, err
	}

	return &stats, nil
}
