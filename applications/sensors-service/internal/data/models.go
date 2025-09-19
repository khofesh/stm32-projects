package data

import (
	"database/sql"
	"errors"
)

var (
	ErrRecordNotFound = errors.New("record not found")
	ErrEditConflict   = errors.New("edit conflict")
)

// Models holds all data repositories
type Models struct {
	Measurements MeasurementRepository
}

// NewModels creates a new Models instance with all repositories
func NewModels(db *sql.DB) *Models {
	return &Models{
		Measurements: MeasurementRepository{DB: db},
	}
}
