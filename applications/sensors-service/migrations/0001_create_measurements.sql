-- Migration: Create measurements hypertable for sensor data
-- Requires: TimescaleDB extension

CREATE EXTENSION IF NOT EXISTS timescaledb;

-- Table for sensor measurements
CREATE TABLE IF NOT EXISTS public.measurements (
    time        TIMESTAMPTZ      NOT NULL,
    pm1_0       REAL,
    pm2_5       REAL,
    pm4_0       REAL,
    pm10        REAL,
    temperature REAL,
    humidity    REAL,
    voc_index   REAL,
    nox_index   REAL
);

-- Convert to hypertable
SELECT create_hypertable('public.measurements', 'time', if_not_exists => TRUE);

-- Helpful indexes
CREATE INDEX IF NOT EXISTS idx_measurements_time_desc
    ON public.measurements (time DESC);

-- compress older chunks to save space
ALTER TABLE public.measurements
    SET (timescaledb.compress,
         timescaledb.compress_segmentby = '');

-- Policy: compress data older than 7 days
-- Note: This will error if policy already exists, which is safe to ignore
DO $$
BEGIN
    PERFORM add_compression_policy('public.measurements', INTERVAL '7 days');
EXCEPTION
    WHEN duplicate_object THEN
        NULL; -- Policy already exists, ignore
END $$;

