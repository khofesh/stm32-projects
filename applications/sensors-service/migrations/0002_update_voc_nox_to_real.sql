-- Migration: Update VOC and NOx index columns from SMALLINT to REAL
-- This allows storing decimal values for better precision

-- Update voc_index column type
ALTER TABLE public.measurements 
    ALTER COLUMN voc_index TYPE REAL;

-- Update nox_index column type  
ALTER TABLE public.measurements 
    ALTER COLUMN nox_index TYPE REAL;

-- Add comment for documentation
COMMENT ON COLUMN public.measurements.voc_index IS 'VOC index with decimal precision (e.g., 15.5)';
COMMENT ON COLUMN public.measurements.nox_index IS 'NOx index with decimal precision (e.g., 8.7)';
