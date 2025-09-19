# Database Migrations (TimescaleDB)

This project uses raw SQL migrations stored in `migrations/`. A helper script is provided to apply them to the TimescaleDB container defined in `compose.yaml`.

## Prerequisites

- Docker and Docker Compose
- The `timescaledb` service from `compose.yaml` running:

```bash
docker compose up -d
```

## Apply migrations

Option 1: Using the helper script (recommended)

```bash
chmod +x scripts/apply_migrations.sh
./scripts/apply_migrations.sh
```

Option 2: Apply a single migration manually via psql

```bash
docker compose exec -T timescaledb psql \
  -U sensor_user -d sensors -v ON_ERROR_STOP=1 \
  < migrations/0001_create_measurements.sql
```

## Schema: measurements hypertable

The initial migration creates a hypertable `public.measurements` with the following columns:

- `time TIMESTAMPTZ NOT NULL`
- `pm1_0 REAL`
- `pm2_5 REAL`
- `pm4_0 REAL`
- `pm10 REAL`
- `temperature REAL`
- `humidity REAL`
- `voc_index REAL`
- `nox_index REAL`

It converts the table to a TimescaleDB hypertable and adds a descending time index. Compression is enabled with a policy to compress chunks older than 7 days.
