#!/usr/bin/env bash
set -euo pipefail

# Apply all SQL files in migrations/ to the running TimescaleDB container via docker compose.
# Requires the service name 'timescaledb' in compose.yaml.

COMPOSE_FILE=${COMPOSE_FILE:-compose.yaml}
DB_SERVICE=${DB_SERVICE:-timescaledb}
DB_USER=${POSTGRES_USER:-sensor_user}
DB_NAME=${POSTGRES_DB:-sensors}

if ! docker compose -f "$COMPOSE_FILE" ps "$DB_SERVICE" >/dev/null 2>&1; then
  echo "Error: docker compose service '$DB_SERVICE' not found (compose file: $COMPOSE_FILE)" >&2
  exit 1
fi

for file in migrations/*.sql; do
  [ -e "$file" ] || { echo "No migration files found in migrations/"; exit 0; }
  echo "Applying migration: $file"
  # Pipe the SQL into psql inside the container
  docker compose -f "$COMPOSE_FILE" exec -T "$DB_SERVICE" \
    psql -U "$DB_USER" -d "$DB_NAME" -v ON_ERROR_STOP=1 < "$file"
  echo "Done: $file"
  echo
done

echo "All migrations applied."
