# SEN55 Data Collector

This application connects to a SEN55 sensor via Bluetooth Low Energy (BLE) and collects sensor data, storing it in a TimescaleDB database.

## Requirements

- Linux system with Bluetooth support
- Go 1.25+ for building
- Bluetooth adapter (tested with TP-Link UB500)
- SEN55 BLE device running the STM32WB firmware
- TimescaleDB instance (via Docker Compose)

## Features

- Connects to SEN55 BLE sensor using the same protocol as `sen55-reader`
- Collects PM1.0, PM2.5, PM4.0, PM10, temperature, humidity, VOC index, and NOx index
- Stores data in TimescaleDB hypertable with proper timestamp handling
- Configurable collection duration (default: 2 minutes)

## Configuration

Edit `config.json` to match your setup:

```json
{
  "ble_address": "00:80:E1:26:06:2E",
  "database": {
    "host": "localhost",
    "port": 5432,
    "user": "sensor_user",
    "password": "your_password",
    "dbname": "sensors"
  },
  "collection_duration_seconds": 120
}
```

## Building

```bash
cd /path/to/sensors-service
make build/cli/collect-data
```

## Manual Testing

```bash
docker compose up -d

./scripts/apply_migrations.sh

# Run the collector
./bin/linux/cli/collect-data config.json
```

## Installation as System Service

1. Build the application:

```bash
make build/cli/collect-data
```

2. Install to system directory:

```bash
sudo mkdir -p /opt/sensors-service
sudo cp cp bin/linux/cli/collect-data /opt/sensors-service/
sudo cp config.json /opt/sensors-service/
sudo chmod +x /opt/sensors-service/collect-data
```

3. Install systemd files:

```bash
sudo cp systemd/sensors-collect.service /etc/systemd/system/
sudo cp systemd/sensors-collect.timer /etc/systemd/system/
```

4. Enable and start the timer:

```bash
sudo systemctl daemon-reload
sudo systemctl enable sensors-collect.timer
sudo systemctl start sensors-collect.timer
```

## Monitoring

Check timer status:

```bash
sudo systemctl status sensors-collect.timer
sudo systemctl list-timers sensors-collect.timer
```

Check service logs:

```bash
sudo journalctl -u sensors-collect.service -f
```

Check recent runs:

```bash
sudo journalctl -u sensors-collect.service --since "1 hour ago"
```
