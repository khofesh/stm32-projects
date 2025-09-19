# SEN55 Sensor Web Dashboard

A modern web dashboard for visualizing SEN55 sensor data stored in TimescaleDB. This service provides real-time monitoring of air quality metrics including particulate matter (PM1.0, PM2.5, PM4.0, PM10), temperature, humidity, VOC index, and NOx index.

## Features

### 🌟 Dashboard Features

- **Real-time Data Display**: Live sensor readings with auto-refresh capability
- **Interactive Charts**: Time-series visualization using Chart.js
- **Responsive Design**: Works on desktop, tablet, and mobile devices
- **Data Export**: CSV export functionality for analysis
- **Statistics Summary**: Aggregated statistics over configurable time periods

### 📊 Visualizations

- **Temperature & Humidity**: Dual-axis line chart
- **Particulate Matter**: Multi-line chart for PM1.0, PM2.5, PM4.0, PM10
- **Air Quality Indices**: VOC and NOx index trends
- **Current Values**: Real-time metric cards
- **Data Table**: Detailed measurement history

### 🔧 API Endpoints

- `GET /` - Main dashboard page
- `GET /api/latest` - Latest sensor reading
- `GET /api/measurements` - Historical measurements with pagination
- `GET /api/stats` - Statistical summaries
- `GET /static/*` - Static assets (CSS, JS)

## Quick Start

### Prerequisites

- Go 1.25+
- TimescaleDB with sensor data
- Configuration file (`config.json`)

### Running the Service

1. **Start the web service:**

   ```bash
   make run/web
   # or
   go run ./cmd/service
   ```

2. **Access the dashboard:**
   Open your browser to `http://localhost:8080`

### Configuration

The service uses the same `config.json` file as the data collector:

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

## API Reference

### GET /api/latest

Returns the most recent sensor measurement.

**Response:**

```json
{
  "time": "2024-01-15T10:30:00Z",
  "pm1_0": 12.5,
  "pm2_5": 15.3,
  "pm4_0": 18.7,
  "pm10": 22.1,
  "temperature": 23.4,
  "humidity": 45.2,
  "voc_index": 120,
  "nox_index": 85
}
```

### GET /api/measurements

Returns historical measurements with optional filtering.

**Query Parameters:**

- `limit` (default: 100, max: 1000) - Number of records to return
- `offset` (default: 0) - Number of records to skip
- `hours` (default: 24, max: 168) - Time range in hours

**Example:**

```
GET /api/measurements?hours=24&limit=200
```

### GET /api/stats

Returns statistical summary for the specified time period.

**Query Parameters:**

- `hours` (default: 24, max: 168) - Time range for statistics

**Response:**

```json
{
  "total_records": 1440,
  "avg_pm2_5": 15.3,
  "max_pm2_5": 45.2,
  "min_pm2_5": 8.1,
  "avg_temperature": 22.5,
  "max_temperature": 28.3,
  "min_temperature": 18.7,
  "avg_humidity": 48.2,
  "max_humidity": 65.1,
  "min_humidity": 32.4
}
```

## Architecture

### Backend (Go)

- **HTTP Server**: Standard library `net/http`
- **Database**: PostgreSQL/TimescaleDB with `lib/pq` driver
- **Templates**: HTML template rendering
- **Static Files**: CSS, JavaScript, and assets

### Frontend (Vanilla JS)

- **Charts**: Chart.js for data visualization
- **Styling**: Modern CSS with gradients and animations
- **Responsive**: Mobile-first design approach
- **Real-time**: Auto-refresh with WebSocket-like updates

## Development

### Building

```bash
make build/service
```

### Code Structure

- **Server struct**: Handles HTTP requests and database connections
- **Measurement struct**: Represents sensor data model
- **Config loading**: JSON configuration management
- **Database queries**: Optimized TimescaleDB queries with proper indexing

### Adding New Features

1. **New API endpoint**: Add handler function and route in `main.go`
2. **Frontend updates**: Modify `dashboard.js` and `index.html`
3. **Styling**: Update `style.css` for visual changes
4. **Database queries**: Add new query functions as needed

## Deployment

### Environment Variables

The service can be configured via environment variables:

- `PORT` - Server port (default: 8080)
- `CONFIG_FILE` - Path to config.json (default: ./config.json)
