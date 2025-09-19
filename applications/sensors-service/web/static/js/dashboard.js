class SensorDashboard {
    constructor() {
        this.charts = {};
        this.autoRefreshEnabled = true;
        this.autoRefreshInterval = null;
        this.currentTimeRange = 24;
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.initializeCharts();
        this.loadData();
        this.startAutoRefresh();
    }

    setupEventListeners() {
        // Time range selector
        document.getElementById('time-range').addEventListener('change', (e) => {
            this.currentTimeRange = parseInt(e.target.value);
            this.loadData();
        });

        // Refresh button
        document.getElementById('refresh-btn').addEventListener('click', () => {
            this.loadData();
        });

        // Auto refresh toggle
        document.getElementById('auto-refresh-btn').addEventListener('click', (e) => {
            this.toggleAutoRefresh();
        });

        // Export CSV button
        document.getElementById('export-csv-btn').addEventListener('click', () => {
            this.exportToCSV();
        });
    }

    initializeCharts() {
        // Temperature & Humidity Chart
        const tempHumidityCtx = document.getElementById('temp-humidity-chart').getContext('2d');
        this.charts.tempHumidity = new Chart(tempHumidityCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Temperature (°C)',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        yAxisID: 'y'
                    },
                    {
                        label: 'Humidity (%)',
                        data: [],
                        borderColor: '#4ecdc4',
                        backgroundColor: 'rgba(78, 205, 196, 0.1)',
                        yAxisID: 'y1'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false,
                },
                scales: {
                    x: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Time'
                        }
                    },
                    y: {
                        type: 'linear',
                        display: true,
                        position: 'left',
                        title: {
                            display: true,
                            text: 'Temperature (°C)'
                        }
                    },
                    y1: {
                        type: 'linear',
                        display: true,
                        position: 'right',
                        title: {
                            display: true,
                            text: 'Humidity (%)'
                        },
                        grid: {
                            drawOnChartArea: false,
                        },
                    }
                }
            }
        });

        // PM Chart
        const pmCtx = document.getElementById('pm-chart').getContext('2d');
        this.charts.pm = new Chart(pmCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'PM1.0',
                        data: [],
                        borderColor: '#95e1d3',
                        backgroundColor: 'rgba(149, 225, 211, 0.1)'
                    },
                    {
                        label: 'PM2.5',
                        data: [],
                        borderColor: '#f38ba8',
                        backgroundColor: 'rgba(243, 139, 168, 0.1)'
                    },
                    {
                        label: 'PM4.0',
                        data: [],
                        borderColor: '#fab387',
                        backgroundColor: 'rgba(250, 179, 135, 0.1)'
                    },
                    {
                        label: 'PM10',
                        data: [],
                        borderColor: '#f9e2af',
                        backgroundColor: 'rgba(249, 226, 175, 0.1)'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false,
                },
                scales: {
                    x: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Time'
                        }
                    },
                    y: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Concentration (µg/m³)'
                        }
                    }
                }
            }
        });

        // Air Quality Chart
        const airQualityCtx = document.getElementById('air-quality-chart').getContext('2d');
        this.charts.airQuality = new Chart(airQualityCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'VOC Index',
                        data: [],
                        borderColor: '#a6e3a1',
                        backgroundColor: 'rgba(166, 227, 161, 0.1)'
                    },
                    {
                        label: 'NOx Index',
                        data: [],
                        borderColor: '#cba6f7',
                        backgroundColor: 'rgba(203, 166, 247, 0.1)'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: 'index',
                    intersect: false,
                },
                scales: {
                    x: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Time'
                        }
                    },
                    y: {
                        display: true,
                        title: {
                            display: true,
                            text: 'Index Value'
                        }
                    }
                }
            }
        });
    }

    async loadData() {
        try {
            this.showLoading();
            
            // Load latest data for current values
            await this.loadLatestData();
            
            // Load historical data for charts and table
            await this.loadHistoricalData();
            
            // Load statistics
            await this.loadStatistics();
            
            this.updateConnectionStatus(true);
            this.updateLastUpdateTime();
            
        } catch (error) {
            console.error('Error loading data:', error);
            this.updateConnectionStatus(false);
            this.showError('Failed to load data');
        } finally {
            this.hideLoading();
        }
    }

    async loadLatestData() {
        const response = await fetch('/api/latest');
        if (!response.ok) throw new Error('Failed to fetch latest data');
        
        const data = await response.json();
        
        if (data.error) {
            this.updateCurrentValues(null);
            return;
        }
        
        this.updateCurrentValues(data);
    }

    async loadHistoricalData() {
        const response = await fetch(`/api/measurements?hours=${this.currentTimeRange}&limit=200`);
        if (!response.ok) throw new Error('Failed to fetch historical data');
        
        const data = await response.json();
        
        this.updateCharts(data);
        this.updateTable(data);
    }

    async loadStatistics() {
        const response = await fetch(`/api/stats?hours=${this.currentTimeRange}`);
        if (!response.ok) throw new Error('Failed to fetch statistics');
        
        const data = await response.json();
        
        this.updateStatistics(data);
    }

    updateCurrentValues(data) {
        const elements = {
            'current-temp': data?.temperature ? `${data.temperature.toFixed(1)}` : '--',
            'current-humidity': data?.humidity ? `${data.humidity.toFixed(1)}` : '--',
            'current-pm25': data?.pm2_5 ? `${data.pm2_5.toFixed(1)}` : '--',
            'current-voc': data?.voc_index ? `${data.voc_index}` : '--'
        };

        Object.entries(elements).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element) {
                element.textContent = value;
                element.classList.add('fade-in');
            }
        });
    }

    updateCharts(data) {
        if (!data || data.length === 0) return;

        // Reverse data to show chronological order
        const reversedData = [...data].reverse();
        
        const labels = reversedData.map(item => 
            new Date(item.time).toLocaleTimeString('en-US', { 
                month: 'short', 
                day: 'numeric', 
                hour: '2-digit', 
                minute: '2-digit' 
            })
        );

        // Update Temperature & Humidity Chart
        this.charts.tempHumidity.data.labels = labels;
        this.charts.tempHumidity.data.datasets[0].data = reversedData.map(item => item.temperature);
        this.charts.tempHumidity.data.datasets[1].data = reversedData.map(item => item.humidity);
        this.charts.tempHumidity.update('none');

        // Update PM Chart
        this.charts.pm.data.labels = labels;
        this.charts.pm.data.datasets[0].data = reversedData.map(item => item.pm1_0);
        this.charts.pm.data.datasets[1].data = reversedData.map(item => item.pm2_5);
        this.charts.pm.data.datasets[2].data = reversedData.map(item => item.pm4_0);
        this.charts.pm.data.datasets[3].data = reversedData.map(item => item.pm10);
        this.charts.pm.update('none');

        // Update Air Quality Chart
        this.charts.airQuality.data.labels = labels;
        this.charts.airQuality.data.datasets[0].data = reversedData.map(item => item.voc_index);
        this.charts.airQuality.data.datasets[1].data = reversedData.map(item => item.nox_index);
        this.charts.airQuality.update('none');
    }

    updateTable(data) {
        const tbody = document.getElementById('measurements-tbody');
        
        if (!data || data.length === 0) {
            tbody.innerHTML = '<tr><td colspan="9" class="loading">No data available</td></tr>';
            return;
        }

        tbody.innerHTML = data.slice(0, 50).map(item => `
            <tr>
                <td>${new Date(item.time).toLocaleString()}</td>
                <td>${item.pm1_0 ? item.pm1_0.toFixed(1) : '--'}</td>
                <td>${item.pm2_5 ? item.pm2_5.toFixed(1) : '--'}</td>
                <td>${item.pm4_0 ? item.pm4_0.toFixed(1) : '--'}</td>
                <td>${item.pm10 ? item.pm10.toFixed(1) : '--'}</td>
                <td>${item.temperature ? item.temperature.toFixed(1) : '--'}</td>
                <td>${item.humidity ? item.humidity.toFixed(1) : '--'}</td>
                <td>${item.voc_index || '--'}</td>
                <td>${item.nox_index || '--'}</td>
            </tr>
        `).join('');
    }

    updateStatistics(data) {
        const elements = {
            'total-records': data.total_records || '--',
            'avg-pm25': data.avg_pm2_5 ? `${data.avg_pm2_5.toFixed(1)} µg/m³` : '--',
            'max-pm25': data.max_pm2_5 ? `${data.max_pm2_5.toFixed(1)} µg/m³` : '--',
            'avg-temp': data.avg_temperature ? `${data.avg_temperature.toFixed(1)}°C` : '--',
            'avg-humidity': data.avg_humidity ? `${data.avg_humidity.toFixed(1)}%` : '--'
        };

        Object.entries(elements).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element) {
                element.textContent = value;
            }
        });
    }

    updateConnectionStatus(isOnline) {
        const statusDot = document.getElementById('connection-status');
        statusDot.className = `status-dot ${isOnline ? 'online' : 'offline'}`;
    }

    updateLastUpdateTime() {
        const now = new Date();
        const timeString = now.toLocaleTimeString();
        
        document.getElementById('last-update').textContent = `Last updated: ${timeString}`;
        document.getElementById('footer-timestamp').textContent = timeString;
    }

    toggleAutoRefresh() {
        const btn = document.getElementById('auto-refresh-btn');
        this.autoRefreshEnabled = !this.autoRefreshEnabled;
        
        btn.setAttribute('data-enabled', this.autoRefreshEnabled);
        btn.textContent = this.autoRefreshEnabled ? '⏱️ Auto Refresh' : '⏸️ Paused';
        
        if (this.autoRefreshEnabled) {
            this.startAutoRefresh();
        } else {
            this.stopAutoRefresh();
        }
    }

    startAutoRefresh() {
        if (this.autoRefreshInterval) {
            clearInterval(this.autoRefreshInterval);
        }
        
        this.autoRefreshInterval = setInterval(() => {
            if (this.autoRefreshEnabled) {
                this.loadData();
            }
        }, 30000); // Refresh every 30 seconds
    }

    stopAutoRefresh() {
        if (this.autoRefreshInterval) {
            clearInterval(this.autoRefreshInterval);
            this.autoRefreshInterval = null;
        }
    }

    showLoading() {
        const refreshBtn = document.getElementById('refresh-btn');
        refreshBtn.innerHTML = '<span class="spinner"></span>Loading...';
        refreshBtn.disabled = true;
    }

    hideLoading() {
        const refreshBtn = document.getElementById('refresh-btn');
        refreshBtn.innerHTML = '🔄 Refresh';
        refreshBtn.disabled = false;
    }

    showError(message) {
        console.error(message);
        // You could add a toast notification here
    }

    async exportToCSV() {
        try {
            const response = await fetch(`/api/measurements?hours=${this.currentTimeRange}&limit=1000`);
            if (!response.ok) throw new Error('Failed to fetch data for export');
            
            const data = await response.json();
            
            if (!data || data.length === 0) {
                alert('No data available for export');
                return;
            }

            const headers = ['Time', 'PM1.0', 'PM2.5', 'PM4.0', 'PM10', 'Temperature', 'Humidity', 'VOC Index', 'NOx Index'];
            const csvContent = [
                headers.join(','),
                ...data.map(item => [
                    new Date(item.time).toISOString(),
                    item.pm1_0 || '',
                    item.pm2_5 || '',
                    item.pm4_0 || '',
                    item.pm10 || '',
                    item.temperature || '',
                    item.humidity || '',
                    item.voc_index || '',
                    item.nox_index || ''
                ].join(','))
            ].join('\n');

            const blob = new Blob([csvContent], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `sensor-data-${new Date().toISOString().split('T')[0]}.csv`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);
            
        } catch (error) {
            console.error('Error exporting CSV:', error);
            alert('Failed to export data');
        }
    }
}

// Initialize dashboard when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new SensorDashboard();
});
