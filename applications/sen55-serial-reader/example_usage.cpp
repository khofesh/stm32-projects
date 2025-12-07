#include <iostream>
#include <string>
#include <iomanip>
#include "serial.hpp"
#include "db.hpp"

struct Sen55Data
{
    float pm1_0, pm2_5, pm4_0, pm10;
    float temperature, humidity;
    int voc_index, nox_index;
};

bool parseJSON(const std::string& json, Sen55Data& data) {
    int parsed = sscanf(json.c_str(),
        "{\"pm1_0\":%f,\"pm2_5\":%f,\"pm4_0\":%f,\"pm10\":%f,"
        "\"temperature\":%f,\"humidity\":%f,\"voc_index\":%d,\"nox_index\":%d}",
        &data.pm1_0, &data.pm2_5, &data.pm4_0, &data.pm10,
        &data.temperature, &data.humidity, &data.voc_index, &data.nox_index);
    
    return (parsed == 8);
}

// Convert Sen55Data to Measurement struct
Measurement toMeasurement(const Sen55Data& data) {
    Measurement m;
    m.pm1_0 = data.pm1_0;
    m.pm2_5 = data.pm2_5;
    m.pm4_0 = data.pm4_0;
    m.pm10 = data.pm10;
    
    // Handle invalid sensor readings (0x7fff = -0.01 for temp/humidity)
    // Adjust these thresholds based on your sensor's invalid value indicators
    if (data.temperature > -100.0f && data.temperature < 100.0f) {
        m.temperature = data.temperature;
    } else {
        m.temperature = std::nullopt;  // NULL in database
    }
    
    if (data.humidity >= 0.0f && data.humidity <= 100.0f) {
        m.humidity = data.humidity;
    } else {
        m.humidity = std::nullopt;
    }
    
    if (data.voc_index >= 0) {
        m.voc_index = data.voc_index;
    } else {
        m.voc_index = std::nullopt;
    }
    
    if (data.nox_index >= 0) {
        m.nox_index = data.nox_index;
    } else {
        m.nox_index = std::nullopt;
    }
    
    return m;
}

int main(int argc, char** argv) {
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " /dev/ttyACM0" << "\n";
        std::cerr << "example: sudo ./build/sen55-serial-reader /dev/ttyACM1" << "\n";
        return 1;
    }
    
    try {
        // Database connection string format:
        // "host=localhost port=5432 dbname=your_db user=your_user password=your_password"
        std::string db_connection = "host=localhost port=5432 dbname=sensors user=postgres password=your_password";
        
        // Initialize database connection
        DbPostgresql db(db_connection);
        
        if (!db.isConnected()) {
            std::cerr << "Failed to connect to database" << std::endl;
            return 1;
        }
        
        // Initialize serial port
        SerialPort serial(argv[1]);
        std::cout << "Connected to serial port!\n" << std::endl;
        
        while (true) {
            std::string line = serial.readLine();
            
            Sen55Data data;
            if (parseJSON(line, data)) {
                // Display data
                std::cout << std::fixed << std::setprecision(1)
                          << "PM2.5: " << data.pm2_5 << " µg/m³ | "
                          << "Temp: " << data.temperature << "°C | "
                          << "Hum: " << data.humidity << "% | "
                          << "VOC: " << data.voc_index << " | "
                          << "NOx: " << data.nox_index << std::endl;
                
                // Convert to Measurement and insert to database
                Measurement measurement = toMeasurement(data);
                db.insert(measurement);
                std::cout << "Data saved to database" << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
