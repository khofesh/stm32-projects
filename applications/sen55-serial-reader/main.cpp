#include <iostream>
#include <string>
#include <iomanip>
#include "serial.hpp"

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

// TODO:
// 1. serial port from cli option, default to /dev/ttyACM0
// 2. database connection
// 3. save parsedJSON data to database
int main() {
    try {
        SerialPort serial("/dev/ttyACM1");
        std::cout << "Connected!\n" << std::endl;
        
        while (true) {
            std::string line = serial.readLine();
            
            Sen55Data data;
            if (parseJSON(line, data)) {
                std::cout << std::fixed << std::setprecision(1)
                          << "PM2.5: " << data.pm2_5 << " µg/m³ | "
                          << "Temp: " << data.temperature << "°C | "
                          << "Hum: " << data.humidity << "% | "
                          << "VOC: " << data.voc_index << " | "
                          << "NOx: " << data.nox_index << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

