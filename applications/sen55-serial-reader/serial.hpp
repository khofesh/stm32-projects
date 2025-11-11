#pragma once

#include <string>

class SerialPort {
private:
    int fd;
    
public:
    SerialPort(const char* port);
    ~SerialPort();
    
    std::string readLine();
};
