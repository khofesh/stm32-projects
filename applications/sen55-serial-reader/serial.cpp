#include "serial.hpp"
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>

SerialPort::SerialPort(const char* port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw std::runtime_error("Error opening serial port");
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Error from tcgetattr");
    }
    
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Error from tcsetattr");
    }
}

SerialPort::~SerialPort() {
    if (fd >= 0) close(fd);
}

std::string SerialPort::readLine() {
    std::string line;
    char c;
    
    while (true) {
        int n = read(fd, &c, 1);
        if (n > 0) {
            if (c == '\n') break;
            else if (c != '\r') line += c;
        }
    }
    return line;
}
