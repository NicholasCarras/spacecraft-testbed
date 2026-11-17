#pragma once

#include <string>

class SerialPort {

public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool openPort(const std::string& devicePath);
    bool readLine(std::string& line);

    void closePort();

private:
    int fileDescriptor = -1;
    std::string pendingLine;
    bool discardingLine = false;
};