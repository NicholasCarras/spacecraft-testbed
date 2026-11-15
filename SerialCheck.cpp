#include <iostream>

#include "FlightTypes.h"
#include "SensorProtocol.h"
#include "SerialPort.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: serial-check <device-path>\n";
        return 2;
    }

    SerialPort port;

    if (!port.openPort(argv[1])) {
        std::cerr << "Could not open the serial connection\n";
        return 1;
    }

    int validReadings = 0;

    for (int attempt = 0; attempt < 10; ++attempt) {
        std::string line;

        if (!port.readLine(line)) {
            std::cerr << "Serial read failed or timed out\n";
            return 1;
        }

        SensorReading reading{};

        if (!parseSensorMessage(line, reading)) {
            std::cerr << "Skipping unrecognized line: " << line << '\n';
            continue;
        }

        std::cout << "Parsed acceleration: "
                  << reading.accelX << ", "
                  << reading.accelY << ", "
                  << reading.accelZ << " m/s^2\n";

        ++validReadings;
    }

    return validReadings > 0 ? 0 : 1;
}