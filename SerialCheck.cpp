#include <iostream>
#include <fstream>
#include <chrono>

#include "FlightTypes.h"
#include "SensorProtocol.h"
#include "SerialPort.h"
#include "FlightLogic.h"

const char* faultName(FaultReason fault) {

    switch(fault) {
        case FaultReason::None:
            return "None";
        case FaultReason::InvalidReading:
            return "InvalidReading";
        case FaultReason::AccelerationAboveLimit:
            return "AccelerationAboveLimit";
        case FaultReason::SensorCommunicationLost:
            return "SensorCommunicationLost";
    }
    return "Unknown";
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: serial-check <device-path>\n";
        return 2;
    }

    std::ofstream logFile("sensor-log.txt", std::ios::app);

    if(!logFile) {
        std::cerr << "Could not open sensor-log.txt\n";
        return 1;
    }

    const auto runStart = std::chrono::steady_clock::now();

    logFile << "Run started\n";
    logFile.flush();

    if(!logFile) {
        std::cerr << "Could not write to sensor-log.txt\n";
        return 1;
    }

    SerialPort port;

    if (!port.openPort(argv[1])) {
        std::cerr << "Could not open the serial connection\n";
        return 1;
    }

    int validReadings = 0;

    FlightState state{};
    const double accelerationLimit = 8.0;

    for (int attempt = 0; attempt < 10; ++attempt) {
        std::string line;

        if (!port.readLine(line)) {
            
            state = handleCommunicationLost(state);

            const auto faultElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - runStart).count();

            logFile << "t_ms " << faultElapsedMs << " Serial read failed or timed out | Mode: Safe\n";

            logFile << "Fault: " << faultName(state.activeFault) << '\n';

            logFile.flush();

            if(!logFile) {
                std::cerr << "Could not write sensor log\n";
            }

            std::cerr << "Serial read failed or timed out\n";
            std::cerr << "Mode: Safe\n";
            if(state.activeFault == FaultReason::SensorCommunicationLost) {
                std::cerr << "Fault: SensorCommunicationLost\n";
            }
            else {
                std::cerr << "Previously recorded fault preserved\n";
            }
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

        state = calculateFlightState(state, reading, accelerationLimit);

        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - runStart).count();

        logFile << "t_ms " << elapsedMs << " ACCEL " << reading.accelX << ' ' << reading.accelY << ' ' << reading.accelZ << " | Mode: " << (state.mode == FlightMode::Safe ? "Safe" : "Collecting") << " | Fault: " << faultName(state.activeFault) << '\n';

        logFile.flush();

        if(!logFile) {
            std::cerr << "Could not write sensor log\n";
            return 1;
        }

        std::cout << "Mode: " << (state.mode == FlightMode::Safe ? "Safe" : "Collecting") << '\n';
    }

    return validReadings > 0 ? 0 : 1;
}