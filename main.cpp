#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <csignal>

#include "FlightTypes.h"
#include "FlightLogic.h"
#include "FlightTests.h"
#include "SerialPort.h"
#include "SensorProtocol.h"

volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) {
    stopRequested = 1;
}

void printReading(const SensorReading& reading) {

    std::cout << reading.accelX << " m/s^2\n";
    std::cout << reading.accelY << " m/s^2\n";
    std::cout << reading.accelZ << " m/s^2\n";
}

SensorReading readSimulatedSensor(int sampleIndex) {

    SensorReading sensor{sampleIndex * 0.1, 0.0, 9.81};

    return sensor;
}

void printMode(FlightMode mode) {

    if(mode == FlightMode::Collecting) {
        std::cout << "Mode: COLLECTING\n";
    }

    else {
        std::cout << "Mode: SAFE\n";
    }
}

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
        case FaultReason::MalformedSensorMessage:
            return "MalformedSensorMessage";
    }
    return "Unknown";
}

void printFault(FaultReason reason) {

    std::cout << "Fault: " << faultName(reason) << '\n';
}

void runMission() {

    FlightState state{};

    const double accelerationLimit = 0.35;

    const auto sampleInterval = std::chrono::milliseconds(500);

    const auto missionStart = std::chrono::steady_clock::now();

    auto nextSampleTime = missionStart + sampleInterval;


    for(int i = 0; i < 8; i++) {

        std::this_thread::sleep_until(nextSampleTime);
        nextSampleTime += sampleInterval;

        if(state.mode == FlightMode::Safe) {
            std::cout << "SAFE: Collection paused\n";
            continue;
        }

        const auto elapsed = std::chrono::steady_clock::now() - missionStart;
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        SensorReading sensor = readSimulatedSensor(i);

        std::cout << "Reading " << i + 1 << " | Elapsed: " << elapsedMs << " ms\n";
        printReading(sensor);

        FlightMode previousMode = state.mode;

        state = calculateFlightState(state, sensor, accelerationLimit);
        
        if(state.mode != previousMode) {
            std::cout << "Mode Changed\n";
            printMode(state.mode);
            printFault(state.activeFault);
        }
    }

    printMode(state.mode);
    printFault(state.activeFault);
}

int runSerialMission(const std::string& devicePath) {

    stopRequested = 0;

    if(std::signal(SIGINT, requestStop) == SIG_ERR || std::signal(SIGTERM, requestStop) == SIG_ERR) {
        std::cerr << "Could not install shutdown handlers\n";
        return 1;
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

    if (!port.openPort(devicePath)) {
        std::cerr << "Could not open the serial connection\n";
        return 1;
    }

    FlightState state{};
    const double accelerationLimit = 8.0;

    int consecutiveMalformed = 0;
    const int malformedLimit = 3;

    bool sensorReadsAvailable = true;

    while(!stopRequested) {

        if(!sensorReadsAvailable) {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - runStart).count();

            std::cout << "SAFE: Sensor reads stopped | Fault: " << faultName(state.activeFault) << '\n';

            logFile << "t_ms= " << elapsedMs << " Status | Mode: Safe | Sensor reads: stopped" << " Fault: " << faultName(state.activeFault) << '\n';

            logFile.flush();

            if(!logFile) {
                std::cerr << "Could not write sensor log\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(stopRequested) {
                break;
            }

            if(port.openPort(devicePath)) {
                sensorReadsAvailable = true;
                consecutiveMalformed = 0;

                std::cout << "serial port reopened; awaiting valid sensor data\n";

                logFile << "Serial port reopened | Mode: Safe | Fault: " << faultName(state.activeFault) << '\n';
                logFile.flush();

                if(!logFile) {
                    std::cerr << "Could not write sensor log\n";
                    return 1;
                }
            }
            continue;
        }

        std::string line;

        bool receivedLine = port.readLine(line);

        if(stopRequested) {
            break;
        }

        if(!receivedLine) {

            port.closePort();
            
            state = handleCommunicationLost(state);

            const auto faultElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - runStart).count();

            logFile << "t_ms= " << faultElapsedMs << " Serial read failed or timed out | Mode: Safe\n";

            logFile << "Fault: " << faultName(state.activeFault) << '\n';

            logFile.flush();

            if(!logFile) {
                std::cerr << "Could not write sensor log\n";
                return 1;
            }

            std::cerr << "Serial read failed or timed out\n";
            std::cerr << "Mode: Safe\n";
            if(state.activeFault == FaultReason::SensorCommunicationLost) {
                std::cerr << "Fault: SensorCommunicationLost\n";
            }
            else {
                std::cerr << "Previously recorded fault preserved\n";
            }
            sensorReadsAvailable = false;
            continue;
        }

        SensorReading reading{};

        if (!parseSensorMessage(line, reading)) {
            if(consecutiveMalformed < malformedLimit) {
                consecutiveMalformed++;
            }

            if (consecutiveMalformed >= malformedLimit &&
                state.mode != FlightMode::Safe) {
                state.mode = FlightMode::Safe;
                state.activeFault = FaultReason::MalformedSensorMessage;
            }

            const auto elapsedMs =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - runStart
                ).count();

            logFile << "t_ms= " << elapsedMs
                    << " Malformed message"
                    << " | Consecutive: " << consecutiveMalformed
                    << " | Mode: "
                    << (state.mode == FlightMode::Safe ? "Safe" : "Collecting")
                    << " | Fault: " << faultName(state.activeFault)
                    << '\n';

            logFile.flush();

            if (!logFile) {
                std::cerr << "Could not write sensor log\n";
                return 1;
            }

            std::cerr << "Skipping malformed message: " << line << '\n';
            continue;
        }

        consecutiveMalformed = 0;

        std::cout << "Parsed acceleration: "
                  << reading.accelX << ", "
                  << reading.accelY << ", "
                  << reading.accelZ << " m/s^2\n";

        state = calculateFlightState(state, reading, accelerationLimit);

        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - runStart).count();

        logFile << "t_ms= " << elapsedMs << " ACCEL " << reading.accelX << ' ' << reading.accelY << ' ' << reading.accelZ << " | Mode: " << (state.mode == FlightMode::Safe ? "Safe" : "Collecting") << " | Fault: " << faultName(state.activeFault) << '\n';

        logFile.flush();

        if(!logFile) {
            std::cerr << "Could not write sensor log\n";
            return 1;
        }

        std::cout << "Mode: " << (state.mode == FlightMode::Safe ? "Safe" : "Collecting") << '\n';
    }

    logFile << "Run stopped by request | Fault: " << faultName(state.activeFault) << '\n';
    logFile.flush();

    if(!logFile) {
        std::cerr << "Could not write shutdown log\n";
        return 1;
    }

    std::cout << "Serial mission stopped\n";
    return 0;
}

int main(int argc, char* argv[]) {
    
    std::cout << "Flight computer starting...\n";

    if(argc == 1) {
        runMission();
        return 0;
    }

    const std::string argument = argv[1];

    if(argument == "--test" && argc == 2) {
        return runModeTests();
    }

    if(argument == "--serial" && argc == 3) {
        return runSerialMission(argv[2]);
    }

    std::cerr << "Usage: flight [--test | --serial <device-path>]\n";
    return 2;
}