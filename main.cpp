#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include "FlightTypes.h"
#include "FlightLogic.h"
#include "FlightTests.h"

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

void printFault(FaultReason reason) {

    if(reason == FaultReason::None) {
        std::cout << "Fault: NONE\n";
    }
    else if(reason == FaultReason::InvalidReading) {
        std::cout <<"Fault: INVALID READING\n";
    }
    else {
        std::cout << "Fault: ACCELERATION ABOVE LIMIT\n";
    }
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

int main(int argc, char* argv[]) {
    
    std::cout << "Flight computer starting...\n";

    if(argc > 2) {
        std::cerr << "Unkown argument\n";
        return 2;
    }

    if(argc > 1) {
        std::string argument = argv[1];
        if(argument == "--test") {
            return runModeTests();
        }
        else {
            std::cerr << "Unkown argument: " << argument << "\n";
            return 2;
        }
    }
    runMission();
    return 0;
}