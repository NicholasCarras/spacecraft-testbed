#include "FlightTests.h"
#include "FlightLogic.h"
#include "SensorProtocol.h"

#include <iostream>
#include <limits>
#include <string>

bool checkState(const FlightState& actual, const FlightState& expected, const std::string& testName) {

    if(actual.mode == expected.mode && actual.activeFault == expected.activeFault) {
        std::cout << "PASS: " << testName << "\n";
        return true;
    }
    else {
        std::cout << "FAIL: " << testName << "\n";
        return false;
    }
}

int runModeTests() {

    const double accelerationLimit = 0.35;

    FlightMode mode{FlightMode::Collecting};

    SensorReading normalReading{0.1, 0.0, 9.81};
    SensorReading highReading{0.5, 0.0, 9.81};
    SensorReading boundaryReading{accelerationLimit, 0.0, 9.81};
    SensorReading invalidReading{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.91};
    SensorReading invalidYReading{0.0, std::numeric_limits<double>::infinity(), 9.81};
    SensorReading invalidZReading{0.0, 0.0, -std::numeric_limits<double>::infinity()};

    int failedTests = 0;

    FlightState collectingState{};
    FlightState accelerationFaultState{FlightMode::Safe, FaultReason::AccelerationAboveLimit};
    FlightState invalidFaultState{FlightMode::Safe, FaultReason::InvalidReading};
    FlightState normalActualState = calculateFlightState(collectingState, normalReading, accelerationLimit);
    FlightState safeActualState = calculateFlightState(accelerationFaultState, normalReading, accelerationLimit);
    FlightState highActualState = calculateFlightState(collectingState, highReading, accelerationLimit);
    FlightState boundaryActualState = calculateFlightState(collectingState, boundaryReading, accelerationLimit);
    FlightState invalidActualState = calculateFlightState(collectingState, invalidReading, accelerationLimit);
    FlightState invalidYActualState = calculateFlightState(collectingState, invalidYReading, accelerationLimit);
    FlightState invalidZActualState = calculateFlightState(collectingState, invalidZReading, accelerationLimit);

    if(!checkState(normalActualState, collectingState, "Normal reading keeps collecting")) {
        failedTests++;
    }
    if(!checkState(safeActualState, accelerationFaultState, "Safe mode stays safe")) {
        failedTests++;
    }
    if(!checkState(highActualState, accelerationFaultState, "High reading enters safe mode")) {
        failedTests++;
    }
    if(!checkState(boundaryActualState, collectingState, "Reading at limit keeps collecting")) {
        failedTests++;
    }
    if(!checkState(invalidActualState, invalidFaultState, "Invalid reading enters safe mode")) {
        failedTests++;
    }
    if(!checkState(invalidYActualState, invalidFaultState, "Invalid reading enters safe mode")) {
        failedTests++;
    }
    if(!checkState(invalidZActualState, invalidFaultState, "Invalid reading enters safe mode")) {
        failedTests++;
    }

    SensorReading parsedReading{};

    bool parsed = parseSensorMessage("ACCEL 0.1 0.0 9.81", parsedReading);

    std::cout << "Parsed " << parsed << "\n";
    std::cout << "X: " << parsedReading.accelX << "\n";
    std::cout << "Y: " << parsedReading.accelY << "\n";
    std::cout << "Z: " << parsedReading.accelZ << "\n";

    std::cout << "Failed Tests: " << failedTests << "\n";

    {
        FlightState initialState{};

        FlightState actual = handleCommunicationLost(initialState);

        FlightState expected{FlightMode::Safe, FaultReason::SensorCommunicationLost};

        if(!checkState(actual, expected, "Communication loss enters Safe")) {
            failedTests++;
        }
    }

    {
        FlightState initialState{FlightMode::Safe, FaultReason::AccelerationAboveLimit};

        FlightState actual = handleCommunicationLost(initialState);

        if(!checkState(actual, initialState, "Communication loss preserves existing fault")) {
            failedTests++;
        }
    }

    if(failedTests == 0) {
        return 0;
    }
    else {
        return 1;
    }
}