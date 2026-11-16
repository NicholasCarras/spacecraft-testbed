#include "FlightLogic.h"
#include <cmath>

bool isAccelerationAboveLimit(const SensorReading& reading, double limit) {

    if(reading.accelX > limit) {
        return true;
    }

    return false;
}

bool isReadingValid(const SensorReading& reading) {

    if(std::isfinite(reading.accelX) && std::isfinite(reading.accelY) && std::isfinite(reading.accelZ)) {
        return true;
    }
    return false;
}

FaultReason detectFault(const SensorReading& reading, double limit) {

    if(!isReadingValid(reading)) {
        return FaultReason::InvalidReading;
    }
    else if(isAccelerationAboveLimit(reading, limit)) {
        return FaultReason::AccelerationAboveLimit;
    }
    else {
        return FaultReason::None;
    }
}

FlightState calculateFlightState(const FlightState& currentState, const SensorReading& reading, double limit) {

    if(currentState.mode == FlightMode::Safe) {
        return currentState;
    }
    else {
        FlightState nextState = currentState;
        nextState.activeFault = detectFault(reading, limit);
        if(nextState.activeFault != FaultReason::None) {
            nextState.mode = FlightMode::Safe;
        }
        return nextState;
    }
}

FlightState handleCommunicationLost(const FlightState& currentState) {

    if(currentState.mode == FlightMode::Safe) {
        return currentState;
    }
    
    FlightState nextState = currentState;
    nextState.mode = FlightMode::Safe;
    nextState.activeFault = FaultReason::SensorCommunicationLost;

    return nextState;
}