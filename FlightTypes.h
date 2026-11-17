#pragma once

struct SensorReading {

    double accelX;
    double accelY;
    double accelZ;
};

enum class FlightMode {

    Collecting,
    Safe
};

enum class FaultReason {

    None,
    InvalidReading,
    AccelerationAboveLimit,
    SensorCommunicationLost,
    MalformedSensorMessage
};

struct FlightState {

    FlightMode mode = FlightMode::Collecting;
    FaultReason activeFault = FaultReason::None;
};