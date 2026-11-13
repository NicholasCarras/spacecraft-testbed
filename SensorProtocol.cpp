#include "SensorProtocol.h"
#include <sstream>

bool parseSensorMessage(const std::string& message, SensorReading& output) {

    std::istringstream input(message);
    std::string messageType;

    if(!(input >> messageType)) {
        return false;
    }
    if(messageType != "ACCEL") {
        return false;
    }

    SensorReading candidate{};

    if(!(input >> candidate.accelX >> candidate.accelY >> candidate.accelZ)) {
        return false;
    }

    input >> std::ws;

    if(!(input.eof())) {
        return false;
    }

    output = candidate;
    return true;
}