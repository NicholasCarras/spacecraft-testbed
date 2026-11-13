#pragma once

#include "FlightTypes.h"
#include <string>

bool parseSensorMessage(const std::string& message, SensorReading& output);