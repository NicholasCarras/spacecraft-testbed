#pragma once

#include "FlightTypes.h"

FlightState calculateFlightState(const FlightState& currentState, const SensorReading& reading, double limit);