#include "FlightMode.h"

PitchRollYawThrust ManualFlightMode::Translate(const PitchRollYawThrust& command)
{
    return command;
}

const char* ManualFlightMode::GetName() const
{
    return "MANUAL";
}
