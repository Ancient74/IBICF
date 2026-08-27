#include "FlightMode.h"

PitchRollYaw ManualFlightMode::Translate(const PitchRollYaw& command)
{
    return command;
}

const char* ManualFlightMode::GetName() const
{
    return "MANUAL";
}
