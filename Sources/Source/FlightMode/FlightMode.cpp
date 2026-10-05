#include "FlightMode.h"
#include "Log.h"

namespace
{
    constexpr const char* MODULE = "FlightMode";
}

void FlightModeBase::OnEnter()
{
    LogInfo(MODULE, "Entering %s flight mode", GetName());
    OnEnterInternal();
}

void FlightModeBase::OnExit()
{
    LogInfo(MODULE, "Exiting %s flight mode", GetName());
    OnExitInternal();
}

void FlightModeBase::OnEnterInternal()
{
}

void FlightModeBase::OnExitInternal()
{
}

PitchRollYawThrust ManualFlightMode::Translate(const PitchRollYawThrust& command)
{
    return command;
}

const char* ManualFlightMode::GetName() const
{
    return "MANUAL";
}
