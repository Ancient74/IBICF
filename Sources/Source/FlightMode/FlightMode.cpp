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

PitchRollYawThrust DisarmedFlightMode::Translate(const PitchRollYawThrust& command)
{
    auto disarmedCommand = command;
    disarmedCommand.thrust = 0;
    return disarmedCommand;
}

const char* DisarmedFlightMode::GetName() const
{
    return "DISARMED";
}

void DisarmedFlightMode::OnEnterInternal()
{
    LogInfo(MODULE, "Motor is disarmed");
}

void DisarmedFlightMode::OnExitInternal()
{
    LogInfo(MODULE, "Motor is armed");
}
