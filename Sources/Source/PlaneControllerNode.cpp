#include "PlaneControllerNode.h"
#include "FlightMode.h"
#include <cstring>
#include <span>

namespace
{
    PitchRollYawThrust FromRcPacket(const RcPacket& packet)
    {
        return { .pitch = packet.pitch, .roll = packet.roll, .yaw = packet.yaw, .thrust = packet.thrust };
    }
}

PlaneControllerNode::PlaneControllerNode(std::shared_ptr<PlaneModel> model
    , std::unique_ptr<IFlightMode> lowPosFlightMode
    , std::unique_ptr<IFlightMode> midPosFlightMode
    , std::unique_ptr<IFlightMode> highPosFlightMode)
    : m_isInitialized(false)
    , m_currentFlightMode(nullptr)
    , m_lowPosFlightMode(std::move(lowPosFlightMode))
    , m_midPosFlightMode(std::move(midPosFlightMode))
    , m_highPosFlightMode(std::move(highPosFlightMode))
    , m_model(std::move(model))
{
}

void PlaneControllerNode::Process(const RcPacket& packet)
{
    bool flightModeChanged = false;
    switch (packet.flightMode)
    {
    case TriSwitchPosition::Low:
        if (m_currentFlightMode != m_lowPosFlightMode.get())
        {
            m_currentFlightMode->OnExit();
            m_currentFlightMode = m_lowPosFlightMode.get();
            flightModeChanged = true;
        }
        break;
    case TriSwitchPosition::Mid:
        if (m_currentFlightMode != m_midPosFlightMode.get())
        {
            m_currentFlightMode->OnExit();
            m_currentFlightMode = m_midPosFlightMode.get();
            flightModeChanged = true;
        }
        break;
    case TriSwitchPosition::High:
        if (m_currentFlightMode != m_highPosFlightMode.get())
        {
            m_currentFlightMode->OnExit();
            m_currentFlightMode = m_highPosFlightMode.get();
            flightModeChanged = true;
        }
        break;

    default:
        break;
    }
    if (flightModeChanged)
    {
        const char* name = m_currentFlightMode->GetName();
        m_currentFlightMode->OnEnter();
        Publish(std::span<const char>(name, std::strlen(name)));
    }
    auto controlCommands = m_currentFlightMode->Translate(FromRcPacket(packet));
    m_model->Pitch(controlCommands.pitch);
    m_model->Roll(controlCommands.roll);
    m_model->Yaw(controlCommands.yaw);

    if (!m_isInitialized && packet.isArmed)
    {
        m_model->Thrust(0);
        return;
    }
    m_isInitialized = true;
    uint8_t allowedThrust = packet.isArmed ? static_cast<uint8_t>(controlCommands.thrust) : 0;
    m_model->Thrust(allowedThrust);
}
