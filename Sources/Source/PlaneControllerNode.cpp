#include "PlaneControllerNode.h"
#include "FlightMode.h"
#include <cstring>
#include <span>
#include "Config.h"

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
    , std::unique_ptr<IFlightMode> highPosFlightMode
    , std::unique_ptr<IFlightMode> failSafeFlightMode)
    : m_isInitialized(false)
    , m_currentFlightMode(nullptr)
    , m_lowPosFlightMode(std::move(lowPosFlightMode))
    , m_midPosFlightMode(std::move(midPosFlightMode))
    , m_highPosFlightMode(std::move(highPosFlightMode))
    , m_failSafeFlightMode(std::move(failSafeFlightMode))
    , m_model(std::move(model))
{
    m_watchdogTimer = xTimerCreate("RcWatchdog",
        pdMS_TO_TICKS(Config::RC_LINK_TIMEOUT_MS),
        pdFALSE, this, &PlaneControllerNode::WatchdogCallback);

    m_outputPushTimer = xTimerCreate("FailsafeOutputPush",
        pdMS_TO_TICKS(Config::FAILSAFE_OUTPUT_PUSH_PERIOD_MS),
        pdTRUE, this, &PlaneControllerNode::OutputPushCallback);
}

void PlaneControllerNode::WatchdogCallback(TimerHandle_t timer)
{
    static_cast<PlaneControllerNode*>(pvTimerGetTimerID(timer))->OnLinkLost();
}

void PlaneControllerNode::OnLinkLost()
{
    m_currentFlightMode.store(m_failSafeFlightMode.get());
    const char* name = m_failSafeFlightMode->GetName();
    Publish(std::span<const char>(name, std::strlen(name)));
    xTimerStart(m_outputPushTimer, 0);
}

void PlaneControllerNode::OutputPushCallback(TimerHandle_t timer)
{
    static_cast<PlaneControllerNode*>(pvTimerGetTimerID(timer))->PushFailsafeOutputs();
}

void PlaneControllerNode::PushFailsafeOutputs()
{
    if (!m_isInitialized.load())
    {
        return;
    }
    auto* mode = m_currentFlightMode.load();
    auto controlCommands = mode->Translate({});
    m_model->Pitch(controlCommands.pitch);
    m_model->Roll(controlCommands.roll);
    m_model->Yaw(controlCommands.yaw);
    m_model->Thrust(static_cast<uint8_t>(controlCommands.thrust));
}

void PlaneControllerNode::Process(const RcPacket& packet)
{
    xTimerReset(m_watchdogTimer, 0);
    if (xTimerIsTimerActive(m_outputPushTimer))
    {
        xTimerStop(m_outputPushTimer, 0);
    }

    bool flightModeChanged = false;
    auto* currentFlightMode = m_currentFlightMode.load();
    switch (packet.flightMode)
    {
    case TriSwitchPosition::Low:
        if (currentFlightMode != m_lowPosFlightMode.get())
        {
            if (currentFlightMode != nullptr)
                currentFlightMode->OnExit();
            m_currentFlightMode.store(m_lowPosFlightMode.get());
            flightModeChanged = true;
        }
        break;
    case TriSwitchPosition::Mid:
        if (currentFlightMode != m_midPosFlightMode.get())
        {
            if (currentFlightMode != nullptr)
                currentFlightMode->OnExit();
            m_currentFlightMode.store(m_midPosFlightMode.get());
            flightModeChanged = true;
        }
        break;
    case TriSwitchPosition::High:
        if (currentFlightMode != m_highPosFlightMode.get())
        {
            if (currentFlightMode != nullptr)
                currentFlightMode->OnExit();
            m_currentFlightMode.store(m_highPosFlightMode.get());
            flightModeChanged = true;
        }
        break;

    default:
        break;
    }
    if (flightModeChanged)
    {
        currentFlightMode = m_currentFlightMode.load();
        const char* name = currentFlightMode->GetName();
        currentFlightMode->OnEnter();
        Publish(std::span<const char>(name, std::strlen(name)));
    }
    auto controlCommands = m_currentFlightMode.load()->Translate(FromRcPacket(packet));
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
