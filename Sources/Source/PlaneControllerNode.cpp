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

PlaneControllerNode::PlaneControllerNode(PlaneModel& model
    , IFlightMode& disarmedFlightMode
    , IFlightMode& lowPosFlightMode
    , IFlightMode& midPosFlightMode
    , IFlightMode& highPosFlightMode
    , IFlightMode& failSafeFlightMode)
    : m_currentFlightMode(&disarmedFlightMode)
    , m_disarmedFlightMode(disarmedFlightMode)
    , m_lowPosFlightMode(lowPosFlightMode)
    , m_midPosFlightMode(midPosFlightMode)
    , m_highPosFlightMode(highPosFlightMode)
    , m_failSafeFlightMode(failSafeFlightMode)
    , m_model(model)
{
    m_disarmedFlightMode.OnEnter();

    m_watchdogTimer = xTimerCreateStatic("RcWatchdog",
        pdMS_TO_TICKS(Config::RC_LINK_TIMEOUT_MS),
        pdFALSE, this, &PlaneControllerNode::WatchdogCallback,
        &m_watchdogTimerBuffer);
    configASSERT(m_watchdogTimer != nullptr);

    m_outputPushTimer = xTimerCreateStatic("FailsafeOutputPush",
        pdMS_TO_TICKS(Config::FAILSAFE_OUTPUT_PUSH_PERIOD_MS),
        pdTRUE, this, &PlaneControllerNode::OutputPushCallback,
        &m_outputPushTimerBuffer);
    configASSERT(m_outputPushTimer != nullptr);
}

void PlaneControllerNode::WatchdogCallback(TimerHandle_t timer)
{
    static_cast<PlaneControllerNode*>(pvTimerGetTimerID(timer))->OnLinkLost();
}

void PlaneControllerNode::OnLinkLost()
{
    auto* previousMode = m_currentFlightMode.load();
    if (previousMode == &m_disarmedFlightMode)
        return;

    if (previousMode != nullptr)
        previousMode->OnExit();
    m_currentFlightMode.store(&m_failSafeFlightMode);
    m_failSafeFlightMode.OnEnter();

    const char* name = m_failSafeFlightMode.GetName();
    Publish(std::span<const char>(name, std::strlen(name)));
    xTimerStart(m_outputPushTimer, 0);
}

void PlaneControllerNode::OutputPushCallback(TimerHandle_t timer)
{
    static_cast<PlaneControllerNode*>(pvTimerGetTimerID(timer))->PushFailsafeOutputs();
}

void PlaneControllerNode::PushFailsafeOutputs()
{
    auto* mode = m_currentFlightMode.load();
    auto controlCommands = mode->Translate({});
    m_model.Pitch(controlCommands.pitch);
    m_model.Roll(controlCommands.roll);
    m_model.Yaw(controlCommands.yaw);
    m_model.Thrust(static_cast<uint8_t>(controlCommands.thrust));
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
    if (!packet.isArmed)
    {
        if (currentFlightMode != &m_disarmedFlightMode)
        {
            if (currentFlightMode != nullptr)
                currentFlightMode->OnExit();
            m_currentFlightMode.store(&m_disarmedFlightMode);
            flightModeChanged = true;
        }
    }
    else
    {
        switch (packet.flightMode)
        {
        case TriSwitchPosition::Low:
            if (currentFlightMode != &m_lowPosFlightMode)
            {
                if (currentFlightMode != nullptr)
                    currentFlightMode->OnExit();
                m_currentFlightMode.store(&m_lowPosFlightMode);
                flightModeChanged = true;
            }
            break;
        case TriSwitchPosition::Mid:
            if (currentFlightMode != &m_midPosFlightMode)
            {
                if (currentFlightMode != nullptr)
                    currentFlightMode->OnExit();
                m_currentFlightMode.store(&m_midPosFlightMode);
                flightModeChanged = true;
            }
            break;
        case TriSwitchPosition::High:
            if (currentFlightMode != &m_highPosFlightMode)
            {
                if (currentFlightMode != nullptr)
                    currentFlightMode->OnExit();
                m_currentFlightMode.store(&m_highPosFlightMode);
                flightModeChanged = true;
            }
            break;
        default:
            break;
        }
    }
    if (flightModeChanged)
    {
        currentFlightMode = m_currentFlightMode.load();
        const char* name = currentFlightMode->GetName();
        currentFlightMode->OnEnter();
        Publish(std::span<const char>(name, std::strlen(name)));
    }
    auto controlCommands = m_currentFlightMode.load()->Translate(FromRcPacket(packet));
    m_model.Pitch(controlCommands.pitch);
    m_model.Roll(controlCommands.roll);
    m_model.Yaw(controlCommands.yaw);
    m_model.Thrust(static_cast<uint8_t>(controlCommands.thrust));
}
