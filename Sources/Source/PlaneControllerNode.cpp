#include "PlaneControllerNode.h"
#include "FlightMode.h"
#include <cstring>
#include <span>
#include "Config.h"

namespace
{
    class MutexLock
    {
    public:
        explicit MutexLock(SemaphoreHandle_t mutex)
            : m_mutex(mutex)
        {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
        }

        ~MutexLock()
        {
            xSemaphoreGive(m_mutex);
        }

        MutexLock(const MutexLock&) = delete;
        MutexLock& operator=(const MutexLock&) = delete;

    private:
        SemaphoreHandle_t m_mutex;
    };

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
    m_modeMutex = xSemaphoreCreateMutexStatic(&m_modeMutexBuffer);
    configASSERT(m_modeMutex != nullptr);
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
    MutexLock lock(m_modeMutex);
    if (xTaskGetTickCount() - m_lastRcPacketTick.load(std::memory_order_relaxed)
        < pdMS_TO_TICKS(Config::RC_LINK_TIMEOUT_MS))
        return;

    auto* previousMode = m_currentFlightMode;
    if (previousMode == &m_disarmedFlightMode || previousMode == &m_failSafeFlightMode)
        return;

    previousMode->OnExit();
    m_currentFlightMode = &m_failSafeFlightMode;
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
    MutexLock lock(m_modeMutex);
    if (m_currentFlightMode != &m_failSafeFlightMode)
        return;

    auto controlCommands = m_currentFlightMode->Translate({});
    m_model.Pitch(controlCommands.pitch);
    m_model.Roll(controlCommands.roll);
    m_model.Yaw(controlCommands.yaw);
    m_model.Thrust(static_cast<uint8_t>(controlCommands.thrust));
}

void PlaneControllerNode::Process(const RcPacket& packet)
{
    m_lastRcPacketTick.store(xTaskGetTickCount(), std::memory_order_relaxed);
    MutexLock lock(m_modeMutex);
    xTimerReset(m_watchdogTimer, 0);
    if (xTimerIsTimerActive(m_outputPushTimer))
    {
        xTimerStop(m_outputPushTimer, 0);
    }
    bool flightModeChanged = false;
    auto* currentFlightMode = m_currentFlightMode;
    if (!packet.isArmed)
    {
        if (currentFlightMode != &m_disarmedFlightMode)
        {
            currentFlightMode->OnExit();
            m_currentFlightMode = &m_disarmedFlightMode;
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
                currentFlightMode->OnExit();
                m_currentFlightMode = &m_lowPosFlightMode;
                flightModeChanged = true;
            }
            break;
        case TriSwitchPosition::Mid:
            if (currentFlightMode != &m_midPosFlightMode)
            {
                currentFlightMode->OnExit();
                m_currentFlightMode = &m_midPosFlightMode;
                flightModeChanged = true;
            }
            break;
        case TriSwitchPosition::High:
            if (currentFlightMode != &m_highPosFlightMode)
            {
                currentFlightMode->OnExit();
                m_currentFlightMode = &m_highPosFlightMode;
                flightModeChanged = true;
            }
            break;
        default:
            break;
        }
    }
    if (flightModeChanged)
    {
        currentFlightMode = m_currentFlightMode;
        const char* name = currentFlightMode->GetName();
        currentFlightMode->OnEnter();
        Publish(std::span<const char>(name, std::strlen(name)));
    }
    auto controlCommands = m_currentFlightMode->Translate(FromRcPacket(packet));
    m_model.Pitch(controlCommands.pitch);
    m_model.Roll(controlCommands.roll);
    m_model.Yaw(controlCommands.yaw);
    m_model.Thrust(static_cast<uint8_t>(controlCommands.thrust));
}

void PlaneControllerNode::PublishFlightModeName()
{
    MutexLock lock(m_modeMutex);
    const char* name = m_currentFlightMode->GetName();
    Publish(std::span<const char>(name, std::strlen(name)));
}
