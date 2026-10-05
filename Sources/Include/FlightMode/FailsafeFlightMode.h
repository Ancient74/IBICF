#pragma once
#include "FlightMode.h"
#include "TaskThread.h"
#include "Config.h"
#include "KalmanLib/PidController.hpp"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"

class FailsafeFlightMode
    : public FlightModeBase
    , public NodeInput<FailsafeFlightMode, Quaternion<float>>
{
public:
    FailsafeFlightMode(const char* taskName);

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void Process(const Quaternion<float>& quat);

    PitchRollYawThrust Translate(const PitchRollYawThrust& pitchRollYaw) override;
    void RunTaskIteration();

    const char* GetName() const override;

protected:
    void OnEnterInternal() override;
    void OnExitInternal() override;

private:
    TaskThread<FailsafeFlightMode, Config::Task::FAILSAFE_FLIGHT_MODE_STACK_SIZE, Config::Task::FAILSAFE_FLIGHT_MODE_PRIORITY> m_thread;

    std::atomic<float> m_currentRollRad = 0.0f;
    std::atomic<float> m_currentPitchRad = 0.0f;
    PidController m_rollPid;

    TickType_t m_failsafeEnteredTick = 0;

    std::atomic<int32_t> m_thrustCommand = 0;
    std::atomic<float> m_rollCommandDeg = 0.0f;
};
