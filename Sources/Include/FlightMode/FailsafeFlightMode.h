#pragma once
#include "FlightMode.h"
#include "TaskThread.h"
#include "KalmanLib/PidController.hpp"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"

class FailsafeFlightMode
    : public IFlightMode
    , public NodeInput<FailsafeFlightMode, Quaternion<float>>
{
public:
    FailsafeFlightMode(const char* taskName);

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void OnEnter() override;
    void OnExit() override;

    void Process(const Quaternion<float>& quat);

    PitchRollYawThrust Translate(const PitchRollYawThrust& pitchRollYaw) override;
    void RunTaskIteration();

    const char* GetName() const override;

private:
    TaskThread<FailsafeFlightMode, 128, configMAX_PRIORITIES / 2 - 9> m_thread;

    std::atomic<float> m_currentRollRad = 0.0f;
    std::atomic<float> m_currentPitchRad = 0.0f;
    PidController m_rollPid;

    TickType_t m_failsafeEnteredTick = 0;

    std::atomic<int32_t> m_thrustCommand = 0;
    std::atomic<float> m_rollCommandDeg = 0.0f;
};
