#pragma once
#include "FlightMode.h"
#include "TaskThread.h"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"
#include "KalmanLib/PidController.hpp"
#include "Config.h"

class CruiseFlightMode
    : public FlightModeBase
    , public NodeInput<CruiseFlightMode, Quaternion<float>>
    , public NodeInput<CruiseFlightMode, float>
{
public:
    CruiseFlightMode(const char* taskName);

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    PitchRollYawThrust Translate(const PitchRollYawThrust& pitchRollYaw) override;

    const char* GetName() const override;

    void Process(const Quaternion<float>& quat);
    void Process(float altitude);

    void RunTaskIteration();

protected:
    void OnEnterInternal() override;
    void OnExitInternal() override;

private:
    TaskThread<CruiseFlightMode, Config::Task::CRUISE_FLIGHT_MODE_STACK_SIZE, Config::Task::CRUISE_FLIGHT_MODE_PRIORITY> m_thread;

    std::atomic<float> m_currentRollRad = 0.0f;
    std::atomic<float> m_currentPitchRad = 0.0f;
    std::atomic<float> m_currentAlt;

    float m_desiredAlt;
    PidController m_rollPid;
    PidController m_altPid;

    std::atomic<float> m_rollCommandDeg = 0.0f;
    std::atomic<float> m_pitchCommandDeg = Config::CRUISE_TRIM_PITCH_DEG;
};
