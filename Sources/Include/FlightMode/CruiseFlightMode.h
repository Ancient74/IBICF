#pragma once
#include "CommonTask.h"
#include "FlightMode.h"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"
#include "KalmanLib/PidController.hpp"
#include "Config.h"

class CruiseFlightMode
    : public IFlightMode
    , public CommonTask<CruiseFlightMode, 128, configMAX_PRIORITIES / 2 - 9>
    , public NodeInput<CruiseFlightMode, Quaternion<float>>
    , public NodeInput<CruiseFlightMode, float>
{
    using Base = CommonTask<CruiseFlightMode, 128, configMAX_PRIORITIES / 2 - 9>;
    using Base::Start;
    using Base::Stop;
public:
    CruiseFlightMode(const char* taskName);

    void OnEnter() override;
    void OnExit() override;

    PitchRollYawThrust Translate(const PitchRollYawThrust& pitchRollYaw) override;

    const char* GetName() const override;

    void Process(const Quaternion<float>& quat);
    void Process(float altitude);

    void RunTaskIteration();

private:
    std::atomic<float> m_currentRollRad = 0.0f;
    std::atomic<float> m_currentPitchRad = 0.0f;
    std::atomic<float> m_currentAlt;

    float m_desiredAlt;
    PidController m_rollPid;
    PidController m_altPid;

    std::atomic<float> m_rollCommandDeg = 0.0f;
    std::atomic<float> m_pitchCommandDeg = Config::CRUISE_TRIM_PITCH_DEG;
};
