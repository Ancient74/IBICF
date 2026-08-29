#pragma once
#include "FlightMode.h"
#include "CommonTask.h"
#include "KalmanLib/PidController.hpp"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"

class FailsafeFlightMode
    : public IFlightMode
    , public CommonTask<FailsafeFlightMode, 128, configMAX_PRIORITIES / 2 - 9>
    , public NodeInput<FailsafeFlightMode, Quaternion<float>>
{
    using Base = CommonTask<FailsafeFlightMode, 128, configMAX_PRIORITIES / 2 - 9>;
    using Base::Start;
    using Base::Stop;
public:
    FailsafeFlightMode(const char* taskName);

    void OnEnter() override;
    void OnExit() override;

    void Process(const Quaternion<float>& quat);

    PitchRollYawThrust Translate(const PitchRollYawThrust& pitchRollYaw) override;
    void RunTaskIteration();

    const char* GetName() const override;

private:
    std::atomic<Attitude> m_currentAtt;
    PidController m_rollPid;

    TickType_t m_failsafeEnteredTick = 0;

    std::atomic<int32_t> m_thrustCommand = 0;
    std::atomic<float> m_rollCommandDeg = 0.0f;
};
