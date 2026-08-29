#include "FailsafeFlightMode.h"
#include <math.h>
#include "Config.h"

FailsafeFlightMode::FailsafeFlightMode(const char* taskName)
    : FailsafeFlightMode::Base(taskName, pdMS_TO_TICKS(1000 / Config::CRUISE_FLIGHT_MODE_REFRESH_RATE_HZ))
    , m_rollPid(
        Config::ROLL_Kp,
        Config::ROLL_Ki,
        Config::ROLL_Kd,
        Config::ROLL_OutMin,
        Config::ROLL_OutMax,
        Config::ROLL_IntMin,
        Config::ROLL_IntMax)
{
}

PitchRollYawThrust FailsafeFlightMode::Translate(const PitchRollYawThrust& pitchRollYaw)
{
    PitchRollYawThrust out;
    out.roll = static_cast<int32_t>(std::roundf(m_rollCommandDeg.load()));
    out.pitch = static_cast<int32_t>(Config::CRUISE_TRIM_PITCH_DEG);
    out.yaw = 0;
    out.thrust = 0;
    return out;
}

void FailsafeFlightMode::OnEnter()
{
    m_rollPid.Reset();
    m_failsafeEnteredTick = xTaskGetTickCount();
    Start();
}

void FailsafeFlightMode::OnExit()
{
    Stop();
}

const char* FailsafeFlightMode::GetName() const
{
    return "FAILSAFE";
}

void FailsafeFlightMode::Process(const Quaternion<float>& quat)
{
    m_currentAtt = quat.ToAttitude();
}

void FailsafeFlightMode::RunTaskIteration()
{
    auto att = m_currentAtt.load();
    constexpr float dt = 1.0f / static_cast<float>(Config::CRUISE_FLIGHT_MODE_REFRESH_RATE_HZ);
    constexpr float RadToDeg = 180.0f / static_cast<float>(M_PI);

    const float rollErrorDeg = -att.rollRad * RadToDeg;
    const float rollCommandDeg = m_rollPid.Update(rollErrorDeg, dt);
    m_rollCommandDeg.store(rollCommandDeg);

    const TickType_t elapsed = xTaskGetTickCount() - m_failsafeEnteredTick;
    const bool withinGracePeriod = elapsed < pdMS_TO_TICKS(Config::FAILSAFE_LINK_TIMEOUT_MS);
    m_thrustCommand.store(withinGracePeriod ? Config::FAILSAFE_HOLD_THRUST : 0);
}
