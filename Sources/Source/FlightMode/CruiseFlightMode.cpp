#include "CruiseFlightMode.h"
#include <projdefs.h>

CruiseFlightMode::CruiseFlightMode(const char* taskName)
    : CruiseFlightMode::Base(taskName, pdMS_TO_TICKS(1000 / Config::CRUISE_FLIGHT_MODE_REFRESH_RATE_HZ))
    , m_rollPid(
        Config::ROLL_Kp,
        Config::ROLL_Ki,
        Config::ROLL_Kd,
        Config::ROLL_OutMin,
        Config::ROLL_OutMax,
        Config::ROLL_IntMin,
        Config::ROLL_IntMax)
    , m_altPid(
        Config::ALT_Kp,
        Config::ALT_Ki,
        Config::ALT_Kd,
        Config::ALT_OutMin,
        Config::ALT_OutMax,
        Config::ALT_IntMin,
        Config::ALT_IntMax)
{
}

void CruiseFlightMode::OnEnter()
{
    m_desiredAlt = m_currentAlt;
    Start();
}

void CruiseFlightMode::OnExit()
{
    Stop();
}

PitchRollYawThrust CruiseFlightMode::Translate(const PitchRollYawThrust& pitchRollYaw)
{
    PitchRollYawThrust out;
    out.roll = static_cast<int32_t>(std::roundf(m_rollCommandDeg.load()));
    out.pitch = static_cast<int32_t>(std::roundf(m_pitchCommandDeg.load()));
    out.yaw = 0;
    out.thrust = pitchRollYaw.thrust;
    return out;
}

const char* CruiseFlightMode::GetName() const
{
    return "CRUISE";
}

void CruiseFlightMode::Process(const Quaternion<float>& quat)
{
    m_currentAtt = quat.ToAttitude();
}

void CruiseFlightMode::Process(float altitude)
{
    m_currentAlt = altitude;
}

void CruiseFlightMode::RunTaskIteration()
{
    auto att = m_currentAtt.load();
    constexpr float dt = 1.0f / static_cast<float>(Config::CRUISE_FLIGHT_MODE_REFRESH_RATE_HZ);
    constexpr float RadToDeg = 180.0f / static_cast<float>(M_PI);

    const float rollErrorDeg = -att.rollRad * RadToDeg;
    const float rollCommandDeg = m_rollPid.Update(rollErrorDeg, dt);

    const float altErrorM = m_desiredAlt - m_currentAlt;
    const float pitchCorrectionDeg = m_altPid.Update(altErrorM, dt);
    const float pitchCommandDeg = Config::CRUISE_TRIM_PITCH_DEG + pitchCorrectionDeg;

    m_rollCommandDeg.store(rollCommandDeg);
    m_pitchCommandDeg.store(pitchCommandDeg);
}
