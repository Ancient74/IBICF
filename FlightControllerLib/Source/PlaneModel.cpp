#include "FlightControllerLib/PlaneModel.hpp"

PlaneModel::PlaneModel(
    ISurfaceController& leftAileron
  , ISurfaceController& rightAileron
  , ISurfaceController& leftRuddervator
  , ISurfaceController& rightRuddervator
  , IThrustController& thrust)
  : m_leftAileron(leftAileron)
  , m_rightAileron(rightAileron)
  , m_leftRuddervator(leftRuddervator)
  , m_rightRuddervator(rightRuddervator)
  , m_thrust(thrust)
{}

void PlaneModel::Roll(int16_t deviation)
{
    m_leftAileron.SetSurface(deviation);
    m_rightAileron.SetSurface(-deviation);
}

void PlaneModel::Pitch(int16_t deviation)
{
    m_pitchCommand = deviation;
    UpdateRuddervators();
}

void PlaneModel::Yaw(int16_t deviation)
{
    m_yawCommand = deviation;
    UpdateRuddervators();
}

void PlaneModel::UpdateRuddervators()
{
    m_leftRuddervator.SetSurface(m_pitchCommand + m_yawCommand);
    m_rightRuddervator.SetSurface(m_pitchCommand - m_yawCommand);
}

void PlaneModel::Thrust(uint16_t thrust)
{
    m_thrust.SetThrust(thrust > 100 ? 100 : thrust);
}
