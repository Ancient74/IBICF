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

Vector3f PlaneModel::GetUpVector() const
{
    Vector3f upVector{};
    upVector(0, 0) = 0.0f;
    upVector(1, 0) = 1.0f;
    upVector(2, 0) = 0.0f;
    return upVector;
}

Vector3f PlaneModel::GetRightVector() const
{
    Vector3f rightVector{};
    rightVector(0, 0) = 0.0f;
    rightVector(1, 0) = 0.0f;
    rightVector(2, 0) = 1.0f;
    return rightVector;
}
