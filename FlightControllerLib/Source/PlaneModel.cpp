#include "FlightControllerLib/PlaneModel.hpp"

PlaneModel::PlaneModel(
    std::unique_ptr<ISurfaceController> leftAileron
  , std::unique_ptr<ISurfaceController> rightAileron
  , std::unique_ptr<ISurfaceController> leftRuddervator
  , std::unique_ptr<ISurfaceController> rightRuddervator
  , std::unique_ptr<IThrustController> thrust)
  : m_leftAileron(std::move(leftAileron))
  , m_rightAileron(std::move(rightAileron))
  , m_leftRuddervator(std::move(leftRuddervator))
  , m_rightRuddervator(std::move(rightRuddervator))
  , m_thrust(std::move(thrust))
{}

void PlaneModel::Roll(int16_t deviation)
{
    m_leftAileron->SetSurface(deviation);
    m_rightAileron->SetSurface(-deviation);
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
    m_leftRuddervator->SetSurface(m_pitchCommand + m_yawCommand);
    m_rightRuddervator->SetSurface(m_pitchCommand - m_yawCommand);
}

void PlaneModel::Thrust(uint16_t thrust)
{
    m_thrust->SetThrust(thrust > 100 ? 100 : thrust);
}
