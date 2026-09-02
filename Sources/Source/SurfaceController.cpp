#include "SurfaceController.h"

SurfaceController::SurfaceController(ServoPwmControl& pwmControl)
    : m_pwmControl(pwmControl)
{}
void SurfaceController::SetSurface(int16_t deviation)
{
    auto angle = 90 + deviation;

    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    m_pwmControl.SetAngle(angle);
}
