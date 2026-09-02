#pragma once
#include "FlightControllerLib/PlaneModel.hpp"
#include "ServoPwmControl.h"

class SurfaceController : public ISurfaceController
{
public:
    explicit SurfaceController(ServoPwmControl& pwmControl);

    void SetSurface(int16_t deviation) override;

private:
    ServoPwmControl& m_pwmControl;
};
