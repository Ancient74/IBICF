#pragma once
#include "FlightControllerLib/PlaneModel.hpp"
#include "ServoPwmControl.h"

class SurfaceController : public ISurfaceController
{
public:
    SurfaceController(std::unique_ptr<ServoPwmControl> pwmControl);

    void SetSurface(int16_t deviation) override;

private:
    std::unique_ptr<ServoPwmControl> m_pwmControl;
};
