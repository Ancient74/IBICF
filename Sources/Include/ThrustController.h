#pragma once
#include "FlightControllerLib/PlaneModel.hpp"
#include "main.h"
#include <stdint.h>

class ThrustController : public IThrustController
{
public:
    ThrustController(TIM_HandleTypeDef& timer, uint32_t channel);
    ~ThrustController();

    void SetThrust(uint8_t value) override;

private:
    TIM_HandleTypeDef& m_timer;
    uint32_t m_channel;
};
