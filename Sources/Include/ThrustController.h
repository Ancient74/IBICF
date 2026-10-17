#pragma once
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"
#include "main.h"
#include <stdint.h>

class ThrustController
    : public IThrustController
    , public NodeOutput<ThrustController, uint8_t, 1>
{
public:
    ThrustController(TIM_HandleTypeDef& timer, uint32_t channel);
    ~ThrustController();

    void PublishCurrentThrust();

    void SetThrust(uint8_t value) override;

private:
    TIM_HandleTypeDef& m_timer;
    uint32_t m_channel;
};
