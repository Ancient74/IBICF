#pragma once
#include "main.h"
#include <stdint.h>
#include <chrono>

class ServoPwmControl
{
public:
    ServoPwmControl(TIM_HandleTypeDef& timer, uint32_t channel
        , std::chrono::microseconds minPulseWidth
        , std::chrono::microseconds maxPulseWidth);
    ~ServoPwmControl();

    void SetAngle(float degrees, float range=180.0f);

private:
    void SetDuty(float duty);

    TIM_HandleTypeDef& m_timer;
    uint32_t m_channel;
    uint32_t m_minTicks;
    uint32_t m_maxTicks;
};
