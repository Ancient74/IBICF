#include "ThrustController.h"

ThrustController::ThrustController(TIM_HandleTypeDef& timer, uint32_t channel)
    : m_timer(timer)
    , m_channel(channel)
{
    HAL_TIM_PWM_Start(&m_timer, m_channel);
}

ThrustController::~ThrustController()
{
    HAL_TIM_PWM_Stop(&m_timer, m_channel);
}

void ThrustController::SetThrust(uint8_t value)
{
    __HAL_TIM_SET_COMPARE(&m_timer, m_channel, value * m_timer.Instance->ARR / 100);
}
