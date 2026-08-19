#include "ServoPwmControl.h"
#include <assert.h>

namespace
{
    uint32_t getTimerInputFreq(TIM_TypeDef* instance)
    {
        const bool onApb2 =
#ifdef TIM1
            instance == TIM1 ||
#endif
#ifdef TIM8
            instance == TIM8 ||
#endif
#ifdef TIM9
            instance == TIM9 ||
#endif
#ifdef TIM10
            instance == TIM10 ||
#endif
#ifdef TIM11
            instance == TIM11 ||
#endif
            false;

        const uint32_t pclk = onApb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

        // Timer kernel clock is 2x PCLK when the APB prescaler is >1, otherwise 1x.
        const uint32_t ppre = onApb2
            ? ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos)
            : ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);

        return (ppre < 4) ? pclk : pclk * 2;
    }

    uint32_t usToTicks(std::chrono::microseconds time,
                       uint32_t timerInputFreq,
                       uint32_t timerPrescaler)
    {
        const uint64_t numerator =
            static_cast<uint64_t>(time.count()) * timerInputFreq;
        return static_cast<uint32_t>(numerator / 1'000'000ULL / (timerPrescaler + 1));
    }
}

ServoPwmControl::ServoPwmControl(
    TIM_HandleTypeDef& timer
    , uint32_t channel
    , std::chrono::microseconds minPulseWidth
    , std::chrono::microseconds maxPulseWidth)
    : m_timer(timer)
    , m_channel(channel)
{
    const uint32_t timerClk = getTimerInputFreq(m_timer.Instance);
    const uint32_t psc = m_timer.Instance->PSC;
    m_minTicks = usToTicks(minPulseWidth, timerClk, psc);
    m_maxTicks = usToTicks(maxPulseWidth, timerClk, psc);

    [[maybe_unused]] HAL_StatusTypeDef s1 = HAL_TIM_PWM_Start(&m_timer, m_channel);
    assert(s1 == HAL_OK);

    SetDuty(0.5f);
}

ServoPwmControl::~ServoPwmControl()
{
    HAL_TIM_PWM_Stop(&m_timer, m_channel);
}

void ServoPwmControl::SetDuty(float duty)
{
    duty = std::clamp(duty, 0.0f, 1.0f);
    const uint32_t ticks =
        m_minTicks + static_cast<uint32_t>(duty * (m_maxTicks - m_minTicks) + 0.5f);
    __HAL_TIM_SET_COMPARE(&m_timer, m_channel, ticks);
}

void ServoPwmControl::SetAngle(float degrees, float range)
{
    SetDuty(degrees / range);
}
