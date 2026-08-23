#include "HalfDuplexSoftwareUart.h"
#include "projdefs.h"
#include <stm32f4xx_hal_tim.h>

namespace
{
    constexpr uint16_t QUEUE_SIZE = 256;

    uint32_t GetApb2TimerClockFreq()
    {
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        return (ppre2 & 0x4u) ? pclk2 * 2 : pclk2;
    }
}

HalfDuplexSoftwareUart::HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, uint32_t baudRate)
    : CommonTask<HalfDuplexSoftwareUart, 512, configMAX_PRIORITIES / 2 - 3>("HalfDuplexUart", 0)
    , m_pin(pin)
    , m_port(port)
    , m_timer(timer)
    , m_ticksPerBit(GetApb2TimerClockFreq() / (m_timer.Instance->PSC + 1) / baudRate)
{
    m_writeQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
    HAL_TIM_Base_Start(&m_timer);
}

void HalfDuplexSoftwareUart::RunTaskIteration()
{
    uint8_t byte;
    if (xQueueReceive(m_writeQueue, &byte, 300) != pdTRUE)
        return;

    HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
    DelayBitPeriod();

    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(m_port, m_pin,  (byte & (1u << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        DelayBitPeriod();
    }
    HAL_GPIO_WritePin(m_port, m_pin, GPIO_PinState::GPIO_PIN_SET);
    DelayBitPeriod();
}

bool HalfDuplexSoftwareUart::Write(std::span<const uint8_t> data)
{
    for(uint16_t i = 0; i < data.size(); i++)
        if (xQueueSend(m_writeQueue, &data[i], 0) != pdTRUE)
            return false;
    return true;
}

void HalfDuplexSoftwareUart::DelayBitPeriod()
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&m_timer);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&m_timer) - start) < m_ticksPerBit) {}
}
