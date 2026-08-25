#pragma once
#include <stdint.h>
#include "CommonTask.h"
#include "PubSubLib/Node.hpp"
#include <span>
#include <stm32f4xx_hal.h>

class HalfDuplexSoftwareUart
    : public CommonTask<HalfDuplexSoftwareUart, 512, configMAX_PRIORITIES / 2 - 10>
    , public NodeOutput<HalfDuplexSoftwareUart, uint8_t, 1>
    , public NodeInput<HalfDuplexSoftwareUart, std::span<const uint8_t>>
{
public:
    // timer is apb2 only
    HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, uint32_t baudRate = 9600);
    ~HalfDuplexSoftwareUart();

    void RunTaskIteration();

    bool Process(std::span<const uint8_t> data);

    static HalfDuplexSoftwareUart* GetInstance();
    uint16_t GetPin() const;
    void HandleExti();

private:
    static HalfDuplexSoftwareUart* s_instance;

    void DelayPeriod(uint16_t period);
    void ArmExti();

    QueueHandle_t m_writeQueue;
    QueueHandle_t m_readingSemaphore;

    uint16_t m_pin;
    GPIO_TypeDef* m_port;
    TIM_HandleTypeDef& m_timer;
    uint32_t m_ticksPerBit;
    uint32_t m_tickPerOversampling;
};
