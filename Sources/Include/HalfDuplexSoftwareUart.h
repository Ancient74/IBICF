#pragma once
#include <stdint.h>
#include "TaskThread.h"
#include "FreeRTOS.h"
#include "PubSubLib/Node.hpp"
#include "stream_buffer.h"
#include <span>
#include <stm32f4xx_hal.h>
#include "Config.h"

class HalfDuplexSoftwareUart
    : public NodeOutput<HalfDuplexSoftwareUart, uint8_t, 1>
    , public NodeInput<HalfDuplexSoftwareUart, std::span<const uint8_t>>
{
public:
    // timer is apb2 only
    HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, uint32_t baudRate = 9600);
    ~HalfDuplexSoftwareUart();

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration();

    bool Process(std::span<const uint8_t> data);

    static HalfDuplexSoftwareUart* GetInstance();
    uint16_t GetPin() const;
    void HandleExti();

private:
    static HalfDuplexSoftwareUart* s_instance;

    void DelayPeriod(uint16_t period);
    void ArmExti();

    TaskThread<HalfDuplexSoftwareUart, Config::Task::HALF_DUPLEX_SOFTWARE_UART_STACK_SIZE, Config::Task::HALF_DUPLEX_SOFTWARE_UART_PRIORITY, false> m_thread;

    StreamBufferHandle_t m_writeStream;
    StaticStreamBuffer_t m_writeStreamBuffer;
    uint8_t m_writeStreamStorage[Config::SW_UART_BUFFER_SIZE];

    SemaphoreHandle_t m_readingSemaphore;
    StaticSemaphore_t m_readingSemaphoreBuffer;

    uint16_t m_pin;
    GPIO_TypeDef* m_port;
    TIM_HandleTypeDef& m_timer;
    uint32_t m_ticksPerBit;
    uint32_t m_tickPerOversampling;

};
