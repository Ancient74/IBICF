#include "HalfDuplexSoftwareUart.h"
#include "Config.h"
#include "projdefs.h"
#include "stm32f4xx_hal_gpio.h"
#include <stm32f4xx_hal_tim.h>

namespace
{
    uint32_t GetApb2TimerClockFreq()
    {
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        return (ppre2 & 0x4u) ? pclk2 * 2 : pclk2;
    }
}

HalfDuplexSoftwareUart* HalfDuplexSoftwareUart::s_instance = nullptr;

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    auto instance = HalfDuplexSoftwareUart::GetInstance();
    if (instance && GPIO_Pin == instance->GetPin())
        instance->HandleExti();
}

HalfDuplexSoftwareUart::HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, uint32_t baudRate)
    : m_thread("HalfDuplexUart", 0, *this)
    , m_pin(pin)
    , m_port(port)
    , m_timer(timer)
    , m_ticksPerBit(GetApb2TimerClockFreq() / (m_timer.Instance->PSC + 1) / baudRate)
    , m_tickPerOversampling(m_ticksPerBit >> 4)
{
    if (!s_instance)
        s_instance = this;
    m_writeStream = xStreamBufferCreateStatic(Config::SW_UART_BUFFER_SIZE, 1, m_writeStreamStorage, &m_writeStreamBuffer);
    configASSERT(m_writeStream != nullptr);
    m_readingSemaphore = xSemaphoreCreateBinaryStatic(&m_readingSemaphoreBuffer);
    HAL_TIM_Base_Start(&m_timer);
}

HalfDuplexSoftwareUart::~HalfDuplexSoftwareUart()
{
    Stop();
    vStreamBufferDelete(m_writeStream);
    vSemaphoreDelete(m_readingSemaphore);
    s_instance = nullptr;
}

void HalfDuplexSoftwareUart::RunTaskIteration()
{
    std::array<uint8_t, Config::SW_UART_CHUNK_SIZE> chunk{};
    while (true)
    {
        const auto bytesRead = xStreamBufferReceive(m_writeStream, chunk.data(), chunk.size(), 0);
        if (bytesRead == 0)
            break;

        for (uint16_t i = 0; i < bytesRead; i++)
        {
            const uint8_t byte = chunk[i];
            HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
            DelayPeriod(m_ticksPerBit);

            for (uint8_t bit = 0; bit < 8; bit++)
            {
                HAL_GPIO_WritePin(m_port, m_pin, (byte & (1u << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                DelayPeriod(m_ticksPerBit);
            }
            HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
            DelayPeriod(m_ticksPerBit);
        }
    }
    if (xSemaphoreTake(m_readingSemaphore, pdMS_TO_TICKS(Config::SOFTWARE_UART_READ_TIMEOUT_MS)) != pdTRUE)
        return;

    while (HAL_GPIO_ReadPin(m_port, m_pin) == GPIO_PIN_RESET)
    {
        uint8_t byte = 0;
        DelayPeriod(m_ticksPerBit);   // 1.0P -> start of bit 0
        for (uint8_t i = 0; i < 8; i++)
        {
            uint8_t oversamplingCounter = 0;
            for (uint8_t j = 0; j < 16; j++)
            {
                oversamplingCounter += (HAL_GPIO_ReadPin(m_port, m_pin) == GPIO_PIN_SET ? 1 : 0);
                DelayPeriod(m_tickPerOversampling);   // period is divided by 16 -> start of next bit
            }
            byte |= (oversamplingCounter >= 8 ? 1 : 0) << i;
        }
        DelayPeriod(m_ticksPerBit >> 1);
        // ignore stop bit reading;
        Publish(byte);
    }
    xSemaphoreGive(m_readingSemaphore);
    ArmExti();
}

bool HalfDuplexSoftwareUart::Process(std::span<const uint8_t> data)
{
    return xStreamBufferSend(m_writeStream, data.data(), data.size(), 0) != 0;
}

void HalfDuplexSoftwareUart::DelayPeriod(uint16_t period)
{
    auto start = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&m_timer));
    while (static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&m_timer) - start) < period) {}
}

uint16_t HalfDuplexSoftwareUart::GetPin() const
{
    return m_pin;
}

HalfDuplexSoftwareUart* HalfDuplexSoftwareUart::GetInstance()
{
    return s_instance;
}

void HalfDuplexSoftwareUart::HandleExti()
{
    EXTI->IMR &= ~m_pin;

    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_readingSemaphore, &woken);
    portYIELD_FROM_ISR(woken);
}

void HalfDuplexSoftwareUart::ArmExti()
{
    __HAL_GPIO_EXTI_CLEAR_IT(m_pin);
    EXTI->IMR |= m_pin;
}
