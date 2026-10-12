#include "HalfDuplexSoftwareUart.h"
#include "Config.h"
#include "projdefs.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include <stm32f4xx_hal_tim.h>
#include "gpio.h"

namespace
{
    constexpr const char* MODULE = "SW_UART";

    uint32_t GetApb2TimerClockFreq()
    {
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        return (ppre2 & 0x4u) ? pclk2 * 2 : pclk2;
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    auto instance = HalfDuplexSoftwareUart::GetInstance();
    if (instance && GPIO_Pin == instance->GetPin())
        instance->HandleExti();
}

void HalfDuplexSoftwareUart::OnDmaCompleted(DMA_HandleTypeDef*)
{
    const auto instance = HalfDuplexSoftwareUart::GetInstance();
    if (instance)
        instance->OnDmaCompleted(static_cast<uint32_t>(0));
}

void HalfDuplexSoftwareUart::OnDmaError(DMA_HandleTypeDef* dma)
{
    const auto instance = HalfDuplexSoftwareUart::GetInstance();
    if (instance)
        instance->OnDmaCompleted(HAL_DMA_GetError(dma));
}

HalfDuplexSoftwareUart::HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, DMA_HandleTypeDef& dma, uint32_t baudRate)
    : m_thread("HalfDuplexUart", *this)
    , m_pin(pin)
    , m_port(port)
    , m_timer(timer)
    , m_dma(dma)
    , m_ticksPerBit(GetApb2TimerClockFreq() / (m_timer.Instance->PSC + 1) / baudRate)
{
    configASSERT(s_instance == nullptr && "HalfDuplexSoftwareUart is a singleton");

    EXTI->IMR &= ~m_pin;

    m_writeStream = xStreamBufferCreateStatic(sizeof(m_writeStreamStorage) - 1, 1, m_writeStreamStorage, &m_writeStreamBuffer);
    configASSERT(m_writeStream != nullptr);
    m_readStream = xStreamBufferCreateStatic(sizeof(m_readStreamStorage) - 1, 1, m_readStreamStorage, &m_readStreamBuffer);
    configASSERT(m_readStream != nullptr);

    __HAL_TIM_SET_AUTORELOAD(&m_timer, m_ticksPerBit - 1);
    m_timer.Instance->EGR = TIM_EGR_UG;            // latch ARR; UDE is off, so no DMA request

    HAL_DMA_RegisterCallback(&m_dma, HAL_DMA_XFER_CPLT_CB_ID, &HalfDuplexSoftwareUart::OnDmaCompleted);
    HAL_DMA_RegisterCallback(&m_dma, HAL_DMA_XFER_ERROR_CB_ID, &HalfDuplexSoftwareUart::OnDmaError);

    m_idleGap = pdMS_TO_TICKS(1000u * Config::SW_UART_BYTE_SIZE * 2 / baudRate) + 1;   // ~2 frames of silence, in RTOS ticks
    s_instance = this;
}

HalfDuplexSoftwareUart::~HalfDuplexSoftwareUart()
{
    Stop();
    vStreamBufferDelete(m_writeStream);
    vStreamBufferDelete(m_readStream);
    HAL_DMA_UnRegisterCallback(&m_dma, HAL_DMA_XFER_CPLT_CB_ID);
    HAL_DMA_UnRegisterCallback(&m_dma, HAL_DMA_XFER_ERROR_CB_ID);
    s_instance = nullptr;
}

void HalfDuplexSoftwareUart::Start()
{
    m_thread.Start();
    ConfigureOpenDrain(m_port, m_pin);
    ArmRx();
}

bool HalfDuplexSoftwareUart::Listening() const
{
    return m_dma.Init.Direction == DMA_PERIPH_TO_MEMORY;
}

bool HalfDuplexSoftwareUart::CaptureInFlight() const
{
    return Listening() && HAL_DMA_GetState(&m_dma) == HAL_DMA_STATE_BUSY;
}

void HalfDuplexSoftwareUart::MaskExti()
{
    EXTI->IMR &= ~m_pin;
}

void HalfDuplexSoftwareUart::StopSampler()
{
    __HAL_TIM_DISABLE(&m_timer);
    __HAL_TIM_DISABLE_DMA(&m_timer, TIM_DMA_UPDATE);
}

void HalfDuplexSoftwareUart::HandleExti()
{
    MaskExti();
    m_lastActivity = xTaskGetTickCountFromISR();
    StartRx();
}

void HalfDuplexSoftwareUart::StartRx()
{
    StopSampler();
    if (HAL_DMA_Start_IT(&m_dma, reinterpret_cast<uint32_t>(&m_port->IDR),
                         reinterpret_cast<uint32_t>(m_dmaBuffer.data()), Config::SW_UART_BYTE_SIZE) != HAL_OK)
    {
        ArmExti();
        return;
    }
    __HAL_TIM_SET_COUNTER(&m_timer, m_ticksPerBit >> 1);
    __HAL_TIM_ENABLE_DMA(&m_timer, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE(&m_timer);
}

void HalfDuplexSoftwareUart::OnRxComplete(uint32_t error, BaseType_t& woken)
{
    StopSampler();
    uint8_t byte = 0;
    const bool valid = (error == 0) && DecodeFrame(byte);
    ArmExti();
    if (valid)
        xStreamBufferSendFromISR(m_readStream, &byte, 1, &woken);
}

void HalfDuplexSoftwareUart::OnTxComplete(uint32_t error)
{
    if (error != 0)
        m_error = error;
    m_dmaDone = true;

    if (error == 0 && xStreamBufferIsEmpty(m_writeStream))
    {
        StopSampler();
        if (SetDmaModeFromISR(DMA_PERIPH_TO_MEMORY))
            ArmExti();
    }
}

void HalfDuplexSoftwareUart::OnDmaCompleted(uint32_t error)
{
    BaseType_t woken = pdFALSE;
    m_lastActivity = xTaskGetTickCountFromISR();

    if (Listening())
        OnRxComplete(error, woken);
    else
        OnTxComplete(error);

    vTaskNotifyGiveFromISR(m_thread.GetTaskHandle(), &woken);
    portYIELD_FROM_ISR(woken);
}

bool HalfDuplexSoftwareUart::AcquireLine()
{
    MaskExti();
    if (CaptureInFlight())
        return false;
    if (HAL_GPIO_ReadPin(m_port, m_pin) == GPIO_PIN_RESET)
    {
        ArmExti();
        return false;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(m_pin);
    return true;
}

uint16_t HalfDuplexSoftwareUart::BuildFrames(std::span<const uint8_t> data)
{
    const uint32_t set = m_pin;
    const uint32_t reset = static_cast<uint32_t>(m_pin) << 16;
    uint16_t n = 0;
    for (size_t i = 0; i < data.size(); i++)
    {
        m_dmaBuffer[n++] = reset;
        for (uint8_t bit = 0; bit < 8; bit++)
            m_dmaBuffer[n++] = (data[i] & (1u << bit)) ? set : reset;
        m_dmaBuffer[n++] = set;
    }
    return n;
}

const char* HalfDuplexSoftwareUart::SendWords(uint16_t words)
{
    ArmTx();
    if (HAL_DMA_Start_IT(&m_dma, reinterpret_cast<uint32_t>(m_dmaBuffer.data()),
                         reinterpret_cast<uint32_t>(&m_port->BSRR), words) != HAL_OK)
        return "HAL_DMA_Start_IT failed";

    __HAL_TIM_SET_COUNTER(&m_timer, 0);
    __HAL_TIM_ENABLE_DMA(&m_timer, TIM_DMA_UPDATE);
    m_timer.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_ENABLE(&m_timer);

    if (!WaitDone(pdMS_TO_TICKS(Config::SW_UART_TIMEOUT_MS)))
        return "completion timeout";
    if (m_error.exchange(0) != 0)
        return "DMA error";
    return nullptr;
}

void HalfDuplexSoftwareUart::WaitStopBit()
{
    __HAL_TIM_CLEAR_FLAG(&m_timer, TIM_FLAG_UPDATE);
    while (!__HAL_TIM_GET_FLAG(&m_timer, TIM_FLAG_UPDATE) && (m_timer.Instance->CR1 & TIM_CR1_CEN) != 0) {}
}

void HalfDuplexSoftwareUart::EndChunk(bool moreFollows)
{
    if (Listening())
        return;
    if (moreFollows)
        WaitStopBit();
    StopSampler();
}

void HalfDuplexSoftwareUart::RunTaskIteration()
{
    Rx();
    ulTaskNotifyTake(pdTRUE, ServiceTx());
}

TickType_t HalfDuplexSoftwareUart::ServiceTx()
{
    const TickType_t idleSleep = pdMS_TO_TICKS(Config::SW_UART_TIMEOUT_MS);
    if (xStreamBufferIsEmpty(m_writeStream))
        return idleSleep;

    const TickType_t quiet = xTaskGetTickCount() - m_lastActivity;
    if (quiet < m_idleGap)
        return m_idleGap - quiet;

    Tx();
    return xStreamBufferIsEmpty(m_writeStream) ? idleSleep : m_idleGap;
}

bool HalfDuplexSoftwareUart::DecodeFrame(uint8_t& byte) const
{
    if ((m_dmaBuffer[0] & m_pin) != 0 || (m_dmaBuffer[Config::SW_UART_BYTE_SIZE - 1] & m_pin) == 0)
        return false;

    byte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((m_dmaBuffer[i + 1] & m_pin) != 0)
            byte |= static_cast<uint8_t>(1u << i);
    }
    return true;
}

void HalfDuplexSoftwareUart::ArmExti()
{
    __HAL_GPIO_EXTI_CLEAR_IT(m_pin);
    EXTI->IMR |= m_pin;
}

void HalfDuplexSoftwareUart::SetDmaMode(uint32_t direction)
{
    if (m_dma.Init.Direction == direction)
        return;
    HAL_DMA_Abort(&m_dma);
    m_dma.Init.Direction = direction;
    HAL_DMA_Init(&m_dma);
}

bool HalfDuplexSoftwareUart::SetDmaModeFromISR(uint32_t direction)
{
    if ((m_dma.Instance->CR & DMA_SxCR_EN) != 0)
        return false;
    m_dma.Instance->CR = (m_dma.Instance->CR & ~DMA_SxCR_DIR) | direction;
    m_dma.Init.Direction = direction;
    return true;
}

void HalfDuplexSoftwareUart::ArmTx()
{
    if (m_dma.Init.Direction != DMA_MEMORY_TO_PERIPH)
    {
        EXTI->IMR &= ~m_pin;
        StopSampler();
        SetDmaMode(DMA_MEMORY_TO_PERIPH);
    }
}

void HalfDuplexSoftwareUart::ArmRx()
{
    StopSampler();
    SetDmaMode(DMA_PERIPH_TO_MEMORY);
    ArmExti();
}

void HalfDuplexSoftwareUart::Rx()
{
    uint8_t byte = 0;
    while (xStreamBufferReceive(m_readStream, &byte, 1, 0) == 1)
        Publish(byte);
}

bool HalfDuplexSoftwareUart::WaitDone(TickType_t timeout)
{
    const TickType_t start = xTaskGetTickCount();
    while (!m_dmaDone.exchange(false))
    {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout)
            return false;
        ulTaskNotifyTake(pdTRUE, timeout - elapsed);
    }
    return true;
}
void HalfDuplexSoftwareUart::Tx()
{
    if (!AcquireLine())
        return;

    std::array<uint8_t, Config::SW_UART_CHUNK_SIZE> chunk{};
    auto count = xStreamBufferReceive(m_writeStream, chunk.data(), chunk.size(), 0);
    if (count == 0)
    {
        ArmExti();
        return;
    }
    m_error = 0;
    m_dmaDone = false;

    const char* failure = nullptr;
    while (count != 0)
    {
        failure = SendWords(BuildFrames(std::span<const uint8_t>(chunk.data(), count)));
        if (failure)
            break;
        count = xStreamBufferReceive(m_writeStream, chunk.data(), chunk.size(), 0);
        EndChunk(count != 0);
    }

    if (!Listening())
        ArmRx();

    if (failure)
        LogError(MODULE, "TX aborted: %s (dma err=%lu)", failure,
                 static_cast<unsigned long>(m_dma.ErrorCode));
}

void HalfDuplexSoftwareUart::Process(std::span<const uint8_t> data)
{
    while (!data.empty())
    {
        const auto sent = xStreamBufferSend(m_writeStream, data.data(),
                                            std::min<size_t>(data.size(), Config::SW_UART_CHUNK_SIZE),
                                            pdMS_TO_TICKS(50));
        if (sent == 0)
            return;
        data = data.subspan(sent);
        xTaskNotifyGive(m_thread.GetTaskHandle());
    }
}

uint16_t HalfDuplexSoftwareUart::GetPin() const
{
    return m_pin;
}

HalfDuplexSoftwareUart* HalfDuplexSoftwareUart::GetInstance()
{
    return s_instance;
}
