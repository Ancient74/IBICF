#include "Uart.h"
#include "Config.h"
#include "Log.h"

namespace
{
    constexpr auto WRAP_MASK = Config::UART_RX_BUFFER_SIZE - 1;

    uint8_t GetUartId(const UART_HandleTypeDef& huart) {
        if (huart.Instance == USART1) return 1;
        if (huart.Instance == USART2) return 2;
        #ifdef UART3
        if (huart.Instance == USART3) return 3;
        #endif
        #ifdef UART4
        if (huart.Instance == UART4)  return 4;
        #endif
        return 0; // Unknown or error
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Uart::DispatchUartEvent(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Uart::DispatchUartError(huart);
}

Uart::Uart(UART_HandleTypeDef& handle, IReader& reader)
    : m_handle(handle)
    , m_reader(reader)
{
    for (auto& slot : s_instances) {
        if (slot == nullptr) { slot = this; break; }
    }
    const auto status = HAL_UARTEx_ReceiveToIdle_DMA(&m_handle, m_buffer.data(), m_buffer.size());
    if (status != HAL_OK)
        LogError("UART", "Failed to start RX (instance=%d, status=%d)", GetUartId(m_handle), static_cast<int>(status));
}

Uart::~Uart()
{
    for (auto& slot : s_instances)
        if (slot == this) { slot = nullptr; break; }
}

void Uart::DispatchUartEvent(UART_HandleTypeDef* huart)
{
    for (auto instance : s_instances)
    {
        if (instance && &instance->m_handle == huart)
        {
            instance->OnUartEvent();
            return;
        }
    }
}

void Uart::DispatchUartError(UART_HandleTypeDef* huart)
{
    for (auto instance : s_instances)
    {
        if (instance && &instance->m_handle == huart)
        {
            instance->OnUartError();
            return;
        }
    }
}

void Uart::OnUartEvent()
{
    const uint16_t head = (m_buffer.size() - __HAL_DMA_GET_COUNTER(m_handle.hdmarx)) & WRAP_MASK;
    const std::span<const uint8_t> fullChunk(m_buffer);

    if (head > m_lastPosition)
    {
        m_reader.OnChunk(fullChunk.subspan(m_lastPosition, head - m_lastPosition));
    }
    else if (head < m_lastPosition)
    {
        m_reader.OnChunk(fullChunk.subspan(m_lastPosition));
        if (head > 0)
            m_reader.OnChunk(fullChunk.first(head));
    }

    m_lastPosition = head;
}

void Uart::OnUartError()
{
    m_lastPosition = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&m_handle, m_buffer.data(), m_buffer.size());
}

void Uart::Write(std::span<const uint8_t> buffer)
{
    const auto status = HAL_UART_Transmit(&m_handle, buffer.data(), static_cast<uint16_t>(buffer.size()), 100);
    if (status != HAL_OK)
        LogWarning("UART", "Transmit failed (instance=%d, status=%d, bytes=%u)", GetUartId(m_handle), static_cast<int>(status),
                   static_cast<unsigned>(buffer.size()));
}
