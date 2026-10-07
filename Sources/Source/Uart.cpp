#include "Uart.h"
#include "Config.h"
#include "Log.h"

namespace
{
    constexpr auto WRAP_MASK = Config::UART_RX_BUFFER_SIZE - 1;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Uart::DispatchUartEvent(huart);
}

std::array<Uart*, Uart::kMaxInstances> Uart::s_instances{};

Uart::Uart(UART_HandleTypeDef& handle, IReader& reader)
    : m_handle(handle)
    , m_reader(reader)
{
    for (auto& slot : s_instances) {
        if (slot == nullptr) { slot = this; break; }
    }
    const auto status = HAL_UARTEx_ReceiveToIdle_DMA(&m_handle, m_buffer.data(), m_buffer.size());
    if (status != HAL_OK)
        LogError("UART", "Failed to start RX (status=%d)", static_cast<int>(status));
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

void Uart::Write(std::span<const uint8_t> buffer)
{
    const auto status = HAL_UART_Transmit(&m_handle, buffer.data(), static_cast<uint16_t>(buffer.size()), 100);
    if (status != HAL_OK)
        LogWarning("UART", "Transmit failed (status=%d, bytes=%u)", static_cast<int>(status),
                   static_cast<unsigned>(buffer.size()));
}
