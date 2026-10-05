#include "Uart.h"
#include "Log.h"

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Uart::DispatchRxComplete(huart);
}

std::array<Uart*, Uart::kMaxInstances> Uart::s_instances{};

Uart::Uart(UART_HandleTypeDef& handle, IReader& reader)
    : m_handle(handle)
    , m_reader(reader)
{
    for (auto& slot : s_instances) {
        if (slot == nullptr) { slot = this; break; }
    }
    const auto status = HAL_UART_Receive_IT(&m_handle, &m_currentByte, 1);
    if (status != HAL_OK)
        LogError("UART", "Failed to start RX interrupt (status=%d)", static_cast<int>(status));
}

Uart::~Uart()
{
    for (auto& slot : s_instances) {
        if (slot == this) { slot = nullptr; break; }
    }
}

void Uart::DispatchRxComplete(UART_HandleTypeDef* huart)
{
    for (auto* inst : s_instances) {
        if (inst && &inst->m_handle == huart) {
            inst->onRxComplete();
            return;
        }
    }
}

void Uart::onRxComplete()
{
    m_reader.OnByte(m_currentByte);
    HAL_UART_Receive_IT(&m_handle, &m_currentByte, 1);
}

void Uart::Write(std::span<const uint8_t> buffer)
{
    const auto status = HAL_UART_Transmit(&m_handle, buffer.data(), static_cast<uint16_t>(buffer.size()), 100);
    if (status != HAL_OK)
        LogWarning("UART", "Transmit failed (status=%d, bytes=%u)", static_cast<int>(status),
                   static_cast<unsigned>(buffer.size()));
}
