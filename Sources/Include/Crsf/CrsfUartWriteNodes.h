#pragma once
#include <span>
#include "PubSubLib/Node.hpp"
#include "Uart.h"

class DeviceInfoWriteNode : public NodeInput<DeviceInfoWriteNode, std::span<const uint8_t>> {
public:
    explicit DeviceInfoWriteNode(Uart& uart)
        : m_uart(uart)
    {
    }

    void Process(std::span<const uint8_t> buffer)
    {
        m_uart.Write(buffer);
    }

private:
    Uart& m_uart;
};
