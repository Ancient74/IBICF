#pragma once
#include <span>
#include "PubSubLib/Node.hpp"
#include "Uart.h"

class DeviceInfoWriteNode : public NodeInput<DeviceInfoWriteNode, std::span<const uint8_t>> {
public:
    DeviceInfoWriteNode(std::shared_ptr<Uart> uart)
        : m_uart(uart)
    {
    }

    void Process(std::span<const uint8_t> buffer)
    {
        m_uart->Write(buffer);
    }

private:
    std::shared_ptr<Uart> m_uart;
};
