#pragma once
#include "CrsfLib/FrameParserNode.hpp"
#include "Uart.h"
#include <memory>
#include "CrsfFrameReaderNode.h"
#include "CrsfFrameProcessorNode.h"

class CrsfLink : public IWriter {
public:
    explicit CrsfLink(UART_HandleTypeDef& huart, std::shared_ptr<PlaneModel> model);

    CrsfLink(const CrsfLink&) = delete;
    CrsfLink& operator=(const CrsfLink&) = delete;

    void Write(std::span<const uint8_t> buffer) override;

private:
    std::shared_ptr<CrsfFrameReaderNode> m_reader;
    CrsfFrameProcessorNode m_processor;
    CrsfLib::FrameParserNode m_frameParser;
    Uart m_uart;
};
