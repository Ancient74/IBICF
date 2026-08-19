#include "CrsfLink.h"

CrsfLink::CrsfLink(UART_HandleTypeDef& huart, std::shared_ptr<PlaneModel> model)
    : m_reader(std::make_shared<CrsfFrameReaderNode>())
    , m_processor(*this, model)
    , m_uart(huart, m_reader)
{
    Connect(*m_reader, m_frameParser);
    Connect(m_frameParser, m_processor);
    m_reader->Start();
}

void CrsfLink::Write(std::span<const uint8_t> buffer)
{
    m_uart.Write(buffer);
}
