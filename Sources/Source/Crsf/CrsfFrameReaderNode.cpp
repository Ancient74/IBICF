#include "CrsfFrameReaderNode.h"
#include "Config.h"
#include "FreeRTOSConfig.h"

CrsfFrameReaderNode::CrsfFrameReaderNode(const char* taskName)
    : m_thread(taskName, 0, *this)
{
    m_streamBuffer = xStreamBufferCreateStatic(Config::CRSF_READER_BUFFER_SIZE, 1, m_buffer, &m_streamBufferBuffer);
    configASSERT(m_streamBuffer != nullptr);
}

CrsfFrameReaderNode::~CrsfFrameReaderNode()
{
    Stop();
}

void CrsfFrameReaderNode::OnByte(uint8_t byte)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xStreamBufferSendFromISR(m_streamBuffer, &byte, 1, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void CrsfFrameReaderNode::RunTaskIteration()
{
    uint8_t chunk[32];
    size_t n = xStreamBufferReceive(m_streamBuffer,
        chunk, sizeof(chunk), pdMS_TO_TICKS(Config::CRSF_READER_TIMEOUT_MS));
    for (size_t i = 0; i < n; ++i)
        Publish(chunk[i]);
}
