#include "CrsfFrameReaderNode.h"
#include "Config.h"
#include "FreeRTOSConfig.h"

CrsfFrameReaderNode::CrsfFrameReaderNode(const char* taskName)
    : m_thread(taskName, *this)
{
    m_streamBuffer = xStreamBufferCreateStatic(sizeof(m_buffer) - 1, 1, m_buffer, &m_streamBufferBuffer);
    configASSERT(m_streamBuffer != nullptr);
}

CrsfFrameReaderNode::~CrsfFrameReaderNode()
{
    Stop();
}

void CrsfFrameReaderNode::OnChunk(std::span<const uint8_t> chunk)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xStreamBufferSendFromISR(m_streamBuffer, chunk.data(), chunk.size(), &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void CrsfFrameReaderNode::RunTaskIteration()
{
    std::array<uint8_t, Config::CRSF_READER_CHUNK_SIZE> chunk{};
    const auto bytesRead = xStreamBufferReceive(m_streamBuffer,
        chunk.data(), chunk.size(), pdMS_TO_TICKS(Config::CRSF_READER_TIMEOUT_MS));
    for (uint16_t i = 0; i < bytesRead; i++)
        Publish(chunk[i]);
}
