#include "CrsfFrameReaderNode.h"

CrsfFrameReaderNode::CrsfFrameReaderNode()
    : CommonTask<CrsfFrameReaderNode, 128, configMAX_PRIORITIES - 2, false>("CrsfByteTask", 0)
{
    m_streamBuffer = xStreamBufferCreate(/*size*/ 256, /*trigger*/ 1);
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
        chunk, sizeof(chunk), portMAX_DELAY);
    for (size_t i = 0; i < n; ++i)
        Publish(chunk[i]);
}
