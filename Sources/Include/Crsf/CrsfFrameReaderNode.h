#pragma once
#include "CrsfLib/Frame.hpp"
#include "CommonTask.h"
#include "Uart.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stream_buffer.h>
#include <PubSubLib/Node.hpp>

class CrsfFrameReaderNode
    : public IReader
    , public CommonTask<CrsfFrameReaderNode, 128, configMAX_PRIORITIES - 2, false>
    , public NodeOutput<CrsfFrameReaderNode, uint8_t, 1>
{
public:
    CrsfFrameReaderNode();
    ~CrsfFrameReaderNode();
    void OnByte(uint8_t byte) override;

    void RunTaskIteration();
private:
    StreamBufferHandle_t m_streamBuffer;
};
