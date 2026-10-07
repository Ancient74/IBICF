#pragma once
#include "CrsfLib/Frame.hpp"
#include "Uart.h"
#include "TaskThread.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stream_buffer.h>
#include <PubSubLib/Node.hpp>
#include "Config.h"

class CrsfFrameReaderNode
    : public IReader
    , public NodeOutput<CrsfFrameReaderNode, uint8_t, 1>
{
public:
    CrsfFrameReaderNode(const char* taskName);
    ~CrsfFrameReaderNode();
    void OnChunk(std::span<const uint8_t> chunk) override;

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration();
private:
    TaskThread<CrsfFrameReaderNode, Config::Task::CRSF_FRAME_READER_NODE_STACK_SIZE, Config::Task::CRSF_FRAME_READER_NODE_PRIORITY, false> m_thread;
    StreamBufferHandle_t m_streamBuffer;
    StaticStreamBuffer_t m_streamBufferBuffer;
    uint8_t m_buffer[Config::CRSF_READER_BUFFER_SIZE];
};
