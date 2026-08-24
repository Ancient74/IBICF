#pragma once
#include "FreeRTOSConfig.h"
#include <CommonTask.h>
#include "PubSubLib/Node.hpp"

#pragma pack(push, 1)
struct TrampFrame
{
    const uint8_t sync = 0x0F;
    uint8_t command;
    struct Parameter
    {
        union
        {
            uint16_t value;

            struct
            {
                uint8_t low;
                uint8_t high;
            };
        };
    } parameter;
    std::array<uint8_t, 10> response;
    uint8_t checksum;
    const uint8_t footer = 0x00;
};
#pragma pack(pop)
static_assert(sizeof(TrampFrame) == 16);

class TrampFrameParser
    : public CommonTask<TrampFrameParser, 64, configMAX_PRIORITIES / 2>
    , public NodeInput<TrampFrameParser, uint8_t>
    , public NodeOutput<TrampFrameParser, TrampFrame, 1>
{
public:
    TrampFrameParser(const char* taskName);
    ~TrampFrameParser();

    void Process(uint8_t byte);
    void RunTaskIteration();

private:
    enum class State
    {
        SyncFrame,
        Command,
        Parameter,
        Response,
        Checksum,
        Footer
    } m_state = TrampFrameParser::State::SyncFrame;
    QueueHandle_t m_inputQueue;
    TrampFrame m_pendingFrame{};
    uint8_t m_parameterIndex = 0;
    uint8_t m_responseIndex = 0;
};
