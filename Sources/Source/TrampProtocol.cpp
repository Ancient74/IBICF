#include "TrampProtocol.h"
#include "Config.h"
#include <span>

namespace
{
    constexpr uint8_t CAPABILITIES_COMMAND = 0x72;
    constexpr uint8_t CURRENT_STATUS_COMMAND = 0x76;
    constexpr uint8_t TEMPERATURE_COMMAND = 0x73;

    uint8_t TrampChecksum(const TrampFrame& frame)
    {
        uint8_t sum = frame.command + frame.parameter.value;
        for (uint8_t i = 0; i < frame.response.size(); i++)
            sum += frame.response[i];

        return sum;
    }
}

TrampFrameParser::TrampFrameParser(const char* taskName)
    : CommonTask<TrampFrameParser, 64, configMAX_PRIORITIES / 2>(taskName, 0)
{
    m_inputQueue = xQueueCreate(sizeof(TrampFrame) * 2, sizeof(uint8_t)); // enough to hold 2 frames
    Start();
}

TrampFrameParser::~TrampFrameParser()
{
    Stop();
    vQueueDelete(m_inputQueue);
}

void TrampFrameParser::Process(uint8_t byte)
{
    xQueueSend(m_inputQueue, &byte, 0);
}

void TrampFrameParser::RunTaskIteration()
{
    uint8_t byte;
    if (xQueueReceive(m_inputQueue, &byte, Config::TRAMP_PROTOCOL_POLLING_RATE_MS) != pdTRUE)
        return;

    switch (m_state)
    {
    case State::SyncFrame:
        if (byte == m_pendingFrame.sync)
            m_state = State::Command;
        break;
    case State::Command:
        if (byte != CAPABILITIES_COMMAND && byte != CURRENT_STATUS_COMMAND && byte != TEMPERATURE_COMMAND)
            m_state = State::SyncFrame;
        else
        {
            m_pendingFrame.command = byte;
            m_state = State::Parameter;
        }
        break;
    case State::Parameter:
        if (m_parameterIndex == 0)
        {
            m_parameterIndex++;
            m_pendingFrame.parameter.low = byte;
        }
        else if(m_parameterIndex == 1)
        {
            m_parameterIndex = 0;
            m_pendingFrame.parameter.high = byte;
            m_state = State::Response;
        }
        break;
    case State::Response:
        m_pendingFrame.response[m_responseIndex++] = byte;
        if (m_responseIndex == m_pendingFrame.response.size())
        {
            m_state = State::Checksum;
            m_responseIndex = 0;
        }
        break;
    case State::Checksum:
        if (TrampChecksum(m_pendingFrame) == byte)
        {
            m_pendingFrame.checksum = byte;
            m_state = State::Footer;
        }
        else
            m_state = State::SyncFrame;
        break;
    case State::Footer:
        if (byte == m_pendingFrame.footer)
            Publish(m_pendingFrame);
        m_pendingFrame.command = 0;
        m_pendingFrame.parameter.value = 0;
        m_pendingFrame.response.fill(0);
        m_pendingFrame.checksum = 0;
        m_state = State::SyncFrame;
    default:
        break;
    }
}
