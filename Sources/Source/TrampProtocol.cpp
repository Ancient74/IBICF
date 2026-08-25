#include "TrampProtocol.h"
#include "Config.h"
#include <span>
#include <cstring>

namespace
{
    uint8_t TrampChecksum(const TrampFrame& frame)
    {
        uint8_t sum = frame.command + frame.parameter.value;
        for (uint8_t i = 0; i < frame.response.size(); i++)
            sum += frame.response[i];

        return sum;
    }

    template <typename T>
    T ExtractPayload(const std::array<uint8_t, 10>& payload)
    {
        static_assert(sizeof(T) == payload.size());
        T value;
        std::memcpy(&value, payload.data(), sizeof(T));
        return value;
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
        if (byte != static_cast<uint8_t>(TrampCommand::Capabilities) && byte != static_cast<uint8_t>(TrampCommand::Status) && byte != static_cast<uint8_t>(TrampCommand::Temperature))
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

TrampProtocolController::TrampProtocolController(const char* taskName)
    : CommonTask<TrampProtocolController, 128, configMAX_PRIORITIES / 2>(taskName, pdMS_TO_TICKS(Config::TRAMP_PROTOCOL_POLLING_RATE_MS))
{
    m_dataMutex = xSemaphoreCreateMutex();
    Start();
}

TrampProtocolController::~TrampProtocolController()
{
    Stop();
    vSemaphoreDelete(m_dataMutex);
}

void TrampProtocolController::RunTaskIteration()
{
    bool hasCapabilities = false;
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    hasCapabilities = m_capabilities.has_value();
    xSemaphoreGive(m_dataMutex);

    if (!hasCapabilities)
        SendCapabilitiesQuery();
    SendStatusQuery();
}

void TrampProtocolController::Process(const TrampFrame& frame)
{
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    switch (static_cast<TrampCommand>(frame.command))
    {
    case TrampCommand::Capabilities:
        m_capabilities = ExtractPayload<TrampCapabilities>(frame.response);
        break;
    case TrampCommand::Status:
        m_status = ExtractPayload<TrampStatus>(frame.response);
        break;
    case TrampCommand::Temperature:
        m_temperature = ExtractPayload<TrampTemperature>(frame.response);
        break;
    default:
        break;
    }
    xSemaphoreGive(m_dataMutex);
}

std::optional<TrampCapabilities> TrampProtocolController::GetCapabilities() const
{
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    auto value = m_capabilities;
    xSemaphoreGive(m_dataMutex);
    return value;
}

std::optional<TrampStatus> TrampProtocolController::GetStatus() const
{
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    auto value = m_status;
    xSemaphoreGive(m_dataMutex);
    return value;
}

std::optional<TrampTemperature> TrampProtocolController::GetTemperature() const
{
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    auto value = m_temperature;
    xSemaphoreGive(m_dataMutex);
    return value;
}

void TrampProtocolController::SetFrequency(uint16_t frequency)
{
    SendParam(TrampCommand::SetFrequency, frequency);
}

void TrampProtocolController::SetPower(uint16_t power)
{
    SendParam(TrampCommand::SetPower, power);
}

void TrampProtocolController::SetPitMode(bool pitMode)
{
    SendParam(TrampCommand::SetPitMode, pitMode);
}

void TrampProtocolController::SendCapabilitiesQuery()
{
    SendQuery(TrampCommand::Capabilities);
}

void TrampProtocolController::SendStatusQuery()
{
    SendQuery(TrampCommand::Status);
}

void TrampProtocolController::SendTemperatureQuery()
{
    SendQuery(TrampCommand::Temperature);
}

void TrampProtocolController::SendParam(TrampCommand command, uint16_t param)
{
    if (command != TrampCommand::SetFrequency && command != TrampCommand::SetPitMode && command != TrampCommand::SetPower)
        return;
    TrampFrame frame{};
    frame.command = static_cast<uint8_t>(command);
    frame.parameter.value = param;
    frame.checksum = TrampChecksum(frame);
    SendCommand(frame);
}

void TrampProtocolController::SendQuery(TrampCommand command)
{
    if (command != TrampCommand::Capabilities && command != TrampCommand::Status && command != TrampCommand::Temperature)
        return;
    TrampFrame frame{};
    frame.command = static_cast<uint8_t>(command);
    frame.checksum = TrampChecksum(frame);
    SendCommand(frame);
}

void TrampProtocolController::SendCommand(const TrampFrame& frame)
{
    const auto bytes = std::bit_cast<std::array<uint8_t, sizeof(TrampFrame)>>(frame);
    Publish(bytes);
}
