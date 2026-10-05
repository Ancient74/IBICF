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
    : m_thread(taskName, 0, *this)
{
    m_inputQueue = xQueueCreateStatic(sizeof(m_inputQueueStorage), sizeof(uint8_t), m_inputQueueStorage, &m_inputQueueBuffer);
    configASSERT(m_inputQueue != nullptr);
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
    : m_thread(taskName, pdMS_TO_TICKS(Config::TRAMP_PROTOCOL_POLLING_RATE_MS), *this)
{
    m_desiredStateMutex = xSemaphoreCreateMutexStatic(&m_desiredStateBuffer);
    m_dataMutex = xSemaphoreCreateMutexStatic(&m_dataBuffer);
}

TrampProtocolController::~TrampProtocolController()
{
    Stop();
    vSemaphoreDelete(m_desiredStateMutex);
    vSemaphoreDelete(m_dataMutex);
}

void TrampProtocolController::MarkResponseReceived()
{
    m_lastResponseTick.store(xTaskGetTickCount());
}

bool TrampProtocolController::HasResponseTimedOut() const
{
    return (xTaskGetTickCount() - m_lastResponseTick.load()) > pdMS_TO_TICKS(Config::TRAMP_PROTOCOL_RESPONSE_TIMEOUT_MS);
}

void TrampProtocolController::InvalidateStatus()
{
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    m_status.reset();
    xSemaphoreGive(m_dataMutex);
}

void TrampProtocolController::Process(const TrampFrame& frame)
{
    if (frame.checksum != TrampChecksum(frame))
        return; // corrupt frame

    MarkResponseReceived();

    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    switch (static_cast<TrampCommand>(frame.command))
    {
    case TrampCommand::Capabilities:
        m_capabilities = ExtractPayload<TrampCapabilities>(frame.response);
        if (m_connectionState.load() == TrampConnectionState::Offline)
            m_connectionState.store(TrampConnectionState::Handshaking);
        break;
    case TrampCommand::Status:
        m_status = ExtractPayload<TrampStatus>(frame.response);
        if (m_connectionState.load() == TrampConnectionState::Handshaking)
            m_connectionState.store(TrampConnectionState::Online);
        break;
    case TrampCommand::Temperature:
        m_temperature = ExtractPayload<TrampTemperature>(frame.response);
        break;
    default:
        break;
    }
    xSemaphoreGive(m_dataMutex);
}

void TrampProtocolController::RunTaskIteration()
{
    if (m_connectionState.load() != TrampConnectionState::Offline && HasResponseTimedOut())
    {
        m_connectionState.store(TrampConnectionState::Offline);
        InvalidateStatus();
        m_frequencyRetries = m_powerRetries = m_pitModeRetries = 0;
        m_lastAttemptedFrequency.reset();
        m_lastAttemptedPower.reset();
        m_lastAttemptedPitMode.reset();
    }

    switch (m_connectionState.load())
    {
    case TrampConnectionState::Offline:
        SendCapabilitiesQuery();
        break;
    case TrampConnectionState::Handshaking:
        SendStatusQuery();
        break;
    case TrampConnectionState::Online:
        ReconcileState();
        break;
    }
}

void TrampProtocolController::ReconcileState()
{
    std::optional<TrampStatus> currentStatus;
    xSemaphoreTake(m_dataMutex, portMAX_DELAY);
    currentStatus = m_status;
    xSemaphoreGive(m_dataMutex);

    if (!currentStatus.has_value())
    {
        SendStatusQuery();
        return;
    }

    DesiredState desired;
    xSemaphoreTake(m_desiredStateMutex, portMAX_DELAY);
    desired = m_desiredState;
    xSemaphoreGive(m_desiredStateMutex);

    if (desired.frequencyMHz.has_value() && *desired.frequencyMHz != currentStatus->freqMHz)
    {
        if (m_lastAttemptedFrequency != desired.frequencyMHz)
        {
            m_lastAttemptedFrequency = desired.frequencyMHz;
            m_frequencyRetries = 0;
        }
        if (m_frequencyRetries < Config::TRAMP_PROTOCOL_MAX_SET_RETRIES)
        {
            SendParam(TrampCommand::SetFrequency, *desired.frequencyMHz);
            m_frequencyRetries++;
            InvalidateStatus();
        }
        return;
    }
    m_frequencyRetries = 0;
    m_lastAttemptedFrequency.reset();

    if (desired.powerMw.has_value() && *desired.powerMw != currentStatus->configuredPowerMw)
    {
        if (m_lastAttemptedPower != desired.powerMw)
        {
            m_lastAttemptedPower = desired.powerMw;
            m_powerRetries = 0;
        }
        if (m_powerRetries < Config::TRAMP_PROTOCOL_MAX_SET_RETRIES)
        {
            SendParam(TrampCommand::SetPower, *desired.powerMw);
            m_powerRetries++;
            InvalidateStatus();
        }
        return;
    }
    m_powerRetries = 0;
    m_lastAttemptedPower.reset();

    if (desired.pitMode.has_value())
    {
        const bool currentPit = currentStatus->pitMode != 0;
        if (*desired.pitMode != currentPit)
        {
            if (m_lastAttemptedPitMode != desired.pitMode)
            {
                m_lastAttemptedPitMode = desired.pitMode;
                m_pitModeRetries = 0;
            }
            if (m_pitModeRetries < Config::TRAMP_PROTOCOL_MAX_SET_RETRIES)
            {
                SendParam(TrampCommand::SetPitMode, *desired.pitMode ? 0 : 1);
                m_pitModeRetries++;
                InvalidateStatus();
            }
            return;
        }
    }
    m_pitModeRetries = 0;
    m_lastAttemptedPitMode.reset();

    SendStatusQuery();
}

void TrampProtocolController::SetFrequency(uint16_t frequency)
{
    xSemaphoreTake(m_desiredStateMutex, portMAX_DELAY);
    m_desiredState.frequencyMHz = frequency;
    xSemaphoreGive(m_desiredStateMutex);
}

void TrampProtocolController::SetPower(uint16_t power)
{
    xSemaphoreTake(m_desiredStateMutex, portMAX_DELAY);
    m_desiredState.powerMw = power;
    xSemaphoreGive(m_desiredStateMutex);
}

void TrampProtocolController::SetPitMode(bool pitMode)
{
    xSemaphoreTake(m_desiredStateMutex, portMAX_DELAY);
    m_desiredState.pitMode = pitMode;
    xSemaphoreGive(m_desiredStateMutex);
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

TrampConnectionState TrampProtocolController::GetConnectionState() const
{
    return m_connectionState.load();
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

void TrampProtocolController::SendQuery(TrampCommand command)
{
    if (command != TrampCommand::Capabilities && command != TrampCommand::Status && command != TrampCommand::Temperature)
        return;
    TrampFrame frame{};
    frame.command = static_cast<uint8_t>(command);
    frame.checksum = TrampChecksum(frame);
    SendCommand(frame);
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

void TrampProtocolController::SendCommand(const TrampFrame& frame)
{
    const auto bytes = std::bit_cast<std::array<uint8_t, sizeof(TrampFrame)>>(frame);
    Publish(bytes);
}
