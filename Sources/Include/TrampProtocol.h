#pragma once
#include "FreeRTOSConfig.h"
#include "TaskThread.h"
#include "Config.h"
#include "PubSubLib/Node.hpp"
#include <stream_buffer.h>
#include <optional>
#include <span>

enum class TrampCommand
{
    Capabilities = 0x72,
    Status = 0x76,
    Temperature = 0x73,
    SetFrequency = 0x46,
    SetPower = 0x50,
    SetPitMode = 0x49
};

#pragma pack(push, 1)
struct TrampFrame
{
    uint16_t GetParameterValue() const
    {
        return static_cast<uint16_t>(response[0]) | (static_cast<uint16_t>(response[1]) << 8);
    }

    void SetParameterValue(uint16_t value)
    {
        SetParameterValueLow(static_cast<uint8_t>(value));
        SetParameterValueHigh(static_cast<uint8_t>(value >> 8));
    }

    void SetParameterValueLow(uint8_t low)
    {
        response[0] = static_cast<uint8_t>(low);
    }

    void SetParameterValueHigh(uint8_t high)
    {
        response[1] = static_cast<uint8_t>(high);
    }

    const uint8_t sync = 0x0F;
    uint8_t command;
    std::array<uint8_t, 12> response;
    uint8_t checksum;
    const uint8_t footer = 0x00;
};
static_assert(sizeof(TrampFrame) == 16);

struct TrampCapabilities
{
    uint16_t minFreqMHz;
    uint16_t maxFreqMHz;
    uint16_t maxPowerMw;
    uint8_t  reserved[6];
};
static_assert(sizeof(TrampCapabilities) == 12);

struct TrampStatus
{
    uint16_t freqMHz;
    uint16_t configuredPowerMw;
    uint8_t  controlMode;   // bit0 = race-lock
    uint8_t  pitMode;
    uint16_t actualPowerMw;
    uint8_t  reserved[4];

    bool IsRaceLocked() const { return controlMode & 0x01; }
};
static_assert(sizeof(TrampStatus) == 12);

struct TrampTemperature
{
    uint8_t  reserved0[4];
    int16_t  tempCelsius;
    uint8_t  reserved1[6];
};
static_assert(sizeof(TrampTemperature) == 12);
#pragma pack(pop)

struct VtxInfo
{
    uint16_t actualPowerMw;
};

class TrampFrameParser
    : public NodeInput<TrampFrameParser, uint8_t>
    , public NodeOutput<TrampFrameParser, TrampFrame, 1>
{
public:
    TrampFrameParser(const char* taskName);
    ~TrampFrameParser();

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

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
    TaskThread<TrampFrameParser, Config::Task::TRAMP_FRAME_PARSER_STACK_SIZE, Config::Task::TRAMP_FRAME_PARSER_PRIORITY> m_thread;

    StreamBufferHandle_t m_inputStreamBuffer;
    StaticStreamBuffer_t m_inputStreamBufferBuffer;
    uint8_t m_inputStreamBufferStorage[sizeof(TrampFrame) * 2 + 1];
    TrampFrame m_pendingFrame{};
    uint8_t m_parameterIndex = 0;
    uint8_t m_responseIndex = 0;
};

enum class TrampConnectionState
{
    Offline,
    Handshaking,
    Online,
};

class TrampProtocolController
    : public NodeInput<TrampProtocolController, TrampFrame>
    , public NodeOutput<TrampProtocolController, std::span<const uint8_t>, 1>
{
public:
    class VtxInfoOutput : public NodeOutput<VtxInfoOutput, VtxInfo, 1>
    {
    public:
        using NodeOutput<VtxInfoOutput, VtxInfo, 1>::Publish;
    };

    TrampProtocolController(const char* taskName);
    ~TrampProtocolController();

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration();
    void Process(const TrampFrame& frame);

    void SetFrequency(uint16_t frequency);
    void SetPower(uint16_t power);
    void SetPitMode(bool pitMode);

    std::optional<TrampCapabilities> GetCapabilities() const;
    std::optional<TrampStatus> GetStatus() const;
    std::optional<TrampTemperature> GetTemperature() const;
    TrampConnectionState GetConnectionState() const;
    VtxInfoOutput& GetVtxInfoNode() { return m_vtxInfoOutput; }

private:
    struct DesiredState
    {
        std::optional<uint16_t> frequencyMHz;
        std::optional<uint16_t> powerMw;
        std::optional<bool> pitMode;
    };

    void SendCapabilitiesQuery();
    void SendStatusQuery();
    void SendTemperatureQuery();
    void SendQuery(TrampCommand command);
    void SendParam(TrampCommand command, uint16_t param);
    void SendCommand(const TrampFrame& frame);

    void ReconcileState();
    void InvalidateStatus();
    void MarkResponseReceived();
    bool HasResponseTimedOut() const;

    TaskThread<TrampProtocolController, Config::Task::TRAMP_PROTOCOL_CONTROLLER_STACK_SIZE, Config::Task::TRAMP_PROTOCOL_CONTROLLER_PRIORITY,
        pdMS_TO_TICKS(Config::TRAMP_PROTOCOL_POLLING_RATE_MS)> m_thread;

    std::atomic<TrampConnectionState> m_connectionState = TrampConnectionState::Offline;
    std::atomic<TickType_t> m_lastResponseTick = 0;

    SemaphoreHandle_t m_desiredStateMutex;
    StaticSemaphore_t m_desiredStateBuffer;

    DesiredState m_desiredState;

    std::optional<uint16_t> m_lastAttemptedFrequency;
    std::optional<uint16_t> m_lastAttemptedPower;
    std::optional<bool> m_lastAttemptedPitMode;

    uint8_t m_frequencyRetries = 0;
    uint8_t m_powerRetries = 0;
    uint8_t m_pitModeRetries = 0;

    SemaphoreHandle_t m_dataMutex;
    StaticSemaphore_t m_dataBuffer;

    std::optional<TrampCapabilities> m_capabilities;
    std::optional<TrampStatus> m_status;
    std::optional<TrampTemperature> m_temperature;
    VtxInfoOutput m_vtxInfoOutput;
};
