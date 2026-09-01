#pragma once
#include "FreeRTOSConfig.h"
#include <CommonTask.h>
#include "PubSubLib/Node.hpp"
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
static_assert(sizeof(TrampFrame) == 16);

struct TrampCapabilities
{
    uint16_t minFreqMHz;
    uint16_t maxFreqMHz;
    uint16_t maxPowerMw;
    uint8_t  reserved[4];
};
static_assert(sizeof(TrampCapabilities) == 10);

struct TrampStatus
{
    uint16_t freqMHz;
    uint16_t configuredPowerMw;
    uint8_t  controlMode;   // bit0 = race-lock
    uint8_t  pitMode;
    uint16_t actualPowerMw;
    uint8_t  reserved[2];

    bool IsRaceLocked() const { return controlMode & 0x01; }
};
static_assert(sizeof(TrampStatus) == 10);

struct TrampTemperature
{
    uint8_t  reserved0[4];
    int16_t  tempCelsius;
    uint8_t  reserved1[4];
};
static_assert(sizeof(TrampTemperature) == 10);
#pragma pack(pop)

class TrampFrameParser
    : public CommonTask<TrampFrameParser, 64, configMAX_PRIORITIES / 2, false>
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

enum class TrampConnectionState
{
    Offline,
    Handshaking,
    Online,
};

class TrampProtocolController
    : public CommonTask<TrampProtocolController, 128, configMAX_PRIORITIES / 2>
    , public NodeInput<TrampProtocolController, TrampFrame>
    , public NodeOutput<TrampProtocolController, std::span<const uint8_t>, 1>
{
public:
    TrampProtocolController(const char* taskName);
    ~TrampProtocolController();

    void RunTaskIteration();
    void Process(const TrampFrame& frame);

    void SetFrequency(uint16_t frequency);
    void SetPower(uint16_t power);
    void SetPitMode(bool pitMode);

    std::optional<TrampCapabilities> GetCapabilities() const;
    std::optional<TrampStatus> GetStatus() const;
    std::optional<TrampTemperature> GetTemperature() const;
    TrampConnectionState GetConnectionState() const;

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

    std::atomic<TrampConnectionState> m_connectionState = TrampConnectionState::Offline;
    std::atomic<TickType_t> m_lastResponseTick = 0;

    SemaphoreHandle_t m_desiredStateMutex;
    DesiredState m_desiredState;

    std::optional<uint16_t> m_lastAttemptedFrequency;
    std::optional<uint16_t> m_lastAttemptedPower;
    std::optional<bool> m_lastAttemptedPitMode;

    uint8_t m_frequencyRetries = 0;
    uint8_t m_powerRetries = 0;
    uint8_t m_pitModeRetries = 0;

    SemaphoreHandle_t m_dataMutex;
    std::optional<TrampCapabilities> m_capabilities;
    std::optional<TrampStatus> m_status;
    std::optional<TrampTemperature> m_temperature;
};
