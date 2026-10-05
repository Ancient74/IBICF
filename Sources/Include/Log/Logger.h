#pragma once
#include <cstdint>
#include "stm32f4xx_hal.h"
#include <array>
#include "Config.h"
#include "TaskThread.h"
#include <optional>

enum class Severity
{
    Info,
    Warning,
    Error
};

struct LogMessage
{
    uint32_t timeStamp;
    Severity severity;
    std::array<char, 32> module;
    std::array<char, 128> message;
    std::array<char, 32> fileName;
    uint16_t line;
};

class Logger
{
public:
    Logger(const char* taskName);

    bool Log(const LogMessage& message);
    void TransmitCompleteFromISR();

    static Logger* GetInstance();

    void RunTaskIteration();
private:
    inline static Logger* s_instance = nullptr;
    TaskThread<Logger, Config::Task::LOGGER_STACK_SIZE, Config::Task::LOGGER_PRIORITY, false> m_thread;

    std::optional<LogMessage> m_pendingMessage = std::nullopt;

    QueueHandle_t m_queue;
    StaticQueue_t m_queueBuffer;
    uint8_t m_queueStorage[Config::LOGGER_QUEUE_SIZE * sizeof(LogMessage)];
    SemaphoreHandle_t m_txCompleteSemaphore{};
    StaticSemaphore_t m_txCompleteSemaphoreBuffer{};
    bool m_txInFlight{false};
};
