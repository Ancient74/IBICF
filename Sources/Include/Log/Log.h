#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>

extern "C" uint32_t HAL_GetTick(void);

enum class Severity : uint8_t
{
    Info,
    Warning,
    Error,
    Heartbeat
};

#pragma pack(push, 1)
struct LogMessage
{
    uint32_t timeStamp;
    Severity severity;
    std::array<char, 32> module;
    std::array<char, 128> message;
    std::array<char, 32> fileName;
    uint16_t line;
};
#pragma pack(pop)

#ifdef DEBUG

template <std::size_t N>
constexpr void CopyString(std::array<char, N>& dst, const char* src)
{
    std::size_t i = 0;
    if (src != nullptr)
    {
        for (; i < N - 1 && src[i] != '\0'; i++)
            dst[i] = src[i];
    }
    dst[i] = '\0';
}

bool SubmitLog(const LogMessage& message);

template <typename... Args>
inline bool Log(const char* module
              , const char* fileName
              , uint16_t line, Severity severity, const char* format, Args&&... args)
{
    LogMessage message{};
    CopyString(message.module, module);
    message.severity = severity;
    message.timeStamp = HAL_GetTick();
    CopyString(message.fileName, fileName);
    message.line = line;
    std::snprintf(message.message.data(), message.message.size(), format, std::forward<Args>(args)...);
    return SubmitLog(message);
}

#define LogError(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Error, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogWarning(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Warning, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogInfo(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Info, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogHeartbeat(module) Log(module, __FILE_NAME__, __LINE__, Severity::Heartbeat, "")

#else

template <typename... Args>
static inline bool LogStub(const char*, const char*, Args&&...) { return true; }

#define LogError(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogWarning(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogInfo(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogInfo(module) LogStub(module, "")

#endif
