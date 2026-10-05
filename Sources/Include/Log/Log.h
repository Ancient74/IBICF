#pragma once
#include "Logger.h"
#include "stm32f4xx_hal.h"
#include <array>
#include <cstddef>
#include <cstdio>
#include <utility>

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
    return Logger::GetInstance()->Log(message);
}

#define LogError(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Error, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogWarning(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Warning, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogInfo(module, fmt, ...) Log(module, __FILE_NAME__, __LINE__, Severity::Info, fmt __VA_OPT__(,) __VA_ARGS__)

#else

template <typename... Args>
static inline bool LogStub(const char*, const char*, Args&&...) { return true; }

#define LogError(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogWarning(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)
#define LogInfo(module, fmt, ...) LogStub(module, fmt __VA_OPT__(,) __VA_ARGS__)

#endif
