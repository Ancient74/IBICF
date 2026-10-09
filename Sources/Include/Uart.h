#pragma once
#include "main.h"
#include <array>
#include <span>
#include <bit>
#include <atomic>
#include "Config.h"

class IReader
{
public:
    virtual ~IReader() = default;
    virtual void OnChunk(std::span<const uint8_t> chunk) = 0;
};

class IWriter
{
public:
    virtual ~IWriter() = default;
    virtual void Write(std::span<const uint8_t> buffer) = 0;
};

class Uart : public IWriter
{
public:
    Uart(UART_HandleTypeDef& handle, IReader& reader);
    ~Uart();

    Uart(const Uart&) = delete;
    Uart(Uart&&) = delete;
    Uart& operator=(const Uart&) = delete;
    Uart& operator=(Uart&&) = delete;

    void Write(std::span<const uint8_t> buffer) override;

    void Restart();

    static void DispatchUartEvent(UART_HandleTypeDef* huart);
    static void DispatchUartError(UART_HandleTypeDef* huart);
private:
    static constexpr uint8_t s_instanceCount = 2
#ifdef UART3
        + 1
#endif
#ifdef UART4
        + 1
#endif
        ;

    static inline std::array<Uart*, s_instanceCount> s_instances{};

    void OnUartEvent();
    void OnUartError();

    UART_HandleTypeDef& m_handle;
    IReader& m_reader;
    std::atomic<uint16_t> m_lastPosition = 0;
    uint16_t m_failedArms = 0;
    std::array<uint8_t, Config::UART_RX_BUFFER_SIZE> m_buffer{};
    static_assert(std::has_single_bit(Config::UART_RX_BUFFER_SIZE));
};
