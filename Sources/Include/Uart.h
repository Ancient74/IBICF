#pragma once
#include "main.h"
#include <array>
#include <span>

class IReader
{
public:
    virtual ~IReader() = default;
    virtual void OnByte(uint8_t byte) = 0;
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

    static void DispatchRxComplete(UART_HandleTypeDef* huart);
private:
    static constexpr size_t kMaxInstances = 4;
    static std::array<Uart*, kMaxInstances> s_instances;

    void onRxComplete();

    UART_HandleTypeDef& m_handle;
    uint8_t m_currentByte;
    IReader& m_reader;
};
