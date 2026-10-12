#pragma once
#include <stdint.h>
#include "TaskThread.h"
#include "FreeRTOS.h"
#include "PubSubLib/Node.hpp"
#include <stm32f4xx_hal.h>
#include "stm32f4xx_hal_dma.h"
#include "stream_buffer.h"
#include <span>
#include "Config.h"

class HalfDuplexSoftwareUart
    : public NodeOutput<HalfDuplexSoftwareUart, uint8_t, 1>
    , public NodeInput<HalfDuplexSoftwareUart, std::span<const uint8_t>>
{
public:
    // timer is apb2 only
    HalfDuplexSoftwareUart(uint16_t pin, GPIO_TypeDef* port, TIM_HandleTypeDef& timer, DMA_HandleTypeDef& dma, uint32_t baudRate = 9600);
    ~HalfDuplexSoftwareUart();

    void Start();
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration();

    void Process(std::span<const uint8_t> data);

    static HalfDuplexSoftwareUart* GetInstance();
    uint16_t GetPin() const;
    void HandleExti();

private:
    static inline HalfDuplexSoftwareUart* s_instance = nullptr;
    static void OnDmaCompleted(DMA_HandleTypeDef* dma);
    static void OnDmaError(DMA_HandleTypeDef* dma);
    void OnDmaCompleted(uint32_t error);

    void Tx();

    void SetDmaMode(uint32_t direction);
    bool SetDmaModeFromISR(uint32_t direction);

    void ArmExti();
    void ArmTx();
    void ArmRx(); // Set timer, dma and gpio in RX mode.
    void StartRx(); // Start timer and DMA. Time critical.
    void Rx(); // Parses RX result after StartRx notifies about completion.

    bool Listening() const;
    bool CaptureInFlight() const;
    void MaskExti();
    void StopSampler();
    void OnRxComplete(uint32_t error, BaseType_t& woken);
    void OnTxComplete(uint32_t error);
    bool AcquireLine();
    uint16_t BuildFrames(std::span<const uint8_t> data);
    const char* SendWords(uint16_t words);
    void WaitStopBit();
    void EndChunk(bool moreFollows);
    TickType_t ServiceTx();

    bool DecodeFrame(uint8_t& byte) const;

    bool WaitDone(TickType_t timeout);

    TaskThread<HalfDuplexSoftwareUart, Config::Task::HALF_DUPLEX_SOFTWARE_UART_STACK_SIZE, Config::Task::HALF_DUPLEX_SOFTWARE_UART_PRIORITY> m_thread;

    StreamBufferHandle_t m_writeStream;
    StaticStreamBuffer_t m_writeStreamBuffer;
    uint8_t m_writeStreamStorage[Config::SW_UART_BUFFER_SIZE + 1];

    StreamBufferHandle_t m_readStream;
    StaticStreamBuffer_t m_readStreamBuffer;
    uint8_t m_readStreamStorage[Config::SW_UART_BUFFER_SIZE + 1];

    std::array<uint32_t, Config::SW_UART_CHUNK_SIZE * Config::SW_UART_BYTE_SIZE> m_dmaBuffer{};

    uint16_t m_pin;
    GPIO_TypeDef* m_port;
    TIM_HandleTypeDef& m_timer;
    DMA_HandleTypeDef& m_dma;
    uint32_t m_ticksPerBit;
    volatile TickType_t m_lastActivity = 0;
    TickType_t m_idleGap = 0;
    std::atomic<bool> m_dmaDone = false;
    std::atomic<uint32_t> m_error = false;
};
