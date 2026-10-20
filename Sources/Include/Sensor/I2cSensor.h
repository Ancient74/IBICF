#pragma once
#include "FreeRTOS.h"
#include <atomic>
#include <stdint.h>
#include <span>
#include "main.h"
#include "semphr.h"
#include "SensorStatus.h"

class I2cBusLock
{
public:
    explicit I2cBusLock(SemaphoreHandle_t m);
    ~I2cBusLock();

    I2cBusLock(const I2cBusLock&) = delete;
    I2cBusLock& operator=(const I2cBusLock&) = delete;

private:
    SemaphoreHandle_t m_mutex;
};

class I2cBus
{
public:
    explicit I2cBus(I2C_HandleTypeDef& i2c);
    ~I2cBus();

    void ScanBus();

    Status WriteBytes(uint16_t deviceAddress, uint16_t registerAddress, std::span<const uint8_t> bytes);
    Status ReadBytes(uint16_t deviceAddress, uint16_t registerAddress, std::span<uint8_t> buffer);

private:
    static constexpr uint8_t s_instanceCount = 3;
    static inline std::array<I2cBus*, s_instanceCount> s_instances{};

    friend void ::HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c);
    friend void ::HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c);
    friend void ::HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c);

    void OnComplete(bool failed);
    void Recover();

    static I2cBus* FindInstance(I2C_HandleTypeDef* hi2c);

    Status Transfer(
        HAL_StatusTypeDef(*TransferFunction)(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
        , uint16_t deviceAddress, uint16_t registerAddress, std::span<uint8_t> buffer);

    I2C_HandleTypeDef& m_i2c;
    SemaphoreHandle_t m_mutex;
    StaticSemaphore_t m_mutexBuffer{};

    SemaphoreHandle_t m_doneSemaphore;
    StaticSemaphore_t m_doneSemaphoreBuffer{};

    std::atomic_bool m_failed;
};

class I2cSensor
{
public:
    I2cSensor(I2cBus& bus, uint16_t address);

    Status WriteBytes(uint16_t registerAddress, std::span<const uint8_t> bytes);
    Status ReadBytes(uint16_t registerAddress, std::span<uint8_t> buffer);

    Status WriteByte(uint16_t registerAddress, uint8_t byte);
    Status ReadByte(uint16_t registerAddress, uint8_t& byte);

private:
    I2cBus& m_bus;
    uint16_t m_address;
};
