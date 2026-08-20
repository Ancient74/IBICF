#pragma once
#include "FreeRTOS.h"
#include <stdint.h>
#include <memory>
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

    template<typename T>
    std::unique_ptr<T> CreateSensor()
    {
        return std::make_unique<T>(m_i2c, m_mutex);
    }

private:
    I2C_HandleTypeDef& m_i2c;
    SemaphoreHandle_t m_mutex;
};

class I2cSensor
{
public:
    I2cSensor(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex, uint16_t address);

    Status WriteBytes(uint16_t registerAddress, std::span<const uint8_t> bytes);
    Status ReadBytes(uint16_t registerAddress, std::span<uint8_t> buffer);

    Status WriteByte(uint16_t registerAddress, uint8_t byte);
    Status ReadByte(uint16_t registerAddress, uint8_t& byte);

private:
    I2C_HandleTypeDef& m_i2c;
    SemaphoreHandle_t m_mutex;
    uint16_t m_address;
};

struct Welford
{
    uint32_t n = 0;
    float mean = 0.0f;
    float M2 = 0.0f;

    void Add(float x)
    {
        ++n;
        const float delta = x - mean;
        mean += delta / static_cast<float>(n);
        M2 += delta * (x - mean);
    }

    float Mean() const { return mean; }
    float Variance() const { return n > 1 ? M2 / static_cast<float>(n - 1) : 0.0f; }
};
