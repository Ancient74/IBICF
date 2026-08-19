#include "I2cSensor.h"

namespace
{
    I2cSensor::Status FromHAL(HAL_StatusTypeDef halStatus)
    {
        switch (halStatus)
        {
        case HAL_StatusTypeDef::HAL_OK:
            return I2cSensor::Status::Ok;
        case HAL_StatusTypeDef::HAL_BUSY:
            return I2cSensor::Status::Busy;
        case HAL_StatusTypeDef::HAL_ERROR:
            return I2cSensor::Status::Error;
        case HAL_StatusTypeDef::HAL_TIMEOUT:
            return I2cSensor::Status::Timeout;

        default:
            return I2cSensor::Status::Error;
        }
    }
}

I2cBusLock::I2cBusLock(SemaphoreHandle_t m)
    : m_mutex(m)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
}

I2cBusLock::~I2cBusLock()
{
    xSemaphoreGive(m_mutex);
}

I2cBus::I2cBus(I2C_HandleTypeDef& i2c)
    : m_i2c(i2c)
{
    m_mutex = xSemaphoreCreateMutex();
    configASSERT(m_mutex);
}

I2cBus::~I2cBus()
{
    if (m_mutex != nullptr)
        vSemaphoreDelete(m_mutex);
}

I2cSensor::I2cSensor(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex, uint16_t address)
    : m_i2c(i2c)
    , m_mutex(mutex)
    , m_address(address << 1)
{}

I2cSensor::Status I2cSensor::WriteBytes(uint16_t registerAddress, std::span<const uint8_t> bytes)
{
    auto lock = I2cBusLock(m_mutex);
    return FromHAL(HAL_I2C_Mem_Write(&m_i2c, m_address, registerAddress, I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(bytes.data()), bytes.size(), 100));
}

I2cSensor::Status I2cSensor::ReadBytes(uint16_t registerAddress, std::span<uint8_t> buffer)
{
    auto lock = I2cBusLock(m_mutex);
    return FromHAL(HAL_I2C_Mem_Read(&m_i2c, m_address, registerAddress, I2C_MEMADD_SIZE_8BIT, buffer.data(), buffer.size(), 100));
}

I2cSensor::Status I2cSensor::WriteByte(uint16_t registerAddress, uint8_t byte)
{
    std::array<uint8_t, 1> buffer{};
    buffer[0] = byte;
    return WriteBytes(registerAddress, buffer);
}

I2cSensor::Status I2cSensor::ReadByte(uint16_t registerAddress, uint8_t& byte)
{
    std::array<uint8_t, 1> buffer{};
    const auto result = ReadBytes(registerAddress, buffer);
    byte = buffer[0];
    return result;
}
