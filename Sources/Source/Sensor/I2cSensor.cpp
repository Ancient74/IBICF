#include "I2cSensor.h"
#include "Log.h"
#include "stm32f4xx_hal_i2c.h"

namespace
{
    constexpr const char* MODULE = "I2C";
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
    m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);
    configASSERT(m_mutex);
}

I2cBus::~I2cBus()
{
    if (m_mutex != nullptr)
        vSemaphoreDelete(m_mutex);
}

void I2cBus::ScanBus()
{
    LogInfo(MODULE, "Scanning i2c bus");
    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        if (HAL_I2C_IsDeviceReady(&m_i2c
                                , static_cast<uint16_t>(address << 1)
                                , 2, 10) == HAL_OK)
        {
            LogInfo(MODULE, "Found module at address: 0x%02X", address);
        }
    }
}

I2cSensor::I2cSensor(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex, uint16_t address)
    : m_i2c(i2c)
    , m_mutex(mutex)
    , m_address(address << 1)
{}

Status I2cSensor::WriteBytes(uint16_t registerAddress, std::span<const uint8_t> bytes)
{
    auto lock = I2cBusLock(m_mutex);
    return FromHAL(HAL_I2C_Mem_Write(&m_i2c, m_address, registerAddress, I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(bytes.data()), bytes.size(), 100));
}

Status I2cSensor::ReadBytes(uint16_t registerAddress, std::span<uint8_t> buffer)
{
    auto lock = I2cBusLock(m_mutex);
    return FromHAL(HAL_I2C_Mem_Read(&m_i2c, m_address, registerAddress, I2C_MEMADD_SIZE_8BIT, buffer.data(), buffer.size(), 100));
}

Status I2cSensor::WriteByte(uint16_t registerAddress, uint8_t byte)
{
    std::array<uint8_t, 1> buffer{};
    buffer[0] = byte;
    return WriteBytes(registerAddress, buffer);
}

Status I2cSensor::ReadByte(uint16_t registerAddress, uint8_t& byte)
{
    std::array<uint8_t, 1> buffer{};
    const auto result = ReadBytes(registerAddress, buffer);
    byte = buffer[0];
    return result;
}
