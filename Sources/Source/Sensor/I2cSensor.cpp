#include "I2cSensor.h"
#include "Log.h"
#include "stm32f4xx_hal_i2c.h"
#include "Config.h"

namespace
{
    constexpr const char* MODULE = "I2C";

    uint8_t GetI2cId(I2C_HandleTypeDef* hi2c)
    {
        if (hi2c->Instance == I2C1) return 1;
        if (hi2c->Instance == I2C2) return 2;
        if (hi2c->Instance == I2C3) return 3;
        return 0;
    }
}

extern "C" void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    const auto instance = I2cBus::FindInstance(hi2c);
    if (instance)
        instance->OnComplete(false);
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    const auto instance = I2cBus::FindInstance(hi2c);
    if (instance)
        instance->OnComplete(false);
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    const auto instance = I2cBus::FindInstance(hi2c);
    if (instance)
        instance->OnComplete(true);
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

I2cBus* I2cBus::FindInstance(I2C_HandleTypeDef* hi2c)
{
    for (auto instance : s_instances)
    {
        if (instance && &instance->m_i2c == hi2c)
            return instance;
    }
    return nullptr;
}

I2cBus::I2cBus(I2C_HandleTypeDef& i2c)
    : m_i2c(i2c)
{
    for (uint8_t i = 0; i < s_instanceCount; i++)
    {
        if (s_instances[i] == nullptr)
        {
            s_instances[i] = this;
            break;
        }
    }
    configASSERT(FindInstance(&i2c) != nullptr);
    m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);
    configASSERT(m_mutex);

    m_doneSemaphore = xSemaphoreCreateBinaryStatic(&m_doneSemaphoreBuffer);
    configASSERT(m_doneSemaphore);
}

I2cBus::~I2cBus()
{
    if (m_mutex != nullptr)
        vSemaphoreDelete(m_mutex);

    if (m_doneSemaphore != nullptr)
        vSemaphoreDelete(m_doneSemaphore);
}

void I2cBus::ScanBus()
{
    I2cBusLock lock{m_mutex};
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

Status I2cBus::WriteBytes(uint16_t deviceAddress, uint16_t registerAddress, std::span<const uint8_t> bytes)
{
    return Transfer(HAL_I2C_Mem_Write_DMA, deviceAddress, registerAddress, { const_cast<uint8_t*>(bytes.data()), bytes.size() });
}

Status I2cBus::ReadBytes(uint16_t deviceAddress, uint16_t registerAddress, std::span<uint8_t> buffer)
{
    return Transfer(HAL_I2C_Mem_Read_DMA, deviceAddress, registerAddress, buffer);
}

void I2cBus::OnComplete(bool failed)
{
    m_failed = failed;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(m_doneSemaphore, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

Status I2cBus::Transfer(HAL_StatusTypeDef(*TransferFunction)(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
    , uint16_t dev, uint16_t reg, std::span<uint8_t> buf)
{
    I2cBusLock lock{m_mutex};
    Status last = Status::Error;

    for (uint8_t i = 0; i < Config::I2C_RETRY_ATTEMPTS; i++)
    {
        xSemaphoreTake(m_doneSemaphore, 0);   // drop stale signal
        m_failed = false;

        if (FromHAL(TransferFunction(&m_i2c, dev, reg, I2C_MEMADD_SIZE_8BIT, buf.data(), buf.size())) != Status::Ok)
        {
            last = Status::Busy;
            Recover();
            continue;
        }

        if (xSemaphoreTake(m_doneSemaphore, pdMS_TO_TICKS(Config::I2C_READ_WRITE_TIMEOUT)) != pdTRUE)
        {
            LogError(MODULE, "I2C timeout (instance=%d, attempt=%d)", GetI2cId(&m_i2c), i + 1);
            last = Status::Busy;
            Recover();
            continue;
        }

        if (!m_failed)
        {
            if (i > 0)
                LogWarning(MODULE, "I2C recovered (instance=%d, dev=0x%02X, after %d failed attempt(s))",
                        GetI2cId(&m_i2c), dev >> 1, i);
            return Status::Ok;
        }

        const auto err = HAL_I2C_GetError(&m_i2c);
        LogError(MODULE, "I2C transfer failed (instance=%d, error=0x%02lX, attempt=%d)",
                 GetI2cId(&m_i2c), err, i + 1);
        last = Status::Error;

        if (err == HAL_I2C_ERROR_AF)
            vTaskDelay(pdMS_TO_TICKS(2)); // NACK: device busy or absent, bus is fine
        else
            Recover();            // BERR / ARLO / OVR
    }
    LogError(MODULE, "I2C transfer FAILED after %d attempts (instance=%d, dev=0x%02X, reg=0x%02X, last status=%d)",
             Config::I2C_RETRY_ATTEMPTS, GetI2cId(&m_i2c), dev >> 1, reg, static_cast<int>(last));
    return last;
}

void I2cBus::Recover()
{
    if (m_i2c.hdmarx) HAL_DMA_Abort(m_i2c.hdmarx);
    if (m_i2c.hdmatx) HAL_DMA_Abort(m_i2c.hdmatx);
    HAL_I2C_DeInit(&m_i2c);
    HAL_I2C_Init(&m_i2c);
}

I2cSensor::I2cSensor(I2cBus& bus, uint16_t address)
    : m_bus(bus)
    , m_address(address << 1)
{}

Status I2cSensor::WriteBytes(uint16_t registerAddress, std::span<const uint8_t> bytes)
{
    return m_bus.WriteBytes(m_address, registerAddress, bytes);
}

Status I2cSensor::ReadBytes(uint16_t registerAddress, std::span<uint8_t> buffer)
{
    return m_bus.ReadBytes(m_address, registerAddress, buffer);
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
