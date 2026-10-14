#include "AdcSensor.h"
#include "Config.h"

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*)
{
    if (auto* self = AdcSensor::GetInstance())
        self->ConversionComplete();
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*)
{
    if (auto* self = AdcSensor::GetInstance())
        self->ConversionError();
}

AdcSensor::AdcSensor(ADC_HandleTypeDef& handle, uint32_t channel)
    : m_handle(handle)
    , m_channel(channel)
{
    configASSERT(s_instance == nullptr && "AdcSensor is a singleton");

    m_completeSem = xSemaphoreCreateBinaryStatic(&m_completeSemBuffer);
    configASSERT(m_completeSem != nullptr);

    s_instance = this;
}

AdcSensor::~AdcSensor()
{
    s_instance = nullptr;
    HAL_ADC_Stop_DMA(&m_handle);
    vSemaphoreDelete(m_completeSem);
}

AdcSensor* AdcSensor::GetInstance()
{
    return s_instance;
}

Status AdcSensor::StartDmaTransactionAndWait(std::span<uint16_t> buffer)
{
    m_error = false;
    xSemaphoreTake(m_completeSem, 0); // drop any stale token

    const auto status = FromHAL(HAL_ADC_Start_DMA(&m_handle, reinterpret_cast<uint32_t*>(buffer.data()), static_cast<uint32_t>(buffer.size())));
    if (status != Status::Ok)
        return status;

    const bool done = xSemaphoreTake(m_completeSem, Config::ADC_READ_TIMEOUT_MS) == pdTRUE;
    HAL_ADC_Stop_DMA(&m_handle);

    if (!done)
        return Status::Busy;
    return m_error ? Status::Error : Status::Ok;
}

void AdcSensor::ConversionComplete()
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_completeSem, &woken);
    portYIELD_FROM_ISR(woken);
}

void AdcSensor::ConversionError()
{
    m_error = true;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_completeSem, &woken);
    portYIELD_FROM_ISR(woken);
}

uint32_t AdcSensor::GetAdcResolutionBits() const
{
    switch (m_handle.Init.Resolution)
    {
    case ADC_RESOLUTION_12B:
        return 12;
    case ADC_RESOLUTION_10B:
        return 10;
    case ADC_RESOLUTION_8B:
        return 8;
    case ADC_RESOLUTION_6B:
        return 6;

    default:
        return 12;
    }
}
