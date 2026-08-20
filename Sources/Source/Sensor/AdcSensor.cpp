// AdcSensor.cpp
#include "AdcSensor.h"

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*)
{
    if (auto* self = AdcSensor::GetInstance())
        self->ConvertionComplete();
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*)
{
    if (auto* self = AdcSensor::GetInstance())
        self->ConversionError();
}

AdcSensor::AdcSensor(ADC_HandleTypeDef& handle)
    : m_handle(handle)
{
    configASSERT(s_instance == nullptr && "AdcSensor is a singleton");
    configASSERT((m_completeSem = xSemaphoreCreateBinary()) != nullptr);
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
    m_error.store(false, std::memory_order_relaxed);

    auto res = FromHAL(HAL_ADC_Start_DMA(&m_handle, reinterpret_cast<uint32_t*>(buffer.data()), buffer.size()));
    if (res != Status::Ok)
        return res;

    xSemaphoreTake(m_completeSem, portMAX_DELAY);

    return m_error.load(std::memory_order_relaxed) ? Status::Error : Status::Ok;
}

void AdcSensor::ConvertionComplete()
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_completeSem, &woken);
    portYIELD_FROM_ISR(woken);
}

void AdcSensor::ConversionError()
{
    m_error.store(true, std::memory_order_release);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_completeSem, &woken);
    portYIELD_FROM_ISR(woken);
}
