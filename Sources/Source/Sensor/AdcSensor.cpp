// AdcSensor.cpp
#include "AdcSensor.h"

namespace
{
    uint32_t GetChannelSamplingCycles(ADC_HandleTypeDef& hadc, uint32_t channel)
    {
        const bool inSmpr2 = channel <= 9;
        __IO uint32_t& smpr = inSmpr2 ? hadc.Instance->SMPR2 : hadc.Instance->SMPR1;
        const uint32_t shift = inSmpr2 ? (channel * 3U) : ((channel - 10U) * 3U);
        const uint32_t smpBits = (smpr >> shift) & 0x7U;

        switch (smpBits)
        {
            case ADC_SAMPLETIME_3CYCLES:   return 3U;
            case ADC_SAMPLETIME_15CYCLES:  return 15U;
            case ADC_SAMPLETIME_28CYCLES:  return 28U;
            case ADC_SAMPLETIME_56CYCLES:  return 56U;
            case ADC_SAMPLETIME_84CYCLES:  return 84U;
            case ADC_SAMPLETIME_112CYCLES: return 112U;
            case ADC_SAMPLETIME_144CYCLES: return 144U;
            case ADC_SAMPLETIME_480CYCLES: return 480U;
            default:                       return 0U;
        }
    }

    uint32_t GetResolutionClockCycles(ADC_HandleTypeDef& hadc)
    {
        switch (hadc.Init.Resolution)
        {
            case ADC_RESOLUTION_12B: return 12U;
            case ADC_RESOLUTION_10B: return 10U;
            case ADC_RESOLUTION_8B:  return 8U;
            case ADC_RESOLUTION_6B:  return 6U;
            default:                 return 12U;
        }
    }
}

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

AdcSensor::AdcSensor(ADC_HandleTypeDef& handle, uint32_t channel)
    : m_handle(handle)
    , m_channel(channel)
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

uint32_t AdcSensor::GetSamplingRate() const
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t adcClock = 0;

    switch (m_handle.Init.ClockPrescaler)
    {
        case ADC_CLOCK_SYNC_PCLK_DIV2: adcClock = pclk2 / 2; break;
        case ADC_CLOCK_SYNC_PCLK_DIV4: adcClock = pclk2 / 4; break;
        case ADC_CLOCK_SYNC_PCLK_DIV6: adcClock = pclk2 / 6; break;
        case ADC_CLOCK_SYNC_PCLK_DIV8: adcClock = pclk2 / 8; break;
        default: return 0;
    }

    const auto samplingCycles = GetChannelSamplingCycles(m_handle, m_channel);
    const auto resolutionClockCycles = GetResolutionClockCycles(m_handle);

    return adcClock / (samplingCycles + resolutionClockCycles);
}

void AdcSensor::ConversionError()
{
    m_error.store(true, std::memory_order_release);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m_completeSem, &woken);
    portYIELD_FROM_ISR(woken);
}


uint32_t AdcSensor::GetAdcResolutionBits() const
{
    return m_handle.Init.Resolution;
}