#pragma once
#include "adc.h"
#include <span>
#include <FreeRTOS.h>
#include <semphr.h>
#include "SensorStatus.h"
#include <atomic>

class AdcSensor
{
public:
    explicit AdcSensor(ADC_HandleTypeDef& handle, uint32_t channel);
    ~AdcSensor();

    AdcSensor(const AdcSensor&) = delete;
    AdcSensor& operator=(const AdcSensor&) = delete;

    Status StartDmaTransactionAndWait(std::span<uint16_t> buffer);

    uint32_t GetAdcResolutionBits() const;

    static AdcSensor* GetInstance();

private:
    void ConversionComplete();
    void ConversionError();

    friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
    friend void ::HAL_ADC_ErrorCallback(ADC_HandleTypeDef*);

    ADC_HandleTypeDef& m_handle;
    SemaphoreHandle_t m_completeSem{};
    StaticSemaphore_t m_completeSemBuffer{};
    std::atomic_bool m_error{false};

    uint32_t m_channel;

    static inline AdcSensor* s_instance = nullptr;
};
