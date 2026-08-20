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
    explicit AdcSensor(ADC_HandleTypeDef& handle);
    ~AdcSensor();

    AdcSensor(const AdcSensor&) = delete;
    AdcSensor& operator=(const AdcSensor&) = delete;

    Status StartDmaTransactionAndWait(std::span<uint16_t> buffer);

    static AdcSensor* GetInstance();

private:
    void ConvertionComplete();
    void ConversionError();

    friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
    friend void ::HAL_ADC_ErrorCallback(ADC_HandleTypeDef*);

    ADC_HandleTypeDef& m_handle;
    SemaphoreHandle_t m_completeSem{};
    std::atomic_bool m_error{false};

    static inline AdcSensor* s_instance = nullptr;
};
