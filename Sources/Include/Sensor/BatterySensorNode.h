#pragma once
#include "AdcSensor.h"
#include "FreeRTOSConfig.h"
#include "TaskThread.h"
#include "Config.h"
#include "Log.h"
#include "PubSubLib/Node.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

struct BatteryInfo
{
    float chargeLevel;
    uint8_t percentLevel;
};

template<float MaxBatteryVoltage, float MinBatteryVoltage, float AdcRefVoltage, float DividerRatio,
         uint32_t SamplingRateHz = Config::BATTERY_SAMPLING_RATE_HZ>
class BatterySensorNode
    : public NodeOutput<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, AdcRefVoltage, DividerRatio, SamplingRateHz>, BatteryInfo, 1>
{
    static_assert(MaxBatteryVoltage > MinBatteryVoltage,
        "MaxBatteryVoltage must be greater than MinBatteryVoltage");
    static_assert(SamplingRateHz > 0, "SamplingRateHz must be greater than zero");

public:
    BatterySensorNode(AdcSensor& adcSensor, const char* taskName)
        : m_thread(taskName, *this)
        , m_adcSensor(adcSensor)
    {
        const uint32_t adcConversionRateHz = m_adcSensor.GetSamplingRate();
        configASSERT(adcConversionRateHz > 0
            && "AdcSensor::GetSamplingRate() returned 0 - check ClockPrescaler config");

        const uint32_t maxSamples = adcConversionRateHz / SamplingRateHz;
        configASSERT(Config::BATTERY_ADC_OVERSAMPLING_RATIO <= maxSamples && "Oversampling burst doesn't fit in one sampling period");
        m_fullScale = static_cast<float>(((1u << m_adcSensor.GetAdcResolutionBits()) - 1u) << Config::BATTERY_EXTRA_BITS);
    }

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        const Status status = m_adcSensor.StartDmaTransactionAndWait(
            std::span<uint16_t>(m_buffer.data(), Config::BATTERY_ADC_OVERSAMPLING_RATIO));

        if (status != Status::Ok)
        {
            if (!m_readingFailed || status != m_lastStatus)
                LogWarning(m_thread.GetName(), "ADC conversion failed (status=%d)", static_cast<int>(status));
            m_readingFailed = true;
            m_lastStatus = status;
            return;
        }
        if (m_readingFailed)
            LogInfo(m_thread.GetName(), "ADC conversions recovered");
        m_readingFailed = false;
        m_lastStatus = Status::Ok;

        uint32_t sum = 0;
        for (uint32_t i = 0; i < Config::BATTERY_ADC_OVERSAMPLING_RATIO; ++i)
            sum += m_buffer[i];

        const uint32_t extendedReading = sum >> Config::BATTERY_EXTRA_BITS;

        const float pinVoltage = (static_cast<float>(extendedReading) / m_fullScale) * AdcRefVoltage;
        const float batteryVoltage = pinVoltage * DividerRatio;
        const float clampedVoltage = std::clamp(batteryVoltage, MinBatteryVoltage, MaxBatteryVoltage);
        const uint8_t percentLevel = static_cast<uint8_t>(clampedVoltage + 0.5f);

        this->Publish({batteryVoltage, percentLevel});
    }

private:
    TaskThread<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, AdcRefVoltage, DividerRatio, SamplingRateHz>,
        Config::Task::BATTERY_SENSOR_NODE_STACK_SIZE, Config::Task::BATTERY_SENSOR_NODE_PRIORITY,
        pdMS_TO_TICKS(1000 / SamplingRateHz)> m_thread;
    AdcSensor& m_adcSensor;
    Status m_lastStatus{Status::Ok};
    bool m_readingFailed{false};
    float m_fullScale;
    std::array<uint16_t, Config::BATTERY_ADC_OVERSAMPLING_RATIO> m_buffer{};
};
