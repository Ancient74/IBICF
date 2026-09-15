#pragma once
#include "AdcSensor.h"
#include "TaskThread.h"
#include "Config.h"
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

template<float MaxBatteryVoltage, float MinBatteryVoltage, float AdcInMaxVoltage>
class BatterySensorNode
    : public NodeOutput<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, AdcInMaxVoltage>, BatteryInfo, 1>
{
    static_assert(MaxBatteryVoltage > MinBatteryVoltage,
        "MaxBatteryVoltage must be greater than MinBatteryVoltage");

public:
    BatterySensorNode(AdcSensor& adcSensor, const char* taskName, uint32_t samplingRateHz)
        : m_thread(taskName, pdMS_TO_TICKS(1000 / samplingRateHz), *this)
        , m_adcSensor(adcSensor)
    {
        const uint32_t adcConversionRateHz = m_adcSensor.GetSamplingRate();
        configASSERT(adcConversionRateHz > 0
            && "AdcSensor::GetSamplingRate() returned 0 - check ClockPrescaler config");

        const float maxSamplesF = (static_cast<float>(adcConversionRateHz) / static_cast<float>(samplingRateHz));
        const uint32_t maxSamples = static_cast<uint32_t>(maxSamplesF);

        uint32_t ratio = 1;
        uint32_t extraBits = 0;
        while (extraBits < Config::BATTERY_MAX_USEFUL_EXTRA_BITS && ratio * 4 <= maxSamples)
        {
            ratio *= 4;
            ++extraBits;
        }

        configASSERT(ratio >= 4
            && "samplingRateHz too high for this ADC's conversion rate: "
               "not even one extra bit of oversampling fits in the period");

        m_oversampleCount = ratio;
        m_extraBits = extraBits;
        m_effectiveMaxCode = (1u << (m_adcSensor.GetAdcResolutionBits() + extraBits)) - 1u;
    }

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        const Status status = m_adcSensor.StartDmaTransactionAndWait(
            std::span<uint16_t>(m_buffer.data(), m_oversampleCount));

        if (status != Status::Ok)
            return;

        uint32_t sum = 0;
        for (uint32_t i = 0; i < m_oversampleCount; ++i)
            sum += m_buffer[i];

        const uint32_t extendedCode = sum >> m_extraBits;

        const float adcVoltage =
            (static_cast<float>(extendedCode) / static_cast<float>(m_effectiveMaxCode))
            * AdcInMaxVoltage;

        const float batteryVoltage = std::clamp(
            adcVoltage * (MaxBatteryVoltage / AdcInMaxVoltage),
            MinBatteryVoltage, MaxBatteryVoltage);

        constexpr float voltageRange = MaxBatteryVoltage - MinBatteryVoltage;
        const float chargePercentRaw =
            (batteryVoltage - MinBatteryVoltage) / voltageRange * 100.0f;
        const float chargePercentClamped = std::clamp(chargePercentRaw, 0.0f, 100.0f);
        const uint8_t percentLevel = static_cast<uint8_t>(chargePercentClamped + 0.5f);

        this->Publish({batteryVoltage, percentLevel});
    }

private:
    TaskThread<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, AdcInMaxVoltage>, 128, configMAX_PRIORITIES / 2 - 2> m_thread;
    AdcSensor& m_adcSensor;
    std::array<uint16_t, Config::BATTERY_ADC_OVERSAMPLING_RATIO> m_buffer{};
    uint32_t m_oversampleCount = 0;
    uint32_t m_extraBits = 0;
    uint32_t m_effectiveMaxCode = 0;
};
