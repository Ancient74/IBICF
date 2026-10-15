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

template<float MaxBatteryVoltage, float MinBatteryVoltage, float DividerRatio,
         uint32_t SamplingRateHz = Config::BATTERY_SAMPLING_RATE_HZ>
class BatterySensorNode
    : public NodeOutput<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, DividerRatio, SamplingRateHz>, BatteryInfo, 1>
{
    static_assert(MaxBatteryVoltage > MinBatteryVoltage,
        "MaxBatteryVoltage must be greater than MinBatteryVoltage");
    static_assert(SamplingRateHz > 0, "SamplingRateHz must be greater than zero");

public:
    BatterySensorNode(AdcSensor& adcSensor, const char* taskName)
        : m_thread(taskName, *this)
        , m_adcSensor(adcSensor)
    {
        m_adcMax = static_cast<float>((1u << m_adcSensor.GetAdcResolutionBits()) - 1u);
        m_vrefCalibration = static_cast<float>(*VREFINT_CAL_ADDR) * m_adcMax / Config::BATTERY_VREF_CALIBRATION_MAX_VALUE;
    }

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        const Status status = m_adcSensor.StartDmaTransactionAndWait(
            std::span<uint16_t>(m_buffer.data(), m_buffer.size()));

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

        uint32_t sumBattery = 0;
        uint32_t sumVref = 0;
        for (uint32_t i = 0; i < Config::BATTERY_ADC_OVERSAMPLING_RATIO; i++)
        {
            sumBattery += m_buffer[2 * i];
            sumVref += m_buffer[2 * i + 1];
        }
        float vdda = (VREFINT_CAL_VREF / 1000.0f);
        if (sumVref != 0)
        {
            const float vrefRaw = static_cast<float>(sumVref) / Config::BATTERY_ADC_OVERSAMPLING_RATIO;
            vdda *= m_vrefCalibration  / vrefRaw;
        }
        const float batteryRaw = static_cast<float>(sumBattery) / static_cast<float>(Config::BATTERY_ADC_OVERSAMPLING_RATIO);
        const float pinVoltage = (batteryRaw / m_adcMax) * vdda;
        const float batteryVoltage = pinVoltage * DividerRatio;
        const float clampedVoltage = std::clamp(batteryVoltage, MinBatteryVoltage, MaxBatteryVoltage);
        const float chargeFraction = (clampedVoltage - MinBatteryVoltage) / (MaxBatteryVoltage - MinBatteryVoltage);
        const uint8_t percentLevel = static_cast<uint8_t>(std::round(chargeFraction * 100.0f));

        this->Publish({batteryVoltage, percentLevel});
    }

private:
    TaskThread<BatterySensorNode<MaxBatteryVoltage, MinBatteryVoltage, DividerRatio, SamplingRateHz>,
        Config::Task::BATTERY_SENSOR_NODE_STACK_SIZE, Config::Task::BATTERY_SENSOR_NODE_PRIORITY,
        pdMS_TO_TICKS(1000 / SamplingRateHz)> m_thread;
    AdcSensor& m_adcSensor;
    Status m_lastStatus{Status::Ok};
    bool m_readingFailed{false};
    float m_adcMax;
    float m_vrefCalibration;
    std::array<uint16_t, Config::BATTERY_ADC_OVERSAMPLING_RATIO * 2> m_buffer{};
};
