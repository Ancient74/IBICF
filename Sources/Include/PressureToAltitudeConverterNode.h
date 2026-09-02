#pragma once
#include "PubSubLib/Node.hpp"
#include "BME280.h"

template<uint8_t MaxSubs>
class PressureToAltitudeConverterNode
    : public NodeInput<PressureToAltitudeConverterNode<MaxSubs>, BME280::Measurements>
    , public NodeOutput<PressureToAltitudeConverterNode<MaxSubs>, float, MaxSubs>
{
public:
    void SetBaseMeasurements(uint32_t basePressure, uint32_t baseTemperature)
    {
        m_basePressure = static_cast<float>(basePressure) / 256.0f;
        m_baseTemperature = (static_cast<float>(baseTemperature) / 100.0f) + 273.15f;
    }

    void Process(const BME280::Measurements& measurements)
    {
        static constexpr float temperatureLapseRate = -0.0065;
        static constexpr float gasConstant = 8.31432;
        static constexpr float g = 9.80665;
        static constexpr float molarMassOfAir = 0.0289644;
        static constexpr float exponent = -gasConstant * temperatureLapseRate / (g * molarMassOfAir);
        const float pressure = static_cast<float>(measurements.pressure) / 256.0f;

        const float altitude = (m_baseTemperature / temperatureLapseRate) * (std::pow(pressure / m_basePressure, exponent) - 1.0f);

        this->Publish(altitude);
    }

private:
    float m_basePressure = 0.0f;
    float m_baseTemperature = 0.0f;
};
