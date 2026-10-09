#pragma once
#include "I2cSensor.h"
#include "KalmanLib/Matrix.hpp"

struct Magnetometer
{
    float x;
    float y;
    float z;
};

inline Magnetometer operator+(const Magnetometer& a, const Magnetometer& b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Magnetometer operator*(const Magnetometer& a, float mult)
{
    return { a.x * mult, a.y * mult, a.z * mult };
}

class Qmc5883L : private I2cSensor
{
public:
    Qmc5883L(I2cBus& bus);

    Status Start();

    Status GetRawValues(Magnetometer& magnetometer);
    void Normalize(Magnetometer& magnetometer);
    void GetSensorVariance(Matrix3f& R);
    static constexpr uint16_t GetSamplingRate() noexcept
    {
        switch (SamplingRateSetting)
        {
        case 0b00: return 10;
        case 0b01: return 50;
        case 0b10: return 100;
        case 0b11: return 200;
        default:   return 10;
        }
    }

private:
    static constexpr uint16_t SamplingRateSetting = 0b10;
    float m_resolution;
};
