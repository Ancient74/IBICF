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
    Qmc5883L(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex);

    Status Start();

    Status GetRawValues(Magnetometer& magnetometer);
    void Normalize(Magnetometer& magnetometer);
    void GetSensorVariance(Matrix3f& R);
    uint16_t GetSamplingRate() const;

private:
    float m_resolution;
};
