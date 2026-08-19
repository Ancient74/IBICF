#pragma once
#include "I2cSensor.h"
#include "KalmanLib/Matrix.hpp"

struct Gyroscope
{
    float x;
    float y;
    float z;
};

struct Accelerometer
{
    float x;
    float y;
    float z;
};

struct GyroscopeAccelerometer
{
    Gyroscope gyro;
    Accelerometer acc;
};

static GyroscopeAccelerometer operator+(const GyroscopeAccelerometer& a, const GyroscopeAccelerometer& b)
{
    GyroscopeAccelerometer res{};
    res.gyro.x = a.gyro.x + b.gyro.x;
    res.gyro.y = a.gyro.y + b.gyro.y;
    res.gyro.z = a.gyro.z + b.gyro.z;

    res.acc.x = a.acc.x + b.acc.x;
    res.acc.y = a.acc.y + b.acc.y;
    res.acc.z = a.acc.z + b.acc.z;

    return res;
}

static GyroscopeAccelerometer operator*(const GyroscopeAccelerometer& a, float mult)
{
    GyroscopeAccelerometer res = a;
    res.gyro.x *= mult;
    res.gyro.y *= mult;
    res.gyro.z *= mult;

    res.acc.x *= mult;
    res.acc.y *= mult;
    res.acc.z *= mult;

    return res;
}

class Mpu6050 : private I2cSensor
{
public:
    Mpu6050(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex);

    I2cSensor::Status Start();

    I2cSensor::Status GetRawValues(GyroscopeAccelerometer& ga);
    void Normalize(GyroscopeAccelerometer& ga);

    // R is noise covariance matrix from accelerometer
    // N is prediction covariance of angles based on gyro integration
    void Calibrate(Matrix3f& R, Matrix3f& N);

    uint16_t GetSamplingRate() const;

private:
    std::array<float, 3> m_gyroCalibration{};
    float m_accelerometerResolution;
    float m_gyroResolution;
};
