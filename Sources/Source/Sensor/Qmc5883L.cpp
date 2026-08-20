#include "Qmc5883L.h"
#include <KalmanLib/Matrix.hpp>

namespace
{
    constexpr uint16_t QMC5883L_ADDRESS = 0x0D;
    constexpr uint16_t QMC5883L_SET_RESET = 0x0B;
    constexpr uint16_t QMC5883L_CONTROL_REGISTER = 0x09;
    constexpr uint16_t QMC5883L_X_LSB_REGISTER = 0x00;
    constexpr uint16_t QMC5883L_SAMPLING_RATE = 0b10;

    constexpr float MAGNET_BIAS[3] = { 515.798978f, -47.838570f, -108.972277f };
    constexpr float MAGNET_MATRIX[3][3] = {
        { 1.241897f, -0.050880f, -0.096114f },
        { -0.050880f, 0.910678f, -0.104038f },
        { -0.096114f, -0.104038f, 1.028557f },
    };

    union ControlRegister
    {
        struct
        {
            uint8_t mode : 2;
            uint8_t outputDataRate : 2;
            uint8_t range : 2;
            uint8_t overSampleRatio: 2;
        } bits;
        uint8_t value;
    };
    static_assert(sizeof(ControlRegister) == sizeof(uint8_t));

    union ValueRegister
    {
        struct
        {
            uint8_t low;
            uint8_t high;
        } highLow;
        int16_t value;
    };
    static_assert(sizeof(ValueRegister) == sizeof(int16_t));

    float GetResolution(ControlRegister control)
    {
        switch (control.bits.range)
        {
        case 0b00:
            return 12000.0f;
        case 0b01:
            return 3000.0f;
        default:
            return 0.0f;
        }
    }

    Vector3f CorrectMagnet(const Magnetometer& magnet, float magnetResolution)
    {
        Vector3f corrected{};

        const float bx = magnet.x - MAGNET_BIAS[0];
        const float by = magnet.y - MAGNET_BIAS[1];
        const float bz = magnet.z - MAGNET_BIAS[2];

        corrected(0, 0) = (MAGNET_MATRIX[0][0] * bx + MAGNET_MATRIX[0][1] * by + MAGNET_MATRIX[0][2] * bz) / magnetResolution;
        corrected(1, 0) = (MAGNET_MATRIX[1][0] * bx + MAGNET_MATRIX[1][1] * by + MAGNET_MATRIX[1][2] * bz) / magnetResolution;
        corrected(2, 0) = (MAGNET_MATRIX[2][0] * bx + MAGNET_MATRIX[2][1] * by + MAGNET_MATRIX[2][2] * bz) / magnetResolution;

        return ::Normalize(corrected);
    }

}

Qmc5883L::Qmc5883L(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex) : I2cSensor(i2c, mutex, QMC5883L_ADDRESS)
{}

Status Qmc5883L::Start()
{
    // Documentation says that its recommended to set it to 0x01
    auto ret = WriteByte(QMC5883L_SET_RESET, 0x01);
    if (ret != Status::Ok)
        return ret;
    HAL_Delay(50);

    ControlRegister control{};
    control.bits.mode = 0b01;
    control.bits.outputDataRate = QMC5883L_SAMPLING_RATE; // 100Hz
    control.bits.range = 0b01; // 8G
    control.bits.overSampleRatio = 0b01; // 256
    ret = WriteByte(QMC5883L_CONTROL_REGISTER, control.value);
    if (ret != Status::Ok)
        return ret;
    m_resolution = GetResolution(control);
    HAL_Delay(50);
    return ret;
}

Status Qmc5883L::GetRawValues(Magnetometer& magnetometer)
{
    uint8_t data[6];
    ValueRegister m[3];

    const auto ret = ReadBytes(QMC5883L_X_LSB_REGISTER, data);
    if (ret != Status::Ok)
        return ret;

    for (auto i = 0; i < 3; i++)
    {
        m[i].highLow.low = data[i*2];
        m[i].highLow.high = data[i*2 + 1];
    }

    magnetometer.x = m[0].value;
    magnetometer.y = m[1].value;
    magnetometer.z = m[2].value;

    return ret;
}

void Qmc5883L::Normalize(Magnetometer& magnetometer)
{
    const auto corrected = CorrectMagnet(magnetometer, m_resolution);
    magnetometer.x = corrected(0,0);
    magnetometer.y = corrected(1,0);
    magnetometer.z = corrected(2,0);
}

uint16_t Qmc5883L::GetSamplingRate() const
{
    switch (QMC5883L_SAMPLING_RATE)
    {
    case 0b00:
        return 10;
    case 0b01:
        return 50;
    case 0b10:
        return 100;
    case 0b11:
        return 200;

    default:
        return 10;
    }
}

void Qmc5883L::GetSensorVariance(Matrix3f& R)
{
    constexpr uint16_t samples = 1000;

    Welford mx, my, mz;

    for (uint16_t i = 0; i < samples; i++)
    {
        Magnetometer m{};
        if (GetRawValues(m) != Status::Ok)
            continue;

        const auto corrected = CorrectMagnet(m, m_resolution);

        mx.Add(corrected(0, 0));
        my.Add(corrected(1, 0));
        mz.Add(corrected(2, 0));

        HAL_Delay(1);
    }

    if (mx.n == 0)
        return;

    R(0, 0) = mx.Variance();
    R(1, 1) = my.Variance();
    R(2, 2) = mz.Variance();
}
