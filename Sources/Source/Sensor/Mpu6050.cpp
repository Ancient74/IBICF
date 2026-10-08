#include "Mpu6050.h"
#include "stm32f4xx_hal.h"
#include <cmath>
#include "Config.h"
#include "SensorStatus.h"
#include "projdefs.h"

namespace
{
    constexpr uint16_t MPU6050_ADDRESS = 0x68;
    constexpr uint16_t MPU6050_WHO_AM_I_VALUE = 0x68;

    constexpr uint16_t MPU6050_SMPRT_DIV = 0X19;
    constexpr uint16_t MPU6050_WHO_AM_I = 0X75;
    constexpr uint16_t MPU6050_CONFIG = 0X1A;
    constexpr uint16_t MPU6050_GYRO_CONFIG = 0X1B;
    constexpr uint16_t MPU6050_ACCEL_CONFIG = 0X1C;
    constexpr uint16_t MPU6050_ACCEL_XOUT_H = 0X3B;
    constexpr uint16_t MPU6050_PWR_MGMT_1 = 0X6B;

    constexpr float ACCELEROMETER_BIAS[3] = { 1.926794f, -10.027902f, 3.518213f };
    constexpr float ACCELEROMETER_MATRIX[3][3] = {
        { 0.989325f, 0.007474f, 0.009166f },
        { 0.007474f, 1.010914f, 0.010260f },
        { 0.009166f, 0.010260f, 0.987679f },
    };

    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    uint16_t GetAccelerometerResolution(uint8_t fullScaleSelect)
    {
        switch (fullScaleSelect)
        {
        case 0b00:
            return 16384;
            break;
        case 0b01:
            return 8192;
        case 0b10:
            return 4096;
        case 0b11:
            return 2048;
        default:
            return 0;
        }
    }

    float GetGyroResolution(uint8_t fullScaleSelect)
    {
        switch (fullScaleSelect)
        {
        case 0b00:
            return 131.0f;
            break;
        case 0b01:
            return 65.5f;
        case 0b10:
            return 32.8f;
        case 0b11:
            return 16.4f;
        default:
            return 0.0f;
        }
    }

    union PowerManagementRegister
    {
        struct
        {
            uint8_t clockSelect : 3;
            uint8_t tempDisable : 1;
            uint8_t empty       : 1;
            uint8_t cycle       : 1;
            uint8_t sleep       : 1;
            uint8_t deviceReset : 1;
        } bits;
        uint8_t value;
    };
    static_assert(sizeof(PowerManagementRegister) == sizeof(uint8_t));

    union ConfigRegister
    {
        struct
        {
            uint8_t dlpfConfig : 3;
            uint8_t extSyncSet : 3;
            uint8_t empty : 2;
        } bits;
        uint8_t value;
    };
    static_assert(sizeof(ConfigRegister) == sizeof(uint8_t));

    union GyroAccelConfigRegister
    {
        struct
        {
            uint8_t empty : 3;
            uint8_t fullScaleSelect : 2;
            uint8_t zSelfTest : 1;
            uint8_t ySelfTest : 1;
            uint8_t xSelfTest : 1;

        } bits;
        uint8_t value;
    };
    static_assert(sizeof(GyroAccelConfigRegister) == sizeof(uint8_t));

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

    Vector3f CorrectAccelerometer(const Accelerometer& acc, float accelerometerResolution)
    {
        Vector3f corrected{};

        const float bx = acc.x - ACCELEROMETER_BIAS[0];
        const float by = acc.y - ACCELEROMETER_BIAS[1];
        const float bz = acc.z - ACCELEROMETER_BIAS[2];

        corrected(0, 0) = (ACCELEROMETER_MATRIX[0][0] * bx + ACCELEROMETER_MATRIX[0][1] * by + ACCELEROMETER_MATRIX[0][2] * bz) / accelerometerResolution;
        corrected(1, 0) = (ACCELEROMETER_MATRIX[1][0] * bx + ACCELEROMETER_MATRIX[1][1] * by + ACCELEROMETER_MATRIX[1][2] * bz) / accelerometerResolution;
        corrected(2, 0) = (ACCELEROMETER_MATRIX[2][0] * bx + ACCELEROMETER_MATRIX[2][1] * by + ACCELEROMETER_MATRIX[2][2] * bz) / accelerometerResolution;

        return corrected;
    }
}

Mpu6050::Mpu6050(I2C_HandleTypeDef& i2c, SemaphoreHandle_t mutex) : I2cSensor(i2c, mutex, MPU6050_ADDRESS)
{}

Status Mpu6050::Start()
{
    uint8_t whoAmI;
    auto ret = ReadByte(MPU6050_WHO_AM_I, whoAmI);
    if (ret != Status::Ok)
        return ret;
    if (whoAmI != MPU6050_WHO_AM_I_VALUE)
        return Status::Error;

    PowerManagementRegister powerManagement{};
    powerManagement.bits.deviceReset = 1;
    ret = WriteByte(MPU6050_PWR_MGMT_1, powerManagement.value);
    if (ret != Status::Ok)
        return ret;
    auto start = xTaskGetTickCount();
    while(true)
    {
        ret = ReadByte(MPU6050_PWR_MGMT_1, powerManagement.value);
        auto now = xTaskGetTickCount();
        if ((now - start) > pdMS_TO_TICKS(Config::MPU_RESET_TIMEOUT))
            return Status::Timeout;
        if (powerManagement.bits.deviceReset == 0 && ret == Status::Ok)
            break;
        vTaskDelay(1);
    }

    powerManagement = {};
    powerManagement.bits.tempDisable = 1;
    powerManagement.bits.clockSelect = 1;
    ret = WriteByte(MPU6050_PWR_MGMT_1, powerManagement.value);
    if (ret != Status::Ok)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    ConfigRegister config{};
    config.bits.dlpfConfig = DlpfValue;
    ret = WriteByte(MPU6050_CONFIG, config.value);
    if (ret != Status::Ok)
        return ret;

    // If DLPF config is 0 then gyro rate is 8kHz
    // Else it is set to 1kHz
    // We want a prescaler to get 1kHz sampling rate
    // Which matches accelerometer sampling rate
    ret = WriteByte(MPU6050_SMPRT_DIV, ClockPrescaler);
    if (ret != Status::Ok)
        return ret;

    GyroAccelConfigRegister gyroConfig{};
    gyroConfig.bits.fullScaleSelect = 0b01;
    ret = WriteByte(MPU6050_GYRO_CONFIG, gyroConfig.value);
    if (ret != Status::Ok)
        return ret;
    m_gyroResolution = GetGyroResolution(gyroConfig.bits.fullScaleSelect);

    GyroAccelConfigRegister accelConfig{};
    accelConfig.bits.fullScaleSelect = 0b11;
    ret = WriteByte(MPU6050_ACCEL_CONFIG, accelConfig.value);
    if (ret != Status::Ok)
        return ret;
    m_accelerometerResolution = GetAccelerometerResolution(accelConfig.bits.fullScaleSelect);
    return ret;
}

Status Mpu6050::GetRawValues(GyroscopeAccelerometer& ga)
{
    uint8_t data[14];
	const auto ret = ReadBytes(MPU6050_ACCEL_XOUT_H, data);
    if (ret != Status::Ok)
        return ret;
    ValueRegister a[3];
    ValueRegister g[3];

    for(auto i = 0; i < 3; i++)
    {
        a[i].highLow.high = data[i*2];
        a[i].highLow.low = data[i*2 + 1];
        // Skip temperature sensor
        g[i].highLow.high = data[i*2 + 8];
        g[i].highLow.low = data[i*2 + 8 + 1];
    }
    ga.acc.x = a[0].value;
    ga.acc.y = a[1].value;
    ga.acc.z = a[2].value;

    ga.gyro.x = g[0].value;
    ga.gyro.y = g[1].value;
    ga.gyro.z = g[2].value;
    return ret;
}

void Mpu6050::Normalize(GyroscopeAccelerometer& ga)
{
    const auto corrected = CorrectAccelerometer(ga.acc, m_accelerometerResolution);
    ga.acc.x = corrected(0, 0);
    ga.acc.y = corrected(1, 0);
    ga.acc.z = corrected(2, 0);

    ga.gyro.x = (ga.gyro.x - m_gyroCalibration[0]) / m_gyroResolution * DEG_TO_RAD;
    ga.gyro.y = (ga.gyro.y - m_gyroCalibration[1]) / m_gyroResolution * DEG_TO_RAD;
    ga.gyro.z = (ga.gyro.z - m_gyroCalibration[2]) / m_gyroResolution * DEG_TO_RAD;
}

void Mpu6050::Calibrate(Matrix3f& R, Matrix3f& N)
{
    constexpr uint16_t calibrationSamples = 1000;

    Welford gx, gy, gz;
    Welford ax, ay, az;

    for (uint16_t i = 0; i < calibrationSamples; i++)
    {
        GyroscopeAccelerometer ga{};
        if (GetRawValues(ga) != Status::Ok)
            continue;

        const auto corrected = CorrectAccelerometer(ga.acc, m_accelerometerResolution);
        Vector3f accel{};
        accel(0, 0) = corrected(0, 0);
        accel(1, 0) = corrected(1, 0);
        accel(2, 0) = corrected(2, 0);
        accel = ::Normalize(accel);

        ax.Add(accel(0, 0));
        ay.Add(accel(1, 0));
        az.Add(accel(2, 0));

        gx.Add(ga.gyro.x);
        gy.Add(ga.gyro.y);
        gz.Add(ga.gyro.z);

        HAL_Delay(1);
    }

    if (gx.n == 0)
        return;

    m_gyroCalibration[0] = gx.Mean();
    m_gyroCalibration[1] = gy.Mean();
    m_gyroCalibration[2] = gz.Mean();

    const float gyroScale = DEG_TO_RAD / m_gyroResolution;
    N(0, 0) = gx.Variance() * gyroScale * gyroScale;
    N(1, 1) = gy.Variance() * gyroScale * gyroScale;
    N(2, 2) = gz.Variance() * gyroScale * gyroScale;

    R(0, 0) = ax.Variance();
    R(1, 1) = ay.Variance();
    R(2, 2) = az.Variance();
}
