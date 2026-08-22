#pragma once
#include "I2cSensor.h"
#include "math.h"

struct Raw20
{
    uint8_t msb;
    uint8_t lsb;
    uint8_t xlsb; // only top 4 bits significant

    [[nodiscard]] constexpr uint32_t value() const noexcept
    {
        return (static_cast<uint32_t>(msb)  << 12)
             | (static_cast<uint32_t>(lsb)  << 4)
             | (xlsb >> 4);
    }
};

struct Raw16
{
    uint8_t msb;
    uint8_t lsb;

    [[nodiscard]] constexpr uint32_t value() const noexcept
    {
        return static_cast<uint32_t>((msb << 8) | lsb);
    }
};

class BME280 : private I2cSensor
{
public:
#pragma pack(push, 1)
    struct CompensationParameters1
    {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
        uint8_t reserved;
        uint8_t dig_H1;
    };
    static_assert(sizeof(CompensationParameters1) == 26);

    struct CompensationParameters2
    {
        int16_t dig_H2;
        uint8_t dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t dig_H6;
    };
#pragma pack(pop)
    static_assert(sizeof(CompensationParameters2) == 8);

    struct Measurements
    {
        uint32_t pressure;
        uint32_t temperature;
        uint32_t humidity;
    };

    // 0x76 if SDO is tied low, 0x77 if SDO is tied high.
    static constexpr uint16_t kDefaultAddress = 0x76;

    BME280(I2C_HandleTypeDef& handle, SemaphoreHandle_t mutex, uint16_t address = kDefaultAddress);

    Status Start();

    Status GetRawValues(Measurements& measurements);
    void Normalize(Measurements& measurements);

    void Calibrate(Measurements& baseMeasurements);

    uint16_t GetSamplingRate() const;

private:
    struct BmeData
    {
        Raw20 pressure;
        Raw20 temperature;
        Raw16 humidity;
    };
    static_assert(sizeof(BmeData) == 8);

    Status ReadCompensationData();
    Status ReadBmeData(BmeData& data);

    uint32_t CompensateTemperature(Raw20 rawTemp);
    uint32_t CompensatePressure(Raw20 rawPressure);
    uint32_t CompensateHumidity(Raw16 rawHumidity);

    int32_t m_tFine = 0;
    CompensationParameters1 m_params1{};
    CompensationParameters2 m_params2{};
};

static BME280::Measurements operator+(const BME280::Measurements& a, const BME280::Measurements& b)
{
    BME280::Measurements res{};
    res.humidity = a.humidity + b.humidity;
    res.pressure = a.pressure + b.pressure;
    res.temperature = a.temperature + b.temperature;

    return res;
}

static BME280::Measurements operator*(const BME280::Measurements& a, float mult)
{
    BME280::Measurements res = a;
    res.humidity = static_cast<uint32_t>(std::lround(a.humidity * mult));
    res.pressure = static_cast<uint32_t>(std::lround(a.pressure * mult));
    res.temperature = static_cast<uint32_t>(std::lround(a.temperature * mult));

    return res;
}
