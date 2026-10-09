#include "BME280.h"
#include "stm32f4xx_hal.h"

namespace
{
    constexpr uint16_t CALIBRATION_SAMPLES = 32;
}

BME280::BME280(I2C_HandleTypeDef& handle, SemaphoreHandle_t mutex, uint16_t address)
    : I2cSensor(handle, mutex, address)
{}

Status BME280::Start()
{
    {
        uint8_t chipId = 0;
        auto res = ReadByte(0xD0, chipId);
        if (res != Status::Ok)
            return res;
        if (chipId != 0x60)
            return Status::Error;
    }

    {
        auto res = WriteByte(0xE0, 0xB6); // soft reset
        if (res != Status::Ok)
            return res;
        HAL_Delay(10);
    }

    {
        auto res = WriteByte(0xF2, OversamplingHumidity); // ctrl_hum: osrs_h = 1
        if (res != Status::Ok)
            return res;
    }

    {
        constexpr uint8_t osrs_t = OversamplingTemperature;
        constexpr uint8_t osrs_p = OversamplingPressure;
        constexpr uint8_t mode   = Mode; // normal mode

        auto res = WriteByte(0xF4, static_cast<uint8_t>((osrs_t << 5) | (osrs_p << 2) | mode));
        if (res != Status::Ok)
            return res;
    }

    {
        constexpr uint8_t t_sb   = StandbyRegister;
        constexpr uint8_t filter = 0;
        constexpr uint8_t spi3w  = 0; // unused on I2C, register bit is don't-care

        auto res = WriteByte(0xF5, static_cast<uint8_t>((t_sb << 5) | (filter << 2) | spi3w));
        if (res != Status::Ok)
            return res;
    }

    HAL_Delay(10);
    return ReadCompensationData();
}

Status BME280::ReadCompensationData()
{
    auto res = ReadBytes(0x88, std::span<uint8_t>{reinterpret_cast<uint8_t*>(&m_params1), sizeof(CompensationParameters1)});
    if (res != Status::Ok)
        return res;

    std::array<uint8_t, 7> raw{};
    res = ReadBytes(0xE1, raw);
    if (res != Status::Ok)
        return res;

    m_params2.dig_H2 = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    m_params2.dig_H3 = raw[2];
    m_params2.dig_H4 = static_cast<int16_t>((static_cast<int8_t>(raw[3]) << 4) | (raw[4] & 0x0F));
    m_params2.dig_H5 = static_cast<int16_t>((static_cast<int8_t>(raw[5]) << 4) | (raw[4] >> 4));
    m_params2.dig_H6 = static_cast<int8_t>(raw[6]);
    return Status::Ok;
}

Status BME280::ReadBmeData(BmeData& data)
{
    return ReadBytes(0xF7, std::span<uint8_t>{reinterpret_cast<uint8_t*>(&data), sizeof(BmeData)});
}

Status BME280::GetRawValues(BME280::Measurements& measurements)
{
    BmeData data{};
    auto res = ReadBmeData(data);
    if (res != Status::Ok)
        return res;

    measurements.temperature = CompensateTemperature(data.temperature);
    measurements.pressure = CompensatePressure(data.pressure);
    measurements.humidity = CompensateHumidity(data.humidity);
    return res;
}

uint32_t BME280::CompensateTemperature(Raw20 rawTemp)
{
    int32_t var1, var2, T;
    auto adcTemp = rawTemp.value();
    var1 = ((((adcTemp>>3) - static_cast<int32_t>(m_params1.dig_T1<<1))) * (static_cast<int32_t>(m_params1.dig_T2))) >> 11;
    var2 = (((((adcTemp>>4) - static_cast<int32_t>(m_params1.dig_T1))
        * ((adcTemp>>4) - static_cast<int32_t>(m_params1.dig_T1)))>> 12)
        * static_cast<int32_t>(m_params1.dig_T3)) >> 14;
    m_tFine = var1 + var2;
    T = (m_tFine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280::CompensatePressure(Raw20 rawPressure)
{
    int64_t var1, var2, p;
    var1 = (static_cast<int64_t>(m_tFine)) - 128000;
    var2 = var1 * var1 * static_cast<int64_t>(m_params1.dig_P6);
    var2 = var2 + ((var1*static_cast<int64_t>(m_params1.dig_P5))<<17);
    var2 = var2 + ((static_cast<int64_t>(m_params1.dig_P4))<<35);
    var1 = ((var1 * var1 * static_cast<int64_t>(m_params1.dig_P3))>>8) + ((var1 * static_cast<int64_t>(m_params1.dig_P2))<<12);
    var1 = ((((1ll)<<47)+var1))*(static_cast<int64_t>(m_params1.dig_P1))>>33;
    if (var1 == 0)
        return 0;
    p = 1048576-rawPressure.value();
    p = (((p<<31)-var2)*3125)/var1;
    var1 = ((static_cast<int64_t>(m_params1.dig_P9)) * (p>>13) * (p>>13)) >> 25;
    var2 = ((static_cast<int64_t>(m_params1.dig_P8)) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((static_cast<int64_t>(m_params1.dig_P7))<<4);
    return static_cast<uint32_t>(p);
}

uint32_t BME280::CompensateHumidity(Raw16 rawHumidity)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (m_tFine - 76800l);
    v_x1_u32r = (((((rawHumidity.value() << 14) - ((static_cast<int32_t>(m_params2.dig_H4)) << 20) - ((static_cast<int32_t>(m_params2.dig_H5)) *
    v_x1_u32r)) + 16384l) >> 15) * (((((((v_x1_u32r *
    (static_cast<int32_t>(m_params2.dig_H6))) >> 10) * (((v_x1_u32r * (static_cast<int32_t>(m_params2.dig_H3))) >> 11) +
    (32768l))) >> 10) + (2097152l)) * (static_cast<int32_t>(m_params2.dig_H2)) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    (static_cast<int32_t>(m_params1.dig_H1))) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return static_cast<uint32_t>(v_x1_u32r>>12);
}

void BME280::Normalize(Measurements&)
{
    // noop
}

void BME280::Calibrate(Measurements& baseMeasurements)
{
    uint16_t samplesTaken = 0;
    const uint16_t rateHz = GetSamplingRate();
    const uint32_t sampleDelayMs = (rateHz > 0) ? (1000u / rateHz) + 1u : 10u;
    Measurements res{};

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        Measurements m{};
        if (GetRawValues(m) == Status::Ok)
        {
            res = res + m;
            ++samplesTaken;
        }
        vTaskDelay(pdMS_TO_TICKS(sampleDelayMs));
    }
    if (samplesTaken > 0)
        res = res * (1.0f / samplesTaken);

    baseMeasurements = res;
}
