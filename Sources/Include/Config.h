#pragma once

namespace Config
{
    constexpr bool ENABLE_MAGNETOMETER_CORRECTION = false;

    constexpr uint32_t BATTERY_MAX_USEFUL_EXTRA_BITS = 4;
    constexpr uint32_t BATTERY_ADC_OVERSAMPLING_RATIO = 1u << (2 * BATTERY_MAX_USEFUL_EXTRA_BITS);
    constexpr float BATTERY_MAX_VOLTAGE = 25.2f;
    constexpr float BATTERY_ADC_MAX_VOLTAGE = 3.0f;
    constexpr uint32_t BATTERY_SAMPLING_RATE_HZ = 20;

    constexpr uint8_t OSD_REFRESH_RATE_HZ = 20;

    constexpr uint32_t SOFTWARE_UART_READ_TIMEOUT_MS = 200;

    constexpr uint32_t TRAMP_PROTOCOL_POLLING_RATE_MS = 200;
}
