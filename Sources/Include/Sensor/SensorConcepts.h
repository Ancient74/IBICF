#pragma once
#include <concepts>
#include "I2cSensor.h"

template<typename O>
concept SampleValue = requires(O o, float mult) {
    { o + o } -> std::same_as<O>;
    { o * mult } -> std::same_as<O>;
};

template<typename T, typename O>
concept SensorDevice = SampleValue<O> && requires(T t, O& o) {
    { t.GetSamplingRate() } -> std::same_as<uint16_t>;
    { t.GetRawValues(o) } -> std::same_as<Status>;
    { t.Normalize(o) } -> std::same_as<void>;
};
