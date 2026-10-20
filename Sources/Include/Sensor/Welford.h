#pragma once
#include <cstdint>

template <typename T>
struct Welford
{
    uint32_t n = 0;
    T mean = T{};
    T M2 = T{};

    void Add(T x)
    {
        ++n;
        const T delta = x - mean;
        mean += delta / static_cast<T>(n);
        M2 += delta * (x - mean);
    }

    void Reset()
    {
        n = 0;
        mean = T{};
        M2 = T{};
    }

    T Mean() const { return mean; }
    T Variance() const { return n > 1 ? M2 / static_cast<T>(n - 1) : T{}; }
};
