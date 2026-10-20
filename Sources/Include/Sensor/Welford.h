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

template <typename T, size_t N>
struct Welford<Matrix<T, N, 1>>
{
    using Vec = Matrix<T, N, 1>;
    using Mat = Matrix<T, N, N>;

    uint32_t n = 0;
    Vec mean{};
    Mat M2{};

    void Add(const Vec& x)
    {
        ++n;
        const Vec delta = x - mean;
        mean += delta / static_cast<T>(n);
        M2 += delta * (x - mean).Transpose();
    }

    void Reset()
    {
        n = 0;
        mean = Vec{};
        M2 = Mat{};
    }

    uint32_t Count() const { return n; }

    Vec Mean() const { return mean; }

    Mat Covariance() const
    {
        return n > 1 ? M2 / static_cast<T>(n - 1) : Mat{};
    }

    Vec Variance() const
    {
        Vec v{};
        if (n > 1)
        {
            const T k = static_cast<T>(1) / static_cast<T>(n - 1);
            for (size_t i = 0; i < N; ++i)
                v(i, 0) = M2(i, i) * k;
        }
        return v;
    }
};
