#include <concepts>
#include <limits>
#include <type_traits>
#include <cmath>

template <std::floating_point T>
constexpr T ConstexprSquareRoot(T x) noexcept
{
    constexpr T inf = std::numeric_limits<T>::infinity();

    if (!(x >= T(0)))                  // negative or NaN
        return std::numeric_limits<T>::quiet_NaN();
    if (x == T(0) || x == inf)
        return x;

    // Range-reduce into [1, 4) using powers of 4 (exact in binary FP):
    // sqrt(x) = sqrt(x / 4^k) * 2^k
    T scale = T(1);
    while (x >= T(4)) { x /= T(4); scale *= T(2); }
    while (x <  T(1)) { x *= T(4); scale /= T(2); }

    // Start above the root (sqrt(x) < 2). Newton then decreases monotonically,
    // so stop as soon as the value stops decreasing.
    T curr = T(2);
    T prev;
    do {
        prev = curr;
        curr = T(0.5) * (curr + x / curr);
    } while (curr < prev);

    return prev * scale;
}

template <std::floating_point T>
constexpr T SquareRoot(T x) noexcept
{
    if (std::is_constant_evaluated())
        return ConstexprSquareRoot(x);
    return std::sqrt(x);
}
