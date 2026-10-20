#pragma once
#include <array>
#include <cassert>
#include <cstddef>
#include <cmath>
#include "SquareRoot.hpp"

template<typename T, size_t Rows, size_t Cols>
class Matrix;

template<typename T, size_t N>
constexpr Matrix<T, N, N> Inverse(const Matrix<T, N, N>&);

template<typename T>
constexpr Matrix<T, 1, 1> Inverse(const Matrix<T, 1, 1>&);

template<typename T>
constexpr Matrix<T, 2, 2> Inverse(const Matrix<T, 2, 2>&);

template<typename T>
constexpr Matrix<T, 3, 3> Inverse(const Matrix<T, 3, 3>&);

template<typename T, size_t Rows, size_t Cols>
class Matrix
{
public:
    constexpr Matrix() = default;

    static constexpr Matrix Identity()
    {
        static_assert(Rows == Cols);

        Matrix result{};

        for(size_t i = 0; i < Rows; ++i)
            result(i, i) = static_cast<T>(1);

        return result;
    }

    constexpr T& operator()(size_t row, size_t col)
    {
        return m_buffer[row][col];
    }

    constexpr const T& operator()(size_t row, size_t col) const
    {
        return m_buffer[row][col];
    }

    constexpr Matrix operator+(const Matrix& other) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) + other(r, c);

        return result;
    }

    constexpr Matrix operator+(T scalar) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) + scalar;

        return result;
    }

    constexpr Matrix operator-(T scalar) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) - scalar;

        return result;
    }

    constexpr Matrix& operator+=(const Matrix& other)
    {
        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                (*this)(r, c) += other(r, c);

        return *this;
    }

    constexpr Matrix operator-(const Matrix& other) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) - other(r, c);

        return result;
    }

    constexpr Matrix& operator-=(const Matrix& other)
    {
        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                (*this)(r, c) -= other(r, c);

        return *this;
    }

    constexpr Matrix operator-() const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = -(*this)(r, c);

        return result;
    }

    constexpr Matrix operator*(T scalar) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) * scalar;

        return result;
    }

    constexpr Matrix operator/(T scalar) const
    {
        Matrix result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(r, c) = (*this)(r, c) / scalar;

        return result;
    }

    constexpr Matrix& operator*=(T scalar)
    {
        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                (*this)(r, c) *= scalar;

        return *this;
    }

    friend constexpr Matrix operator*(T scalar, const Matrix& matrix)
    {
        return matrix * scalar;
    }

    constexpr Matrix<T, Cols, Rows> Transpose() const
    {
        Matrix<T, Cols, Rows> result;

        for(size_t r = 0; r < Rows; ++r)
            for(size_t c = 0; c < Cols; ++c)
                result(c, r) = (*this)(r, c);

        return result;
    }

    constexpr auto Inverse() const
    {
        return ::Inverse(*this);
    }

private:
    std::array<std::array<T, Cols>, Rows> m_buffer{};
};

template<typename T,
         size_t Rows,
         size_t Inner,
         size_t Cols>
constexpr Matrix<T, Rows, Cols>
operator*(const Matrix<T, Rows, Inner>& a,
          const Matrix<T, Inner, Cols>& b)
{
    Matrix<T, Rows, Cols> result;

    for(size_t r = 0; r < Rows; ++r)
    {
        for(size_t c = 0; c < Cols; ++c)
        {
            T sum{};

            for(size_t k = 0; k < Inner; ++k)
                sum += a(r, k) * b(k, c);

            result(r, c) = sum;
        }
    }

    return result;
}

template<typename T, size_t N>
constexpr Matrix<T, N, N> Inverse(const Matrix<T, N, N>& matrix)
{
    Matrix<T, N, N> A = matrix;
    Matrix<T, N, N> I = Matrix<T, N, N>::Identity();

    for(size_t col = 0; col < N; ++col)
    {
        const T pivot = A(col, col);

        assert(pivot != static_cast<T>(0));

        const T invPivot = static_cast<T>(1) / pivot;

        for(size_t j = 0; j < N; ++j)
        {
            A(col, j) *= invPivot;
            I(col, j) *= invPivot;
        }

        for(size_t row = 0; row < N; ++row)
        {
            if(row == col)
                continue;

            const T factor = A(row, col);

            for(size_t j = 0; j < N; ++j)
            {
                A(row, j) -= factor * A(col, j);
                I(row, j) -= factor * I(col, j);
            }
        }
    }

    return I;
}

template<typename T>
constexpr Matrix<T, 1, 1> Inverse(const Matrix<T, 1, 1>& m)
{
    Matrix<T, 1, 1> result;

    assert(m(0,0) != static_cast<T>(0));

    result(0,0) = static_cast<T>(1) / m(0,0);

    return result;
}

template<typename T>
constexpr Matrix<T, 2, 2> Inverse(const Matrix<T, 2, 2>& m)
{
    const T a = m(0,0);
    const T b = m(0,1);
    const T c = m(1,0);
    const T d = m(1,1);

    const T det = a * d - b * c;

    assert(det != static_cast<T>(0));

    const T invDet = static_cast<T>(1) / det;

    Matrix<T, 2, 2> result;

    result(0,0) =  d * invDet;
    result(0,1) = -b * invDet;
    result(1,0) = -c * invDet;
    result(1,1) =  a * invDet;

    return result;
}

template<typename T>
constexpr Matrix<T, 3, 3> Inverse(const Matrix<T, 3, 3>& m)
{
    const T a = m(0,0);
    const T b = m(0,1);
    const T c = m(0,2);

    const T d = m(1,0);
    const T e = m(1,1);
    const T f = m(1,2);

    const T g = m(2,0);
    const T h = m(2,1);
    const T i = m(2,2);

    const T det =
          a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g);

    assert(det != static_cast<T>(0));

    const T invDet = static_cast<T>(1) / det;

    Matrix<T, 3, 3> result;

    result(0,0) =  (e*i - f*h) * invDet;
    result(0,1) = -(b*i - c*h) * invDet;
    result(0,2) =  (b*f - c*e) * invDet;

    result(1,0) = -(d*i - f*g) * invDet;
    result(1,1) =  (a*i - c*g) * invDet;
    result(1,2) = -(a*f - c*d) * invDet;

    result(2,0) =  (d*h - e*g) * invDet;
    result(2,1) = -(a*h - b*g) * invDet;
    result(2,2) =  (a*e - b*d) * invDet;

    return result;
}

template<typename T, size_t N>
using Vector = Matrix<T, N, 1>;

using Vector2f = Vector<float, 2>;
using Vector3f = Vector<float, 3>;
using Vector4f = Vector<float, 4>;

using Matrix2f = Matrix<float, 2, 2>;
using Matrix3f = Matrix<float, 3, 3>;
using Matrix4f = Matrix<float, 4, 4>;

template<typename T, size_t N>
constexpr T DotProduct(const Vector<T, N>& a, const Vector<T, N>& b)
{
    T result{};
    for (size_t i = 0; i < N; i++)
        result += a(i, 0) * b(i, 0);
    return result;
}

template<typename T, size_t N>
constexpr T NormSquared(const Vector<T, N>& a)
{
    return DotProduct(a, a);
}

template<typename T, size_t N>
constexpr T Norm(const Vector<T, N>& a)
{
    return SquareRoot(NormSquared(a));
}

template<typename T, size_t N>
constexpr Vector<T, N> Normalize(const Vector<T, N>& a)
{
    return a * (static_cast<T>(1) / Norm(a));
}

template<typename T>
constexpr Vector<T, 3> CrossProduct(const Vector<T, 3>& a,
                                    const Vector<T, 3>& b)
{
    Vector<T, 3> result{};

    result(0, 0) =
        a(1, 0) * b(2, 0) -
        a(2, 0) * b(1, 0);

    result(1, 0) =
        a(2, 0) * b(0, 0) -
        a(0, 0) * b(2, 0);

    result(2, 0) =
        a(0, 0) * b(1, 0) -
        a(1, 0) * b(0, 0);

    return result;
}
