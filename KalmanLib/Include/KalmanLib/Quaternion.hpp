#pragma once
#include "Matrix.hpp"
#include <cmath>

struct Attitude
{
    float rollRad;
    float pitchRad;
};

template<typename T>
class Quaternion
{
public:
    static Quaternion FromAxisAngle(const Vector<T, 3>& axis, T angle)
    {
        const T half = angle / static_cast<T>(2);
        const T axisNorm = Norm(axis);
        if (axisNorm == static_cast<T>(0) || angle == static_cast<T>(0))
            return Identity();

        return { std::cos(half), std::sin(half) * (axis / axisNorm) };
    }

    static constexpr Quaternion FromRotationMatrix(const Matrix3f& matrix)
    {
        const T m00 = static_cast<T>(matrix(0, 0));
        const T m11 = static_cast<T>(matrix(1, 1));
        const T m22 = static_cast<T>(matrix(2, 2));
        const T trace = m00 + m11 + m22;
        T scalar;
        Vector<T, 3> vector{};

        if (trace > static_cast<T>(0))
        {
            const T scale = static_cast<T>(2) * SquareRoot(trace + static_cast<T>(1));
            scalar = scale / static_cast<T>(4);
            vector(0, 0) = (static_cast<T>(matrix(2, 1)) - static_cast<T>(matrix(1, 2))) / scale;
            vector(1, 0) = (static_cast<T>(matrix(0, 2)) - static_cast<T>(matrix(2, 0))) / scale;
            vector(2, 0) = (static_cast<T>(matrix(1, 0)) - static_cast<T>(matrix(0, 1))) / scale;
        }
        else if (m00 > m11 && m00 > m22)
        {
            const T scale = static_cast<T>(2) * SquareRoot(
                static_cast<T>(1) + m00 - m11 - m22);
            scalar = (static_cast<T>(matrix(2, 1)) - static_cast<T>(matrix(1, 2))) / scale;
            vector(0, 0) = scale / static_cast<T>(4);
            vector(1, 0) = (static_cast<T>(matrix(0, 1)) + static_cast<T>(matrix(1, 0))) / scale;
            vector(2, 0) = (static_cast<T>(matrix(0, 2)) + static_cast<T>(matrix(2, 0))) / scale;
        }
        else if (m11 > m22)
        {
            const T scale = static_cast<T>(2) * SquareRoot(
                static_cast<T>(1) + m11 - m00 - m22);
            scalar = (static_cast<T>(matrix(0, 2)) - static_cast<T>(matrix(2, 0))) / scale;
            vector(0, 0) = (static_cast<T>(matrix(0, 1)) + static_cast<T>(matrix(1, 0))) / scale;
            vector(1, 0) = scale / static_cast<T>(4);
            vector(2, 0) = (static_cast<T>(matrix(1, 2)) + static_cast<T>(matrix(2, 1))) / scale;
        }
        else
        {
            const T scale = static_cast<T>(2) * SquareRoot(
                static_cast<T>(1) + m22 - m00 - m11);
            scalar = (static_cast<T>(matrix(1, 0)) - static_cast<T>(matrix(0, 1))) / scale;
            vector(0, 0) = (static_cast<T>(matrix(0, 2)) + static_cast<T>(matrix(2, 0))) / scale;
            vector(1, 0) = (static_cast<T>(matrix(1, 2)) + static_cast<T>(matrix(2, 1))) / scale;
            vector(2, 0) = scale / static_cast<T>(4);
        }

        return {scalar, vector};
    }

    static constexpr Quaternion FromTwoVectors(const Vector<T, 3>& a, const Vector<T, 3>& b)
    {
        Quaternion q(T(1) + DotProduct(a, b), CrossProduct(a, b));
        q.Normalize();
        return q;
    }

    static constexpr Quaternion Identity()
    {
        return { static_cast<T>(1), {} };
    }

    constexpr Quaternion() : Quaternion(Identity())
    {
    }

    constexpr Quaternion operator*(const Quaternion& rhs) const
    {
        const T scalar = m_scalar * rhs.m_scalar - DotProduct(m_vector, rhs.m_vector);
        const Vector<T, 3> vector = m_scalar * rhs.m_vector
                                + rhs.m_scalar * m_vector
                                + CrossProduct(m_vector, rhs.m_vector);
        return Quaternion(scalar, vector);
    }

    constexpr Vector<T, 3> Rotate(const Vector<T, 3>& rotate) const
    {
        Quaternion fromRotation(static_cast<T>(0), rotate);
        Quaternion conjugate = Conjugate();
        return ((*this)*fromRotation*conjugate).m_vector;
    }

    constexpr Vector<T, 3> InverseRotate(const Vector<T, 3>& rotate) const
    {
        Quaternion fromRotation(static_cast<T>(0), rotate);
        return (Conjugate()*fromRotation*(*this)).m_vector;
    }

    constexpr Matrix<T, 3, 3> ToRotationMatrix() const
    {
        const T w = m_scalar;
        const T x = m_vector(0, 0);
        const T y = m_vector(1, 0);
        const T z = m_vector(2, 0);

        Matrix<T, 3, 3> R;

        R(0,0) = static_cast<T>(1) - static_cast<T>(2) * (y*y + z*z);
        R(0,1) = static_cast<T>(2) * (x*y - w*z);
        R(0,2) = static_cast<T>(2) * (x*z + w*y);

        R(1,0) = static_cast<T>(2) * (x*y + w*z);
        R(1,1) = static_cast<T>(1) - static_cast<T>(2) * (x*x + z*z);
        R(1,2) = static_cast<T>(2) * (y*z - w*x);

        R(2,0) = static_cast<T>(2) * (x*z - w*y);
        R(2,1) = static_cast<T>(2) * (y*z + w*x);
        R(2,2) = static_cast<T>(1) - static_cast<T>(2) * (x*x + y*y);

        return R;
    }

    constexpr void Normalize()
    {
        const T norm = SquareRoot(m_scalar * m_scalar + DotProduct(m_vector, m_vector));
        if (norm == static_cast<T>(0))
            return;

        m_scalar /= norm;
        m_vector = m_vector / norm;
    }

    constexpr Vector<T, 4> GetRawValues() const
    {
        Vector<T, 4> raw{};
        raw(0,0) = m_scalar;
        raw(1,0) = m_vector(0,0);
        raw(2,0) = m_vector(1,0);
        raw(3,0) = m_vector(2,0);
        return raw;
    }

    Attitude ToAttitude() const
    {
        const T w = m_scalar, x = m_vector(0,0), y = m_vector(1,0), z = m_vector(2,0);
        const T roll = std::atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));

        const T sinPitch = 2.0f * (w*y - z*x);
        const T pitch = std::abs(sinPitch) >= 1.0f
            ? std::copysign(static_cast<T>(M_PI) / 2.0f, sinPitch)
            : std::asin(sinPitch);

        return {roll, pitch};
    }

private:
    constexpr Quaternion(T scalar, const Vector<T, 3>& vec) : m_vector(vec), m_scalar(scalar) {}

    constexpr Quaternion Conjugate() const
    {
        return { m_scalar, -m_vector };
    }

    Vector<T, 3> m_vector;
    T m_scalar;
};
