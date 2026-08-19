#pragma once
#include "Matrix.hpp"
#include "Quaternion.hpp"

class MultiplicativeExtendedKalmanFilter
{
public:
    MultiplicativeExtendedKalmanFilter(
        const Vector3f& gravityDirection,
        const Vector3f& magReference,
        const Matrix3f& accelR,
        const Matrix3f& magnetR,
        const Matrix3f& N)
        : m_gravityDirection(gravityDirection)
        , m_accelR(accelR)
        , m_N(N)
    {
        // Remove vertical component from magReference. Keep only heading
        m_magReference = magReference - m_gravityDirection * DotProduct(magReference, m_gravityDirection);

        m_horizontalAxis = CrossProduct(m_gravityDirection, m_magReference);

        // To project covariance onto such axis we do
        // σ²(a) = (aᵀ M a) / (aᵀ a)
        const auto coeff = (m_horizontalAxis.Transpose() * m_horizontalAxis)(0,0);
        // this variance is in gauss^2
        const auto variance = (m_horizontalAxis.Transpose() * magnetR * m_horizontalAxis)(0,0) / coeff;

        // we need to convert it to rad^2
        m_magR = variance / NormSquared(m_magReference);
    }

    Quaternion<float> Step(const Vector3f& gyro, const Vector3f& accel, const Vector3f& magnet, float dt, bool useMagnetometer = true)
    {
        Predict(gyro, dt);
        UpdateAccelerometer(accel, dt);
        if (useMagnetometer)
            UpdateMagnetometer(magnet, dt);

        return m_rotation;
    }

private:
    void Predict(const Vector3f& input, float dt)
    {
        Matrix3f A{};
        A(0, 0) = 0.0f;
        A(0, 1) = input(2,0);
        A(0, 2) = -input(1,0);

        A(1, 0) = -input(2,0);
        A(1, 1) = 0.0f;
        A(1, 2) = input(0,0);

        A(2, 0) = input(1,0);
        A(2, 1) = -input(0,0);
        A(2, 2) = 0.0f;

        const auto F = Matrix3f::Identity() + A * dt;
        const auto Q = m_N*dt*dt;
        m_P = F * m_P * F.Transpose() + Q;
        m_rotation = m_rotation * Quaternion<float>::FromAxisAngle(input, Norm(input) * dt);
    }

    void UpdateAccelerometer(const Vector3f& measurements, float dt)
    {
        const auto R = m_rotation.ToRotationMatrix().Transpose(); // world to body transformation
        // Gravity direction is in world coordinates and if we rotate it into body coordinates
        // we can now say that this is what are we expecting from sensor.
        // Say gravity direction is 0,0,1; and accelerometer's body coordinates (current readings)
        // give us 1,0,0. It means that X axis lays at axis where gravity is
        // Then this multiplication gives us exactly that.
        // We get current rotation estimate, and rotate our world reference into body coords and get ~1,0,0
        // ~ because of error which we are minimizing with filtering.
        const auto h = R * m_gravityDirection;
        Matrix3f H{};
        H(0, 0) = 0.0f;
        H(0, 1) = -h(2,0);
        H(0, 2) =  h(1,0);

        H(1, 0) = h(2,0);
        H(1, 1) = 0.0f;
        H(1, 2) = -h(0,0);

        H(2, 0) = -h(1,0);
        H(2, 1) = h(0,0);
        H(2, 2) = 0.0f;

        const auto S = H*m_P*H.Transpose() + m_accelR;
        const auto K = m_P*H.Transpose()*S.Inverse();
        const auto error = measurements - h;
        const auto errorGain = K * error;
        InjectRotation(errorGain);

        const auto IKH = Matrix3f::Identity() - K*H;
        m_P = IKH * m_P * IKH.Transpose() + K * m_accelR * K.Transpose();
    }

    void UpdateMagnetometer(const Vector3f& measurements, float dt)
    {
        // body to world rotation
        // for magnetometer we want to work in world coordinates so wa want to transform everything onto world plane
        const auto R = m_rotation.ToRotationMatrix();
        const auto worldMag = R * measurements;

        //Lets project measurements vector onto a plane perpendicular to gravity direction
        //Perpendicular plane to gravity is exactly horizontal plane.
        //So now we have vector that points to estimated heading
        const auto measH = worldMag - m_gravityDirection * DotProduct(worldMag, m_gravityDirection);

        // this code does not work because non horizontal components leak through rotation and introduce wrong errors
        // const auto error = measH / Norm(measH) - refH / Norm(refH);
        // correct way of getting an error is calculating an angle between 2 horizontal vectors

        // This is standard way of calculating an angle between 2 vectors along axis n
        // here vectors are refH and measH and vector is gravity vector.
        // So we projected vectors onto horizontal plane and took an angle by rotating them along vertical (gravity) axis
        const float dPsi = std::atan2(
            DotProduct(m_gravityDirection, CrossProduct(m_magReference, measH)),
            DotProduct(m_magReference, measH)
        );
        printf("psi: %ld\n", lroundf(dPsi * 1000));
        // dPsi is an error, innovation

        // next step is to calculate innovation covariance S
        // its usually calculated as H*P*H^T + R
        // need to calculate H = ∂(dPsi)/∂(δθ)
        // dPsi is pure horizontal rotation that is rotated along m_gravityDirection
        // dPsi = atan2(g*(magH(x)measH), magH * measH); say magH = r, measH = m
        // dPsi = atan2(s, c),   s = g·(r × m),   c = r · m
        // d(dPsi) = (c·ds − s·dc) / (s² + c²) source: https://math.stackexchange.com/questions/3570633/derivative-of-operatornamearctan2
        // r is constant and m is something that depends on δθ
        // lets assume δθ = 0
        // which means m0 = r => s0 = g(r × r) = 0; c = r * r = |r|^2
        // since s0
        // d(dPsi) = (c·ds) / (c²) = |r|^2 * ds / |r|^2^2 = ds / |r|^2;
        // ds = d (g·(r × m)) = (g × r)·m
        // let w = g × r - constant vector; so ds = d(w*m)
        // ds = w*dm = w*d(measH)
        // measH = worldMag - m_gravityDirection * DotProduct(worldMag, m_gravityDirection)
        // it can be rewritten as
        // measH = (I - g*g^T)*worldMag
        // define P = (I - g*g^T)
        // measH = P*worldMag = P*(R*z); where z is body frame measurements
        // measH = P*(R*z)
        // ds = w*dm = w*d(measH)
        // ds = w*d(P*(R*z));
        // lets derivate dm = d(P*(R*z)) = P*d(Rz)
        // Assume that R is estimated noisy rotation where Rtrue = R(I + [dTheta(x)]) = R + R[dTheta(x)]
        // d(P*(R*z)) = P * d(R*z) = P * d(R + R[dTheta(x)])*z = P*d(Rz) + P*(R[dTheta(x)]*z)
        // P*d(Rz) = 0 since Rz is constant
        // m = P*(R[dTheta(x)]*z)
        // recall [a(x)]b = a(x)b = -[b(x)]a
        // m = P*(R[-z(x)]*dTheta)
        // ds = w*dm = w*d(P*(R[-z(x)]*dTheta)) = w*P*R[-z(x)]d(dTheta)
        // d(dPsi) = ds / |r|^2 = w*P*R[-z(x)]d(dTheta) / |r|^2
        // H = ∂(dPsi)/∂(δθ) = ds / |r|^2 = w*P*R[-z(x)] / |r|^2
        // H = (g × r)*P*R[-z(x)] / |r|^2
        // Check what is P; P = (I - g*g^T); Its property is that P*g = 0
        // H = (g × r)*(I - g*g^T)*R[-z(x)] / |r|^2 = ((g × r) - (g × r) * g*g^T)*R[-z(x)] / |r|^2
        // (g × r) * g is 0 because first term is perpendicular so
        // H = (g × r)*R[-z(x)] / |r|^2
        // There is a property: R [z×] Rᵀ = [(Rz)×]
        // Which gives us R [z×] = [(Rz)×] R
        // H = -(g × r)*[(Rz)×] R / |r|^2
        // Lets linearize Rz = r - we are close to reference
        // H = -(g × r)*[r×] R / |r|^2
        // use identity: (a(x)b)*[b(x)] = (|b|^2)*a^T
        // in our case = -(g × r)*([r(x)]) = -|r|^2 * g^T
        // Which gives us H = -|r|^2 * g^T * R / |r|^2 = -g^T * R

        const auto H  = -m_gravityDirection.Transpose() * R;

        const auto S = H*m_P*H.Transpose() + m_magR;
        const auto K = m_P*H.Transpose()*S.Inverse();
        const auto errorGain = K * dPsi;
        InjectRotation(errorGain);

        const auto IKH = Matrix3f::Identity() - K*H;
        m_P = IKH * m_P * IKH.Transpose() + K * m_magR * K.Transpose();
    }

    void InjectRotation(const Vector3f& thetaAngle)
    {
        const float eps = 1e-4f;
        const auto angle = Norm(thetaAngle);
        auto dq = Quaternion<float>::Identity();
        if(angle > eps)
        {
            dq = Quaternion<float>::FromAxisAngle(
                thetaAngle / angle,
                angle);
        }

        m_rotation = m_rotation * dq;
        m_rotation.Normalize();
    }

    Vector3f m_gravityDirection;
    Vector3f m_horizontalAxis;
    Vector3f m_magReference;
    Vector3f m_horizontalDirection;
    Matrix3f m_accelR;
    float m_magR;
    Matrix3f m_N;
    Matrix3f m_P = Matrix3f::Identity();
    Quaternion<float> m_rotation = Quaternion<float>::Identity();
};
