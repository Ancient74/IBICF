#pragma once
#include <cmath>
#include <cstdio>
#include "Config.h"
#include "Qmc5883L.h"
#include "Mpu6050.h"
#include <optional>
#include <KalmanLib/MultiplicativeExtendedKalmanFilter.hpp>

namespace
{
    constexpr UBaseType_t queueLength = 16;

    // Tagged sample posted by the sampling tasks and consumed by the fusion task.
    struct FusionSample
    {
        enum class Source : uint8_t
        {
            Imu,
            Mag
        };

        Source source;
        union
        {
            GyroscopeAccelerometer imu;
            Magnetometer mag;
        };
    };
}

class FusionCore
{
public:
    FusionCore(const Matrix3f& accelR, const Matrix3f& magnetR, const Matrix3f& N)
    {
        Vector3f gravityDirection{};
        gravityDirection(2, 0) = 1.0f;
        Vector3f magnetometerDirection{};
        magnetometerDirection(0,0) = 0.434f;
        magnetometerDirection(1,0) = 0.061f;
        magnetometerDirection(2,0) = 0.899f;
        m_kalmanFilter = std::make_unique<MultiplicativeExtendedKalmanFilter>(gravityDirection, magnetometerDirection, accelR, magnetR, N);
    }

    Quaternion<float> Step(const GyroscopeAccelerometer& imu, bool hasMag, const Magnetometer& mag, float dt)
    {
        Vector3f accel{};
        accel(0, 0) = imu.acc.x;
        accel(1, 0) = imu.acc.y;
        accel(2, 0) = imu.acc.z;
        accel = Normalize(accel);

        Vector3f gyro{};
        gyro(0, 0) = imu.gyro.x;
        gyro(1, 0) = imu.gyro.y;
        gyro(2, 0) = imu.gyro.z;

        Vector3f magnet{};
        if (hasMag)
        {
            magnet(0, 0) = mag.x;
            magnet(1, 0) = mag.y;
            magnet(2, 0) = mag.z;
        }

        return m_kalmanFilter->Step(gyro, accel, magnet, dt, hasMag);
    }

private:
    std::unique_ptr<MultiplicativeExtendedKalmanFilter> m_kalmanFilter;
};

template<uint8_t MaxSubs>
class SensorFusionNode
    : public NodeOutput<SensorFusionNode<MaxSubs>, Quaternion<float>, MaxSubs>
    , public CommonTask<SensorFusionNode<MaxSubs>, 128, configMAX_PRIORITIES / 2 - 4>
{
public:
    struct ImuPort : NodeInput<ImuPort, GyroscopeAccelerometer>
    {
        using InputType = GyroscopeAccelerometer;
        explicit ImuPort(SensorFusionNode* owner) : self(owner) {}
        void OnMessage(const GyroscopeAccelerometer& s) { self->PostImu(s); }
        SensorFusionNode* self;
    };
    struct MagPort : NodeInput<MagPort, Magnetometer>
    {
        using InputType = Magnetometer;
        explicit MagPort(SensorFusionNode* owner) : self(owner) {}
        void OnMessage(const Magnetometer& s) { self->PostMag(s); }
        SensorFusionNode* self;
    };

    using OutputType = Quaternion<float>;

    SensorFusionNode(const char* taskName, float nominalDt, const Matrix3f& accelR, const Matrix3f& magnetR, const Matrix3f& N)
        : CommonTask<SensorFusionNode, 128, configMAX_PRIORITIES / 2 - 4>(taskName, 0)
        , m_core(accelR, magnetR, N), m_nominalDt(nominalDt)
    {
        m_sampleQueue = xQueueCreate(queueLength, sizeof(FusionSample));
        configASSERT(m_sampleQueue != nullptr);
    }

    ~SensorFusionNode()
    {
        this->Stop();
        vQueueDelete(m_sampleQueue);
    }

    ImuPort& ImuInput()
    {
        return m_imuPort;
    }

    MagPort& MagInput()
    {
        return m_magPort;
    }

    void RunTaskIteration()
    {
        FusionSample item{};
        if (xQueueReceive(m_sampleQueue, &item, pdMS_TO_TICKS(200)) != pdTRUE)
            return;

        if (item.source == FusionSample::Source::Mag)
        {
            m_latestMag = item.mag;
            return;
        }

        const TickType_t now = xTaskGetTickCount();
        float dt = m_nominalDt;
        if (m_hasLastTick)
        {
            dt = (now - m_lastTick) * static_cast<float>(portTICK_PERIOD_MS) / 1000.0f;
            if (dt <= 0.0f) dt = m_nominalDt;
        }
        m_lastTick = now;
        m_hasLastTick = true;

        const bool hasMag = m_latestMag.has_value();
        const Magnetometer mag = hasMag ? *m_latestMag : Magnetometer{};
        const auto quat = m_core.Step(item.imu, hasMag, mag, dt);
        m_latestMag = std::nullopt;

        this->Publish(quat);
    }

private:

    void PostImu(const GyroscopeAccelerometer& s)
    {
        FusionSample item{FusionSample::Source::Imu};
        item.imu = s;
        xQueueSend(m_sampleQueue, &item, 0);
    }

    void PostMag(const Magnetometer& s)
    {
        FusionSample item{FusionSample::Source::Mag};
        item.mag = s;
        xQueueSend(m_sampleQueue, &item, 0);
    }

    ImuPort m_imuPort{this};
    MagPort m_magPort{this};

    TickType_t m_lastTick{};
    bool m_hasLastTick{false};
    QueueHandle_t m_sampleQueue{};
    std::optional<Magnetometer> m_latestMag{};

    FusionCore m_core;
    float m_nominalDt;
};
