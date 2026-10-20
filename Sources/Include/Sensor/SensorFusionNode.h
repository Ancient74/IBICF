#pragma once
#include <cmath>
#include <cstdio>
#include "Config.h"
#include "Qmc5883L.h"
#include "Mpu6050.h"
#include "TaskThread.h"
#include <optional>
#include <KalmanLib/MultiplicativeExtendedKalmanFilter.hpp>

namespace
{
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
    FusionCore()
        : m_kalmanFilter(
              Vector3f{[](){ Vector3f v{}; v(2, 0) = 1.0f; return v; }()},
              Vector3f{[](){ Vector3f v{}; v(0,0) = 0.434f; v(1,0) = 0.061f; v(2,0) = 0.899f; return v; }()})
    {
    }

    void SetAccelerometerVariance(const Matrix3f& accelR)
    {
        m_kalmanFilter.SetAccelerometerVariance(accelR);
    }

    void SetMagnetometerVariance(const Matrix3f& magnetR)
    {
        m_kalmanFilter.SetMagnetometerVariance(magnetR);
    }

    void SetGyroscopeVariance(const Matrix3f& N)
    {
        m_kalmanFilter.SetGyroscopeVariance(N);
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

        return m_kalmanFilter.Step(gyro, accel, magnet, dt, hasMag);
    }

private:
    MultiplicativeExtendedKalmanFilter m_kalmanFilter;
};

template<uint8_t MaxSubs>
class SensorFusionNode
    : public NodeOutput<SensorFusionNode<MaxSubs>, Quaternion<float>, MaxSubs>
{
public:
    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }
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

    void SetAccelerometerVariance(const Matrix3f& accelR)
    {
        m_core.SetAccelerometerVariance(accelR);
    }

    void SetMagnetometerVariance(const Matrix3f& magnetR)
    {
        m_core.SetMagnetometerVariance(magnetR);
    }

    void SetGyroscopeVariance(const Matrix3f& N)
    {
        m_core.SetGyroscopeVariance(N);
    }

    SensorFusionNode(const char* taskName)
        : m_thread(taskName, *this)
    {
        m_sampleQueue = xQueueCreateStatic(Config::SENSOR_FUSION_NODE_QUEUE_SIZE, sizeof(FusionSample), m_sampleQueueStorage, &m_sampleQueueBuffer);
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
        if (xQueueReceive(m_sampleQueue, &item, pdMS_TO_TICKS(Config::SENSOR_FUSION_NODE_TIMEOUT_MS)) != pdTRUE)
            return;

        if (item.source == FusionSample::Source::Mag)
        {
            m_latestMag = item.mag;
            return;
        }

        const TickType_t now = xTaskGetTickCount();
        if (!m_hasLastTick)
        {
            m_hasLastTick = true;
            m_lastTick = now;
            return;
        }
        float dt = (now - m_lastTick) * static_cast<float>(portTICK_PERIOD_MS) / 1000.0f;

        m_lastTick = now;
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

    TaskThread<SensorFusionNode<MaxSubs>, Config::Task::SENSOR_FUSION_NODE_STACK_SIZE, Config::Task::SENSOR_FUSION_NODE_PRIORITY> m_thread;

    ImuPort m_imuPort{this};
    MagPort m_magPort{this};

    TickType_t m_lastTick{};
    bool m_hasLastTick{false};
    QueueHandle_t m_sampleQueue{};
    StaticQueue_t m_sampleQueueBuffer{};
    uint8_t m_sampleQueueStorage[Config::SENSOR_FUSION_NODE_QUEUE_SIZE * sizeof(FusionSample)]{};

    std::optional<Magnetometer> m_latestMag{};

    FusionCore m_core;
};
