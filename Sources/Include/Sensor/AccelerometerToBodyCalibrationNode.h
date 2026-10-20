#pragma once
#include "Config.h"
#include "PubSubLib/Node.hpp"
#include "KalmanLib/Quaternion.hpp"
#include "TaskThread.h"
#include "Welford.h"

class AccelerometerToBodyCalibrationNode
    : public NodeInput<AccelerometerToBodyCalibrationNode, bool>
    , public NodeInput<AccelerometerToBodyCalibrationNode, Quaternion<float>>
{
    enum class CalibrationState
    {
        Idle,
        CalculateUp,
        WaitingForRight,
        CalculateRight
    };

public:
    AccelerometerToBodyCalibrationNode(const char* taskName, const Vector3f& worldUpVector, const Vector3f& worldRightVector);
    ~AccelerometerToBodyCalibrationNode();

    void Process(bool switchState);
    void Process(const Quaternion<float>& rotation);

    void RunTaskIteration();

private:
    bool m_lastSwitchState = false;
    std::atomic<CalibrationState> m_calibrationState = CalibrationState::Idle;
    TaskThread<AccelerometerToBodyCalibrationNode, Config::Task::ACCELEROMETER_TO_BODY_CALIBRATION_NODE_STACK_SIZE, Config::Task::ACCELEROMETER_TO_BODY_CALIBRATION_NODE_PRIORITY> m_thread;

    Vector3f m_worldUpVector{};
    Vector3f m_worldRightVector{};

    Welford<Vector3f> m_upVectorAccumulator{};
    Welford<Vector3f> m_rightVectorAccumulator{};

    QueueHandle_t m_queue{};
    StaticQueue_t m_queueBuffer{};
    uint8_t m_queueStorage[Config::ACCELEROMETER_TO_BODY_CALIBRATION_NODE_BUFFER_SIZE * sizeof(Quaternion<float>)]{};
};
