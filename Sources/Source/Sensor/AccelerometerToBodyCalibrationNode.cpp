#include "AccelerometerToBodyCalibrationNode.h"
#include "Log.h"

namespace
{
    constexpr const char* MODULE = "IMU Calibration";
}

AccelerometerToBodyCalibrationNode::AccelerometerToBodyCalibrationNode(const char* taskName, const Vector3f& worldUpVector, const Vector3f& worldRightVector)
        : m_thread{taskName, *this}
        , m_worldUpVector{worldUpVector}
        , m_worldRightVector{worldRightVector}
{
    m_queue = xQueueCreateStatic(Config::ACCELEROMETER_TO_BODY_CALIBRATION_NODE_BUFFER_SIZE, sizeof(Quaternion<float>), m_queueStorage, &m_queueBuffer);
    configASSERT(m_queue != nullptr);
}

AccelerometerToBodyCalibrationNode::~AccelerometerToBodyCalibrationNode()
{
    m_thread.Stop();
    vQueueDelete(m_queue);
}

void AccelerometerToBodyCalibrationNode::Process(bool switchState)
{
    if (switchState == m_lastSwitchState)
        return;
    m_lastSwitchState = switchState;
    if (!switchState)
        return;

    switch (m_calibrationState)
    {
        case CalibrationState::Idle:
        {
            m_calibrationState = CalibrationState::CalculateUp;
            LogInfo(MODULE, "Starting accelerometer to body calibration. Please keep the aircraft level and still.");
            m_thread.Start();
            break;
        }
        case CalibrationState::CalculateUp:
        {
            m_calibrationState = CalibrationState::WaitingForRight;
            LogInfo(MODULE, "Accelerometer to body calibration: Up vector calculated. Please rotate the aircraft to the right position.");
            break;
        }
        case CalibrationState::WaitingForRight:
        {
            m_calibrationState = CalibrationState::CalculateRight;
            LogInfo(MODULE, "Accelerometer to body calibration: Right vector calculation started. Please keep the aircraft still.");
            break;
        }
        case CalibrationState::CalculateRight:
        {
            m_thread.Stop();
            m_calibrationState = CalibrationState::Idle;
            LogInfo(MODULE, "Accelerometer to body calibration complete.");
            const auto upVector = m_upVectorAccumulator.Mean();
            auto rightVector = m_rightVectorAccumulator.Mean();

            rightVector = rightVector - (DotProduct(rightVector, upVector) * upVector);
            rightVector = -rightVector / Norm(rightVector);

            LogInfo(MODULE, "Up vector: [%.4f, %.4f, %.4f]", upVector(0, 0), upVector(1, 0), upVector(2, 0));
            LogInfo(MODULE, "Right vector: [%.4f, %.4f, %.4f]", rightVector(0, 0), rightVector(1, 0), rightVector(2, 0));
            m_upVectorAccumulator.Reset();
            m_rightVectorAccumulator.Reset();
            break;
        }
    }
}

void AccelerometerToBodyCalibrationNode::Process(const Quaternion<float>& rotation)
{
    xQueueSend(m_queue, &rotation, 0);
}

void AccelerometerToBodyCalibrationNode::RunTaskIteration()
{
    Quaternion<float> rotation{};
    if (xQueueReceive(m_queue, &rotation, pdMS_TO_TICKS(Config::ACCELEROMETER_TO_BODY_CALIBRATION_NODE_TIMEOUT_MS)) != pdTRUE)
        return;

    switch (m_calibrationState)
    {
        case CalibrationState::CalculateUp:
        {
            auto upVector = rotation.Rotate(m_worldUpVector);
            m_upVectorAccumulator.Add(upVector);
            break;
        }
        case CalibrationState::CalculateRight:
        {
            auto rightVector = rotation.Rotate(m_worldRightVector);
            m_rightVectorAccumulator.Add(rightVector);
            break;
        }
        default:
            break;
    }
}
