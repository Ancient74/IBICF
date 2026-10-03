#pragma once
#include "FreeRTOS.h"
#include "semphr.h"
#include "SensorStatus.h"
#include "SensorConcepts.h"
#include "TaskThread.h"
#include "Config.h"
#include <PubSubLib/Node.hpp>

template<typename SensorT, typename SensorOutputT, uint8_t MaxSubs>
requires SensorDevice<SensorT, SensorOutputT>
class SensorSamplingNode
    : public NodeOutput<SensorSamplingNode<SensorT, SensorOutputT, MaxSubs>, SensorOutputT, MaxSubs>
{
public:
    using OutputType = SensorOutputT;

    SensorSamplingNode(SensorT& sensor, const char* taskName)
        : m_thread(taskName, pdMS_TO_TICKS(1000 / sensor.GetSamplingRate()), *this),
          m_sensor(sensor) {}

    ~SensorSamplingNode()
    {
        this->Stop();
    }

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        SensorOutputT raw{};
        if (m_sensor.GetRawValues(raw) == Status::Ok)
        {
            m_sensor.Normalize(raw);
            this->Publish(raw);
        }
    }

    TaskThread<SensorSamplingNode<SensorT, SensorOutputT, MaxSubs>, Config::Task::SENSOR_SAMPLING_NODE_STACK_SIZE, Config::Task::SENSOR_SAMPLING_NODE_PRIORITY> m_thread;
    SensorT& m_sensor;
};
