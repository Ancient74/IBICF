#pragma once
#include <memory>
#include "FreeRTOS.h"
#include "semphr.h"
#include "SensorStatus.h"
#include "SensorConcepts.h"
#include "CommonTask.h"
#include <PubSubLib/Node.hpp>

template<typename SensorT, typename SensorOutputT, uint8_t MaxSubs>
requires SensorDevice<SensorT, SensorOutputT>
class SensorSamplingNode
    : public NodeOutput<SensorSamplingNode<SensorT, SensorOutputT, MaxSubs>, SensorOutputT, MaxSubs>
    , public CommonTask<SensorSamplingNode<SensorT, SensorOutputT, MaxSubs>, 128, configMAX_PRIORITIES / 2>
{
    using Base = CommonTask<SensorSamplingNode<SensorT, SensorOutputT, MaxSubs>, 128, configMAX_PRIORITIES / 2>;

public:
    using OutputType = SensorOutputT;

    SensorSamplingNode(std::unique_ptr<SensorT> sensor, const char* taskName)
        : Base(taskName, pdMS_TO_TICKS(1000 / sensor->GetSamplingRate())),
          m_sensor(std::move(sensor)) {}

    ~SensorSamplingNode()
    {
        this->Stop();
    }

    void RunTaskIteration()
    {
        SensorOutputT raw{};
        if (m_sensor->GetRawValues(raw) == Status::Ok)
        {
            m_sensor->Normalize(raw);
            this->Publish(raw);
        }
    }

    std::unique_ptr<SensorT> m_sensor;
};
