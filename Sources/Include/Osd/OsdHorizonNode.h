#pragma once
#include <KalmanLib/Quaternion.hpp>
#include "OsdController.h"
#include "CommonTask.h"
#include <optional>
#include <PubSubLib/Node.hpp>

class OsdHorizonNode
    : public CommonTask<OsdHorizonNode, 512, configMAX_PRIORITIES / 2 - 1>
    , public NodeInput<OsdHorizonNode, Quaternion<float>>
{
    using Base = CommonTask<OsdHorizonNode, 512, configMAX_PRIORITIES / 2 - 1>;

public:
    using InputType = Quaternion<float>;

    OsdHorizonNode(std::shared_ptr<OsdController> osd, uint16_t updateRateHz)
        : Base("OsdHorizon", pdMS_TO_TICKS(1000 / updateRateHz)),
          m_osd(osd), m_renderer(osd->Rows(), osd->Cols())
    {
        m_mutex = xSemaphoreCreateMutex();
        configASSERT(m_mutex != nullptr);
        Start();
    }

    ~OsdHorizonNode()
    {
        Stop();
        vSemaphoreDelete(m_mutex);
    }

    void Process(const Quaternion<float>& quat)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_latestQuat = quat;
        xSemaphoreGive(m_mutex);
    }

    void RunTaskIteration()
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        auto quat = m_latestQuat;
        m_latestQuat = std::nullopt;
        xSemaphoreGive(m_mutex);
        if (!quat.has_value())
            return;

        HorizonRenderer<>::CellList cells{};
        const uint8_t count = m_renderer.Compute(quat->ToAttitude(), cells);
        m_osd->DrawCells({cells.data(), count});
    }

private:
    std::shared_ptr<OsdController> m_osd;
    HorizonRenderer<> m_renderer;
    std::optional<Quaternion<float>> m_latestQuat{};
    SemaphoreHandle_t m_mutex{};
};
