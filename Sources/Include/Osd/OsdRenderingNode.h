#pragma once
#include "CommonTask.h"
#include <optional>
#include <PubSubLib/Node.hpp>
#include "OsdController.h"

template<typename Derived, typename RenderingData>
class OsdRenderingNode
    : public CommonTask<Derived, 512, configMAX_PRIORITIES / 2 - 2>
    , public NodeInput<Derived, RenderingData>
{
    using Base = CommonTask<Derived, 512, configMAX_PRIORITIES / 2 - 2>;

public:
    OsdRenderingNode(std::shared_ptr<OsdController> osd, const char* taskName, uint16_t updateRateHz)
        : Base(taskName, pdMS_TO_TICKS(1000 / updateRateHz))
        , m_osd(std::move(osd))
    {
        static_assert(requires(Derived& renderingNode, RenderingData data) {
            { renderingNode.Render(data) } -> std::same_as<void>;
        });
        m_mutex = xSemaphoreCreateMutex();
        configASSERT(m_mutex != nullptr);
        this->Start();
    }

    ~OsdRenderingNode()
    {
        this->Stop();
        vSemaphoreDelete(m_mutex);
    }

    void Process(RenderingData renderingData)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_latestData = renderingData;
        xSemaphoreGive(m_mutex);
    }

    void RunTaskIteration()
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        auto data = m_latestData;
        m_latestData = std::nullopt;
        xSemaphoreGive(m_mutex);
        if (!data.has_value())
            return;

        auto* self = static_cast<Derived*>(this);
        self->Render(data.value());
    }

protected:
    std::shared_ptr<OsdController> GetOsdController() const { return m_osd; }

private:
    std::shared_ptr<OsdController> m_osd;
    std::optional<RenderingData> m_latestData{};
    SemaphoreHandle_t m_mutex{};
};
