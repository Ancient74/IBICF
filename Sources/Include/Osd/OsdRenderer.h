#pragma once
#include "OsdController.h"
#include "PubSubLib/Node.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <atomic>
#include <concepts>
#include <optional>

class IOsdRenderable
{
public:
    virtual ~IOsdRenderable() = default;
    virtual void RenderIfPending() = 0;
};

template<typename Derived, typename RenderingData>
class OsdRenderer : public IOsdRenderable, public NodeInput<Derived, RenderingData>
{
public:
    explicit OsdRenderer(OsdController& osd)
        : m_osd(osd)
    {
        static_assert(requires(Derived& r, RenderingData data) {
            { r.Render(data) } -> std::same_as<void>;
        });
        m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);
        configASSERT(m_mutex != nullptr);
    }

    ~OsdRenderer() override
    {
        vSemaphoreDelete(m_mutex);
    }

    void Process(RenderingData renderingData)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_latestData = renderingData;
        xSemaphoreGive(m_mutex);
    }

    void SetEnabled(bool enabled)
    {
        m_enabled.store(enabled, std::memory_order_release);
    }

    void RenderIfPending() override
    {
        if (!m_enabled.load(std::memory_order_acquire))
            return;

        xSemaphoreTake(m_mutex, portMAX_DELAY);
        auto data = m_latestData;
        m_latestData = std::nullopt;
        xSemaphoreGive(m_mutex);
        if (!data.has_value())
            return;
        static_cast<Derived*>(this)->Render(data.value());
    }

protected:
    OsdController& GetOsdController() const
    {
        return m_osd;
    }

private:
    OsdController& m_osd;
    std::optional<RenderingData> m_latestData{};
    SemaphoreHandle_t m_mutex{};
    StaticSemaphore_t m_mutexBuffer{};
    std::atomic_bool m_enabled{true};
};
