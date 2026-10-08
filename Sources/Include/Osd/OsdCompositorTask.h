#pragma once
#include <array>
#include <cstdint>
#include "TaskThread.h"
#include "Config.h"
#include "OsdRenderer.h"

template<uint8_t RenderersAmount, uint16_t UpdateRateHz = Config::OSD_REFRESH_RATE_HZ>
class OsdCompositorTask
{
public:
    static_assert(UpdateRateHz > 0, "UpdateRateHz must be greater than zero");

    explicit OsdCompositorTask(const char* taskName)
        : m_thread(taskName, *this) {}

    ~OsdCompositorTask() { m_thread.Stop(); }

    void Register(IOsdRenderable& renderable)
    {
        configASSERT(m_count < RenderersAmount);
        m_renderables[m_count++] = &renderable;
    }

    void StartCompositor() { m_thread.Start(); }
    void StopCompositor() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        for (uint8_t i = 0; i < m_count; ++i)
            m_renderables[i]->RenderIfPending();
    }

private:
    TaskThread<OsdCompositorTask<RenderersAmount, UpdateRateHz>, Config::Task::OSD_COMPOSITOR_TASK_STACK_SIZE,
        Config::Task::OSD_COMPOSITOR_TASK_PRIORITY, pdMS_TO_TICKS(1000 / UpdateRateHz)> m_thread;
    std::array<IOsdRenderable*, RenderersAmount> m_renderables{};
    uint8_t m_count{0};
};
