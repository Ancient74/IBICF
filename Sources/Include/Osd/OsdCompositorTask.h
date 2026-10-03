#pragma once
#include <array>
#include <cstdint>
#include "TaskThread.h"
#include "Config.h"
#include "OsdRenderer.h"

template<uint8_t RenderersAmount>
class OsdCompositorTask
{
public:
    OsdCompositorTask(const char* taskName, uint16_t updateRateHz)
        : m_thread(taskName, pdMS_TO_TICKS(1000 / updateRateHz), *this) {}

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
    TaskThread<OsdCompositorTask<RenderersAmount>, Config::Task::OSD_COMPOSITOR_TASK_STACK_SIZE, Config::Task::OSD_COMPOSITOR_TASK_PRIORITY> m_thread;
    std::array<IOsdRenderable*, RenderersAmount> m_renderables{};
    uint8_t m_count{0};
};
