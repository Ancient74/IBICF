#pragma once
#include <cstdint>
#include "TaskThread.h"
#include "Config.h"

template<typename RenderTarget, uint16_t UpdateRateHz = Config::OSD_REFRESH_RATE_HZ>
class OsdRenderer
{
public:
    static_assert(UpdateRateHz > 0, "UpdateRateHz must be greater than zero");

    OsdRenderer(const char* taskName, RenderTarget& renderTarget)
        : m_thread(taskName, *this)
        , m_renderTarget(renderTarget)
    {
    }

    ~OsdRenderer() { m_thread.Stop(); }

    void Start() { m_thread.Start(); }
    void Stop() { m_thread.Stop(); }

    void RunTaskIteration()
    {
        m_renderTarget.RenderIfPending();
    }

private:
    TaskThread<OsdRenderer<RenderTarget, UpdateRateHz>, Config::Task::OSD_RENDERER_TASK_STACK_SIZE,
        Config::Task::OSD_RENDERER_TASK_PRIORITY, pdMS_TO_TICKS(1000 / UpdateRateHz)> m_thread;
    RenderTarget& m_renderTarget;
};
