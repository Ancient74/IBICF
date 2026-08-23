#pragma once

#include <vector>

#include "CommonTask.h"
#include "OsdRenderer.h"

template<uint8_t RenderersAmount>
class OsdCompositorTask : public CommonTask<OsdCompositorTask<RenderersAmount>, 512, configMAX_PRIORITIES / 2 - 2>
{
    using Base = CommonTask<OsdCompositorTask<RenderersAmount>, 512, configMAX_PRIORITIES / 2 - 2>;
    using Base::Start;

public:
    OsdCompositorTask(const char* taskName, uint16_t updateRateHz)
        : Base(taskName, pdMS_TO_TICKS(1000 / updateRateHz))
    {
        m_renderables.reserve(RenderersAmount);
    }

    ~OsdCompositorTask() { this->Stop(); }

    void Register(IOsdRenderable& renderable)
    {
        configASSERT(m_renderables.size() < RenderersAmount);
        m_renderables.push_back(&renderable);
    }

    void StartCompositor() { Base::Start(); }

    void RunTaskIteration()
    {
        for (auto* r : m_renderables)
            r->RenderIfPending();
    }

private:
    std::vector<IOsdRenderable*> m_renderables;
};
