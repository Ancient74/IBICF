#pragma once
#include <array>
#include <cstdint>
#include "CommonTask.h"
#include "OsdRenderer.h"

template<uint8_t RenderersAmount>
class OsdCompositorTask : public CommonTask<OsdCompositorTask<RenderersAmount>, 256, configMAX_PRIORITIES / 2 - 2>
{
    using Base = CommonTask<OsdCompositorTask<RenderersAmount>, 256, configMAX_PRIORITIES / 2 - 2>;
    using Base::Start;

public:
    OsdCompositorTask(const char* taskName, uint16_t updateRateHz)
        : Base(taskName, pdMS_TO_TICKS(1000 / updateRateHz)) {}

    ~OsdCompositorTask() { this->Stop(); }

    void Register(IOsdRenderable& renderable)
    {
        configASSERT(m_count < RenderersAmount);
        m_renderables[m_count++] = &renderable;
    }

    void StartCompositor() { Base::Start(); }

    void RunTaskIteration()
    {
        for (uint8_t i = 0; i < m_count; ++i)
            m_renderables[i]->RenderIfPending();
    }

private:
    std::array<IOsdRenderable*, RenderersAmount> m_renderables{};
    uint8_t m_count{0};
};
