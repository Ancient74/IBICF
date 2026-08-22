#pragma once
#include <KalmanLib/Quaternion.hpp>
#include "OsdHorizonRenderer.h"
#include "OsdRenderingNode.h"

class OsdHorizonNode : public OsdRenderingNode<OsdHorizonNode, Quaternion<float>>
{
public:
    OsdHorizonNode(std::shared_ptr<OsdController> osd, const char* taskName, uint16_t updateRateHz)
        : OsdRenderingNode<OsdHorizonNode, Quaternion<float>>(std::move(osd), taskName, updateRateHz)
        , m_renderer(osd ? osd->Rows() : 0, osd ? osd->Cols() : 0)
    {
    }

    void Render(Quaternion<float> quat)
    {
        OsdHorizonRenderer<>::CellList cells{};
        const uint8_t count = m_renderer.Compute(quat.ToAttitude(), cells);
        std::span<const OsdCell> cellsSpan(cells.data(), count);
        GetOsdController()->DrawCells(cellsSpan);
    }

private:
    OsdHorizonRenderer<> m_renderer;
};
