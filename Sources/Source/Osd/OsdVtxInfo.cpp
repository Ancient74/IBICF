#include "OsdVtxInfo.h"
#include <cstdio>

OsdVtxInfo::OsdVtxInfo(OsdController& osd, TrampProtocolController& controller)
        : m_osd(osd)
        , m_controller(controller)
        , m_xPos(m_osd.Cols() - 10)
        , m_yPos(2)
{
}

void OsdVtxInfo::RenderIfPending()
{
    const auto status = m_controller.GetStatus();
    const auto power = status->actualPowerMw;
    const auto isPit = status->pitMode;
    if (m_currentPower == power && m_currentPit == isPit)
        return;
    m_currentPit = isPit;
    m_currentPower = power;

    char buffer[8] = "PIT md";
    if (!isPit)
        std::snprintf(buffer, sizeof(buffer), "%dmv", power);

    std::array<OsdCell, 8> cells;
    cells[0].charCode = VtxIcon;
    cells[0].col = m_xPos;
    cells[0].row = m_yPos;
    for (uint8_t i = 1; i < cells.size(); i++)
    {
        cells[i].charCode = s_symbols[buffer[i-1]];
        cells[i].col = m_xPos + i;
        cells[i].row = m_yPos;
    }
    m_osd.DrawCells(cells);
}
