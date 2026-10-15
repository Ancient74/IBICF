#include "OsdVtxInfo.h"
#include <cstdio>

void OsdVtxInfo::Render(const VtxInfo& info)
{
    char buffer[8] = "PIT md";
    if (!info.pitMode)
        std::snprintf(buffer, sizeof(buffer), "%umW", static_cast<unsigned>(info.actualPowerMw));

    std::array<OsdCell, 8> cells;
    cells[0].charCode = VtxIcon;
    cells[0].col = m_xPos;
    cells[0].row = m_yPos;
    for (uint8_t i = 1; i < cells.size(); i++)
    {
        cells[i].charCode = s_symbols[static_cast<unsigned char>(buffer[i - 1])];
        cells[i].col = m_xPos + i;
        cells[i].row = m_yPos;
    }
    GetOsdController().DrawCells(cells);
}
