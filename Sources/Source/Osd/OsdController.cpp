#include "OsdController.h"
#include "Log.h"
#include <algorithm>

namespace
{
    constexpr const char* MODULE = "OSD_CONTROLLER";
}

OsdController::OsdController(OsdModule& module)
    : m_module(module)
{
    m_module.ClearScreen();
    const auto status = m_module.GetInputSignalStatus();
    if (!status)
    {
        LogWarning(MODULE, "GetInputSignalStatus return value is empty");
        return;
    }
    LogInfo(MODULE, "GetInputSignalStatus return value: %d", static_cast<uint8_t>(*status));
    m_rows = status == InputSignalStatus::Ntsc ? 11 : 15;

    m_module.ConfigureOutput(OutputMode::TextAndVideo, status == InputSignalStatus::Ntsc ? VideoStandard::Ntsc : VideoStandard::Pal, false, false);
    m_module.DisplayCharacter(m_rows >> 2, s_cols >> 2, 0x8E); // Crosshair at the middle
}

uint8_t OsdController::Rows() const
{
    return m_rows;
}

void OsdController::DrawCells(std::span<const OsdCell> cells)
{
    for (const auto& prev : m_lastCells)
    {
        const bool stillUsed = std::any_of(cells.begin(), cells.end(),
            [&](const auto& c) { return c.row == prev.row && c.col == prev.col; });
        if (!stillUsed)
            m_module.DisplayCharacter(prev.row, prev.col, ' ');
    }
    for (const auto& c : cells)
        m_module.DisplayCharacter(c.row, c.col, c.charCode);

    m_lastCells.assign(cells.begin(), cells.end());
}
