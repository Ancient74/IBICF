#include "OsdController.h"
#include <algorithm>

OsdController::OsdController(std::unique_ptr<OsdModule> module)
    : m_module(std::move(module))
{
    m_module->ClearScreen();
    auto status = m_module->GetInputSignalStatus();
    if (!status)
        return;
    m_rows = status == InputSignalStatus::Ntsc ? 11 : 15;
    m_cols = 29;

    m_module->ConfigureOutput(OutputMode::TextAndVideo, status == InputSignalStatus::Ntsc ? VideoStandard::Ntsc : VideoStandard::Pal, false, false);
    m_module->DisplayCharacter(m_rows >> 2, m_cols >> 2, 0x8E); // Crosshair at the middle
}

uint8_t OsdController::Rows() const
{
    return m_rows;
}

uint8_t OsdController::Cols() const
{
    return m_cols;
}

void OsdController::DrawCells(std::span<const HorizonRenderer<>::Cell> cells)
{
    for (const auto& prev : m_lastCells)
    {
        const bool stillUsed = std::any_of(cells.begin(), cells.end(),
            [&](const auto& c) { return c.row == prev.row && c.col == prev.col; });
        if (!stillUsed)
            m_module->DisplayCharacter(prev.row, prev.col, ' ');
    }
    for (const auto& c : cells)
        m_module->DisplayCharacter(c.row, c.col, c.charCode);

    m_lastCells.assign(cells.begin(), cells.end());
}
