#include "OsdController.h"
#include "Log.h"
#include <cstdint>

namespace
{
    constexpr const char* MODULE = "OSD_CONTROLLER";
}

OsdController::OsdController(OsdModule& module)
    : m_module(module)
{
    m_module.ClearScreen();
    for (auto& row : m_lastChars)
        row.fill(' ');

    const auto status = m_module.GetInputSignalStatus();
    if (!status)
    {
        LogWarning(MODULE, "GetInputSignalStatus return value is empty");
        return;
    }
    LogInfo(MODULE, "GetInputSignalStatus return value: %d", static_cast<uint8_t>(*status));
    const auto mode = status == InputSignalStatus::Ntsc ? Config::NTSC_MODE : Config::PAL_MODE;
    if (mode != Config::OSD_OPERATION_MODE)
        LogError(MODULE, "Input signal mode (%s) does not match configured OSD operation mode (%s)",
                   mode == Config::NTSC_MODE ? "NTSC" : "PAL",
                   Config::OSD_OPERATION_MODE == Config::NTSC_MODE ? "NTSC" : "PAL");

    m_module.ConfigureOutput(OutputMode::TextAndVideo, status == InputSignalStatus::Ntsc ? VideoStandard::Ntsc : VideoStandard::Pal, false, false);
}

void OsdController::DrawCells(std::span<const OsdCell> cells)
{
    if (cells.size() > s_maxCellCount)
    {
        LogWarning(MODULE, "DrawCells received too many cells: %u", static_cast<uint32_t>(cells.size()));
        return;
    }

    std::array<std::array<uint8_t, Config::OSD_COLS>, Config::OSD_ROWS> nextChars{};
    for (auto& row : nextChars)
        row.fill(' ');

    for (const auto& cell : cells)
    {
        if (cell.row >= Config::OSD_ROWS || cell.col >= Config::OSD_COLS)
        {
            LogWarning(MODULE, "DrawCells received out-of-range cell: row=%d col=%d", cell.row, cell.col);
            continue;
        }
        nextChars[cell.row][cell.col] = cell.charCode;
    }

    for (uint8_t row = 0; row < Config::OSD_ROWS; row++)
    {
        for (uint8_t col = 0; col < Config::OSD_COLS; col++)
        {
            if (nextChars[row][col] == m_lastChars[row][col])
                continue;

            m_module.DisplayCharacter(row, col, nextChars[row][col]);
        }
    }

    m_lastChars = nextChars;
}
