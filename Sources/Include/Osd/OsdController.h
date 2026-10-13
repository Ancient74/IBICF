#pragma once
#include "OsdModule.h"
#include <array>
#include <cstddef>

struct OsdCell
{
    uint8_t row;
    uint8_t col;
    uint8_t charCode;
};

class OsdController
{
public:
    explicit OsdController(OsdModule& module);

    uint8_t Rows() const;
    static constexpr uint8_t Cols() { return s_cols; }

    void DrawCells(std::span<const OsdCell> cells);

private:
    static constexpr size_t s_maxCellCount = 16 * Config::OSD_COLS;
    std::array<OsdCell, s_maxCellCount> m_lastCells{};
    size_t m_lastCellCount = 0;
    OsdModule& m_module;
    uint8_t m_rows;
    static constexpr uint8_t s_cols = Config::OSD_COLS;
};
