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

    static constexpr uint8_t Rows() { return Config::OSD_ROWS; }
    static constexpr uint8_t Cols() { return Config::OSD_COLS; }

    void DrawCells(std::span<const OsdCell> cells);

private:
    static constexpr size_t s_maxCellCount = Config::OSD_ROWS * Config::OSD_COLS;
    std::array<std::array<uint8_t, Config::OSD_COLS>, Config::OSD_ROWS> m_lastChars{};
    OsdModule& m_module;
};
