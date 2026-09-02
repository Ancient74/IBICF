#pragma once
#include "OsdModule.h"

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
    std::vector<OsdCell> m_lastCells;
    OsdModule& m_module;
    uint8_t m_rows;
    static constexpr uint8_t s_cols = Config::OSD_COLS;
};
