#pragma once
#include <memory>
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
    OsdController(std::unique_ptr<OsdModule> module);

    uint8_t Rows() const;
    uint8_t Cols() const;

    void DrawCells(std::span<const OsdCell> cells);

private:
    std::vector<OsdCell> m_lastCells;
    std::unique_ptr<OsdModule> m_module;
    uint8_t m_rows;
    uint8_t m_cols;
};
