#pragma once
#include <memory>
#include "OsdModule.h"
#include "OsdHorizonRenderer.h"

class OsdController
{
public:
    OsdController(std::unique_ptr<OsdModule> module);

    uint8_t Rows() const;
    uint8_t Cols() const;

    void DrawCells(std::span<const HorizonRenderer<>::Cell> cells);

private:
    std::vector<HorizonRenderer<>::Cell> m_lastCells;
    std::unique_ptr<OsdModule> m_module;
    uint8_t m_rows;
    uint8_t m_cols;
};
