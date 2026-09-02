#pragma once
#include "OsdRenderer.h"
#include "CrsfLib/Frame.hpp"

class OsdRcLinkNode : public OsdRenderer<OsdRcLinkNode, CrsfLib::LinkStatisticsFrame>
{
public:
    explicit OsdRcLinkNode(OsdController& osd)
        : OsdRenderer<OsdRcLinkNode, CrsfLib::LinkStatisticsFrame>(osd)
        , m_xPos(0)
        , m_yPos(2)
    {
    }

    void Render(const CrsfLib::LinkStatisticsFrame& frame)
    {
        char lqText[6];
        std::snprintf(lqText, sizeof(lqText), "LQ%d", frame.upLinkQuality);

        char rssiText[6];
        std::snprintf(rssiText, sizeof(rssiText), "-%d", frame.upRssiAnt1);

        constexpr size_t MaxCells = (sizeof(lqText) - 1) + 1 + (sizeof(rssiText) - 1);
        std::array<OsdCell, MaxCells> cells{};
        size_t idx = 0;

        uint8_t col = m_xPos;
        for (const char* p = lqText; *p != '\0'; ++p)
        {
            cells[idx].charCode = s_symbols[static_cast<uint8_t>(*p)];
            cells[idx].col = col++;
            cells[idx].row = m_yPos;
            ++idx;
        }

        col = m_xPos;
        cells[idx].charCode = LinkIcon;
        cells[idx].col = col++;
        cells[idx].row = m_yPos + 1;
        ++idx;

        for (const char* p = rssiText; *p != '\0'; ++p)
        {
            cells[idx].charCode = s_symbols[static_cast<uint8_t>(*p)];
            cells[idx].col = col++;
            cells[idx].row = m_yPos + 1;
            ++idx;
        }

        GetOsdController().DrawCells(std::span<OsdCell>(cells.data(), idx));
    }

private:
    static constexpr uint8_t LinkIcon = 0xC3;

    static constexpr std::array<uint8_t, 256> s_symbols = [] {
        std::array<uint8_t, 256> result{};

        result['0'] = 0x01;
        result['1'] = 0x02;
        result['2'] = 0x03;
        result['3'] = 0x04;
        result['4'] = 0x05;
        result['5'] = 0x06;
        result['6'] = 0x07;
        result['7'] = 0x08;
        result['8'] = 0x09;
        result['9'] = 0x0A;
        result['L'] = 0x16;
        result['Q'] = 0x1B;
        result['-'] = 0x7F;
        result[' '] = 0x00;

        return result;
    }();

    const uint8_t m_xPos;
    const uint8_t m_yPos;
};
