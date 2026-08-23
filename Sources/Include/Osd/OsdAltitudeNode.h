#pragma once
#include "OsdRenderer.h"

class OsdAltitudeNode : public OsdRenderer<OsdAltitudeNode, float>
{
public:
    OsdAltitudeNode(std::shared_ptr<OsdController> osd)
        : OsdRenderer<OsdAltitudeNode, float>(std::move(osd))
        , m_xPos(GetOsdController()->Cols() - 7)
        , m_yPos(GetOsdController()->Rows() - 6)
    {
    }

    void Render(float altitude)
    {
        char buffer[8];
        std::snprintf(buffer, sizeof(buffer), "%.1fm", altitude);

        std::array<OsdCell, 9> cells;
        cells[0].charCode = AltCharCode;
        cells[0].col = m_xPos;
        cells[0].row = m_yPos;
        for (uint8_t i = 1; i < cells.size(); i++)
        {
            cells[i].charCode = s_symbols[buffer[i-1]];
            cells[i].col = m_xPos + i;
            cells[i].row = m_yPos;
        }
        GetOsdController()->DrawCells(cells);
    }

private:
    static constexpr uint8_t AltCharCode = 0xCA;

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
        result['.'] = 0x80;
        result['m'] = 0x31;
        result['-'] = 0x7F;
        result[' '] = 0x00;

        return result;
    }();

    const uint8_t m_xPos;
    const uint8_t m_yPos;
};
