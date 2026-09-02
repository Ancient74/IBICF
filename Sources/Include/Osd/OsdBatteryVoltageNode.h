#pragma once
#include "BatterySensorNode.h"
#include "OsdRenderer.h"

class OsdBatteryVoltageNode : public OsdRenderer<OsdBatteryVoltageNode, BatteryInfo>
{
public:
    explicit OsdBatteryVoltageNode(OsdController& osd)
        : OsdRenderer<OsdBatteryVoltageNode, BatteryInfo>(osd)
        , m_xPos(GetOsdController().Cols() - 6) // -6 for every character before last col (I25.5V)
        , m_yPos(0)
    {
    }

    void Render(BatteryInfo info)
    {
        char buffer[6];
        const auto value = info.chargeLevel;
        if (value < 10.0f)
            std::snprintf(buffer, sizeof(buffer), "%.2fv", value);
        else if (value < 100.0f)
            std::snprintf(buffer, sizeof(buffer), "%.1fv", value);
        else
            std::snprintf(buffer, sizeof(buffer), "%.0fv", value);

        const auto percent = info.percentLevel;
        uint8_t chargeLevelIcon = Charge100CharCode;
        if (percent < 10)
            chargeLevelIcon = Charge0CharCode;
        else if (percent < 15)
            chargeLevelIcon = Charge10CharCode;
        else if (percent < 30)
            chargeLevelIcon = Charge15CharCode;
        else if (percent < 50)
            chargeLevelIcon = Charge30CharCode;
        else if (percent < 80)
            chargeLevelIcon = Charge50CharCode;
        else if (percent < 90)
            chargeLevelIcon = Charge80CharCode;

        std::array<OsdCell, 6> cells;
        cells[0].charCode = chargeLevelIcon;
        cells[0].col = m_xPos;
        cells[0].row = m_yPos;
        for (uint8_t i = 1; i < cells.size(); i++)
        {
            cells[i].charCode = s_symbols[buffer[i-1]];
            cells[i].col = m_xPos + i;
            cells[i].row = m_yPos;
        }
        GetOsdController().DrawCells(cells);
    }

private:
    static constexpr uint8_t Charge100CharCode = 0xE9;
    static constexpr uint8_t Charge80CharCode = 0xEA;
    static constexpr uint8_t Charge50CharCode = 0xEB;
    static constexpr uint8_t Charge30CharCode = 0xEC;
    static constexpr uint8_t Charge15CharCode = 0xED;
    static constexpr uint8_t Charge10CharCode = 0xEE;
    static constexpr uint8_t Charge0CharCode = 0xEF;

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
        result['v'] = 0x3A;
        result[' '] = 0x00;

        return result;
    }();

    const uint8_t m_xPos;
    const uint8_t m_yPos;
};
