#pragma once
#include "OsdRenderer.h"

class OsdFlightModeNode : public OsdRenderer<OsdFlightModeNode, std::span<const char>>
{
public:
    explicit OsdFlightModeNode(OsdController& osd)
        : OsdRenderer<OsdFlightModeNode, std::span<const char>>(osd)
        , m_yPos(GetOsdController().Rows() - 1)
    {
    }

    void Render(std::span<const char> flightMode)
    {
        constexpr auto cols = OsdController::Cols();
        const uint8_t len = static_cast<uint8_t>(std::min<size_t>(flightMode.size(), cols));
        const uint8_t startCol = static_cast<uint8_t>((cols - len) / 2);

        std::array<OsdCell, cols> cells{};
        for (uint8_t i = 0; i < len; i++)
        {
            cells[i].charCode = s_symbols[static_cast<unsigned char>(flightMode[i])];
            cells[i].col = startCol + i;
            cells[i].row = m_yPos;
        }

        GetOsdController().DrawCells(std::span<OsdCell>(cells.data(), len));
    }

private:
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

        result['A'] = 0x0B;
        result['B'] = 0x0C;
        result['C'] = 0x0D;
        result['D'] = 0x0E;
        result['E'] = 0x0F;
        result['F'] = 0x10;
        result['G'] = 0x11;
        result['H'] = 0x12;
        result['I'] = 0x13;
        result['J'] = 0x14;
        result['K'] = 0x15;
        result['L'] = 0x16;
        result['M'] = 0x17;
        result['N'] = 0x18;
        result['O'] = 0x19;
        result['P'] = 0x1A;
        result['Q'] = 0x1B;
        result['R'] = 0x1C;
        result['S'] = 0x1D;
        result['T'] = 0x1E;
        result['U'] = 0x1F;
        result['V'] = 0x20;
        result['W'] = 0x21;
        result['X'] = 0x22;
        result['Y'] = 0x23;
        result['Z'] = 0x24;

        result['a'] = 0x25;
        result['b'] = 0x26;
        result['c'] = 0x27;
        result['d'] = 0x28;
        result['e'] = 0x29;
        result['f'] = 0x2A;
        result['g'] = 0x2B;
        result['h'] = 0x2C;
        result['i'] = 0x2D;
        result['j'] = 0x2E;
        result['k'] = 0x2F;
        result['l'] = 0x30;
        result['m'] = 0x31;
        result['n'] = 0x32;
        result['o'] = 0x33;
        result['p'] = 0x34;
        result['q'] = 0x35;
        result['r'] = 0x36;
        result['s'] = 0x37;
        result['t'] = 0x38;
        result['u'] = 0x39;
        result['v'] = 0x3A;
        result['w'] = 0x3B;
        result['x'] = 0x3C;
        result['y'] = 0x3D;
        result['z'] = 0x3E;

        return result;
    }();

    const uint8_t m_yPos;
};
