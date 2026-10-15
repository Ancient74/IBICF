#pragma once
#include "OsdRenderer.h"
#include <array>
#include <cstdint>
#include <cstdio>
#include <span>

class OsdThrustNode : public OsdRenderer<OsdThrustNode, uint8_t>
{
public:
    explicit OsdThrustNode(OsdController& osd)
        : OsdRenderer<OsdThrustNode, uint8_t>(osd)
    {
    }

    void Render(uint8_t thrust)
    {
        char text[6];
        std::snprintf(text, sizeof(text), "T%u%%", static_cast<unsigned>(thrust));

        std::array<OsdCell, 5> cells{};
        uint8_t length = 0;
        while (length < cells.size() && text[length] != '\0')
            ++length;
        for (uint8_t i = 0; i < length; ++i)
        {
            cells[i].charCode = s_symbols[static_cast<unsigned char>(text[i])];
            cells[i].col = i;
            cells[i].row = 0;
        }
        GetOsdController().DrawCells(std::span<const OsdCell>(cells.data(), length));
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
        result['T'] = 0x1E;
        result['%'] = 0x25;
        return result;
    }();
};
