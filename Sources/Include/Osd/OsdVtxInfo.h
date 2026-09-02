#pragma once
#include "BatterySensorNode.h"
#include "OsdRenderer.h"
#include "TrampProtocol.h"

class OsdVtxInfo : public IOsdRenderable
{
public:
    OsdVtxInfo(OsdController& osd, TrampProtocolController& controller);

    void RenderIfPending() override;

private:
    static constexpr uint8_t VtxIcon = 0xBE;
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
        result['v'] = 0x3A;
        result['m'] = 0x31;
        result['P'] = 0x2A;
        result['I'] = 0x12;
        result['T'] = 0x1E;
        result[' '] = 0x00;

        return result;
    }();

    OsdController& m_osd;
    TrampProtocolController& m_controller;
    const uint8_t m_xPos;
    const uint8_t m_yPos;
    bool m_currentPit = false;
    uint16_t m_currentPower = false;
};
