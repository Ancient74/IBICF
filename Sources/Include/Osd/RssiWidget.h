#pragma once
#include "OsdWidget.h"
#include <array>
#include <cstdint>

class RssiWidget : public OsdWidget<RssiWidget, uint8_t, 1, 5>
{
    using Base = OsdWidget<RssiWidget, uint8_t, 1, 5>;

public:
    using Content = typename Base::Content;

    explicit RssiWidget(uint8_t rssi = 0)
        : Base(rssi)
    {
    }

    Content RenderContent(const uint8_t& rssi) const
    {
        Content content{};
        content[0].fill(OsdChar::Space);
        content[0][0] = OsdChar::Rssi;
        content[0][1] = OsdChar::Minus;

        std::array<uint8_t, 3> digits{};
        std::size_t digitCount = 0;
        uint8_t value = rssi;
        do
        {
            digits[digitCount++] = value % 10;
            value /= 10;
        } while (value != 0 && digitCount < digits.size());

        for (std::size_t i = 0; i < digitCount; ++i)
            content[0][i + 2] = static_cast<OsdChar>(
                static_cast<uint8_t>(OsdChar::Digit0) + digits[digitCount - 1 - i]);

        return content;
    }
};
