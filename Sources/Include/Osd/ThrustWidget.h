#pragma once
#include "OsdWidget.h"
#include <algorithm>
#include <cstdint>

class ThrustWidget : public OsdWidget<ThrustWidget, uint8_t, 1, 4>
{
    using Base = OsdWidget<ThrustWidget, uint8_t, 1, 4>;

public:
    using Content = typename Base::Content;

    explicit ThrustWidget(uint8_t throttle = 0)
        : Base(throttle)
    {
    }

    Content RenderContent(uint8_t throttle) const
    {
        Content content{};
        content[0].fill(OsdChar::Space);
        content[0][0] = OsdChar::Throttle;

        uint8_t value = std::min<uint8_t>(throttle, 100);
        for (std::size_t i = 0; i < 3; ++i)
        {
            content[0][3 - i] = static_cast<OsdChar>(
                static_cast<uint8_t>(OsdChar::Digit0) + value % 10);
            value /= 10;
            if (value == 0)
                break;
        }

        return content;
    }
};
