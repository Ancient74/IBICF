#pragma once
#include "OsdWidget.h"
#include "TrampProtocol.h"
#include <cstdint>

class VtxInfoWidget : public OsdWidget<VtxInfoWidget, VtxInfo, 1, 7>
{
    using Base = OsdWidget<VtxInfoWidget, VtxInfo, 1, 7>;

public:
    using Content = typename Base::Content;

    explicit VtxInfoWidget(VtxInfo initialInfo = {})
        : Base(initialInfo)
    {
    }

    Content RenderContent(const VtxInfo& info) const
    {
        Content content{};
        content[0].fill(OsdChar::Space);
        content[0][0] = OsdChar::V;
        content[0][1] = OsdChar::T;
        content[0][2] = OsdChar::X;

        uint32_t power = info.actualPowerMw;
        if (power > 9999)
        {
            for (std::size_t i = 3; i < content[0].size(); ++i)
                content[0][i] = OsdChar::Max;
            return content;
        }

        for (std::size_t i = 0; i < 4; ++i)
        {
            const auto digit = power % 10;
            content[0][6 - i] = static_cast<OsdChar>(
                static_cast<uint8_t>(OsdChar::Digit0) + digit);
            power /= 10;
            if (power == 0)
                break;
        }

        return content;
    }
};
