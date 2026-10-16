#pragma once
#include "OsdWidget.h"
#include <algorithm>
#include <array>
#include <cstdint>

struct LqData
{
    uint8_t rfProfile = 0;
    uint8_t linkQuality = 0;
};

class LqWidget : public OsdWidget<LqWidget, LqData, 1, 7>
{
    using Base = OsdWidget<LqWidget, LqData, 1, 7>;

public:
    using Content = typename Base::Content;

    explicit LqWidget(LqData initialData = {})
        : Base(initialData)
    {
    }

    Content RenderContent(const LqData& data) const
    {
        Content content{};
        content[0].fill(OsdChar::Space);

        const auto profile = data.rfProfile;
        if (profile < 0)
        {
            content[0][0] = OsdChar::Question;
            content[0][1] = OsdChar::Question;
            content[0][2] = OsdChar::Question;
        }
        else
        {
            WriteRightAligned(content, 0, static_cast<uint32_t>(profile));
        }

        content[0][3] = OsdChar::Colon;
        const auto quality = std::min<uint32_t>(data.linkQuality, 100);
        WriteRightAligned(content, 4, quality);

        return content;
    }

private:
    static void WriteRightAligned(Content& content, std::size_t start, uint32_t value)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            content[0][start + 2 - i] = static_cast<OsdChar>(
                static_cast<uint8_t>(OsdChar::Digit0) + value % 10);
            value /= 10;
            if (value == 0)
                break;
        }
    }
};
