#pragma once
#include "BatterySensorNode.h"
#include "OsdWidget.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

class BatteryWidget : public OsdWidget<BatteryWidget, BatteryInfo, 1, 6>
{
    using Base = OsdWidget<BatteryWidget, BatteryInfo, 1, 6>;

public:
    explicit BatteryWidget(BatteryInfo initialInfo = {})
        : Base(initialInfo)
    {
    }

    Content RenderContent(const BatteryInfo& info) const
    {
        Content content{};
        content[0][0] = BatteryIcon(info.percentLevel);
        content[0][5] = OsdChar::Volt;

        char number[6]{};
        const float absoluteVoltage = std::fabs(info.chargeLevel);
        int32_t length = 0;
        if (absoluteVoltage < 10.0f)
            length = std::snprintf(number, sizeof(number), "%4.2f", info.chargeLevel);
        else if (absoluteVoltage < 100.0f)
            length = std::snprintf(number, sizeof(number), "%4.1f", info.chargeLevel);
        else
            length = std::snprintf(number, sizeof(number), "%4.0f", info.chargeLevel);

        if (length > 4 && absoluteVoltage < 10.0f)
            length = std::snprintf(number, sizeof(number), "%4.1f", info.chargeLevel);

        if (length > 4)
            length = std::snprintf(number, sizeof(number), "%4.0f", info.chargeLevel);

        if (length > 4)
        {
            for (std::size_t i = 0; i < 4; ++i)
                content[0][i + 1] = OsdChar::Max;
            return content;
        }

        for (std::size_t i = 0; i < 4; ++i)
            content[0][i + 1] = OsdCharFromChar(number[i]);

        return content;
    }

private:
    static OsdChar BatteryIcon(uint8_t percentLevel)
    {
        static constexpr OsdChar Icons[] = {
            OsdChar::MainBattery,
            OsdChar::BatteryEmpty,
            OsdChar::Battery1,
            OsdChar::Battery2,
            OsdChar::Battery3,
            OsdChar::Battery4,
            OsdChar::Battery5,
            OsdChar::BatteryFull,
        };
        const auto clampedPercent = std::min<uint8_t>(percentLevel, 100);
        const auto level = std::min<uint8_t>(clampedPercent * 8 / 100, 7);
        return Icons[level];
    }
};
