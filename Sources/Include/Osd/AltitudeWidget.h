#pragma once
#include "OsdWidget.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

class AltitudeWidget : public OsdWidget<AltitudeWidget, float, 1, 7>
{
    using Base = OsdWidget<AltitudeWidget, float, 1, 7>;

public:
    explicit AltitudeWidget(float altitude = 0.0f)
        : Base(altitude)
    {
    }

    Content RenderContent(const float& altitude) const
    {
        Content content{};
        content[0][0] = OsdChar::Altitude;
        content[0][6] = OsdChar::Meter;

        char number[6]{};
        const float absoluteAltitude = std::fabs(altitude);
        int32_t length = 0;
        if (absoluteAltitude < 1000.0f)
            length = std::snprintf(number, sizeof(number), "%5.1f", absoluteAltitude);

        if (absoluteAltitude >= 1000.0f || length > 5)
            length = std::snprintf(number, sizeof(number), "%5.0f", absoluteAltitude);

        if (length > 5)
        {
            for (std::size_t i = 0; i < 5; ++i)
                content[0][i + 1] = OsdChar::Max;
            return content;
        }

        std::size_t leadingSpaces = 0;
        while (leadingSpaces < 5 && number[leadingSpaces] == ' ')
            ++leadingSpaces;

        for (std::size_t i = 0; i < 5; ++i)
            content[0][i + 1] = OsdCharFromChar(number[i]);

        if (altitude < 0.0f)
        {
            if (leadingSpaces == 0)
            {
                for (std::size_t i = 0; i < 5; ++i)
                    content[0][i + 1] = OsdChar::Max;
                return content;
            }

            content[0][leadingSpaces] = OsdChar::Minus;
        }

        return content;
    }
};
