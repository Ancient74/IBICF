#pragma once
#include "Config.h"
#include "OsdWidget.h"
#include <KalmanLib/Quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

class HorizonLineWidget : public OsdWidget<HorizonLineWidget, Attitude, Config::OSD_ROWS - 4, 9>
{
    using Base = OsdWidget<HorizonLineWidget, Attitude, Config::OSD_ROWS - 4, 9>;
public:
    using Content = typename Base::Content;
    static constexpr std::size_t Cols = 9;
    static constexpr std::size_t VisibleRows = Config::OSD_ROWS - 4;

    explicit HorizonLineWidget(const Quaternion<float>& orientation)
        : Base(orientation.ToAttitude(Config::WORLD_UP_VECTOR))
    {
    }

    void Set(const Quaternion<float>& orientation)
    {
        Base::Set(orientation.ToAttitude(Config::WORLD_UP_VECTOR));
    }

    Content RenderContent(const Attitude& data) const
    {
        // One "step" = one AhBar glyph position = 2 px. A row has 9 steps.
        constexpr int32_t CharWidthPx  = 12;
        constexpr int32_t CharHeightPx = 18;
        constexpr int32_t StepsPerRow  = 9; // AhBar0..AhBar8
        constexpr int32_t PxPerStep = CharHeightPx / StepsPerRow; // 2
        constexpr int32_t TotalSteps = static_cast<int32_t>(VisibleRows) * StepsPerRow;
        constexpr float MaxRollRad = 60.0f * std::numbers::pi_v<float> / 180.0f;
        constexpr float MiddleCol = static_cast<float>(Cols - 1) / 2.0f;

        // 1. Where does the line cross the middle column? (level flight = middle of the widget)
        constexpr float HalfPi = std::numbers::pi_v<float> / 2.0f;
        const float pitchFraction = data.pitchRad / HalfPi; // -1 .. +1
        const float centerStep = (TotalSteps / 2.0f) * (1.0f + pitchFraction); // nose up -> line goes down

        // 2. How steep is it? dy = dx * tan(roll), all in pixels.
        const float tanRoll = std::tan(std::clamp(data.rollRad, -MaxRollRad, MaxRollRad));

        Content content{};
        for (auto& row : content)
            row.fill(OsdChar::Space);

        for (std::size_t col = 0; col < Cols; ++col)
        {
            const float dxPx = (static_cast<float>(col) - MiddleCol) * CharWidthPx; // right of centre = positive
            const float dyPx = dxPx * tanRoll;  // +roll: right side goes UP
            const float stepExact = centerStep - dyPx / PxPerStep; // up = smaller step number

            if (!(stepExact >= -0.5f && stepExact < TotalSteps - 0.5f)) // outside the widget (or NaN)
                continue;

            const int32_t step = static_cast<int32_t>(stepExact + 0.5f); // nearest glyph position
            const int32_t row = step / StepsPerRow;
            const int32_t glyph = step % StepsPerRow;
            content[static_cast<std::size_t>(row)][col] = static_cast<OsdChar>(static_cast<int32_t>(OsdChar::AhBar0) + glyph);
        }
        //LogInfo("Horizon", "rol: %f, pitch: %f" , data.pitchRad, data.rollRad);
        return content;
    }
};
