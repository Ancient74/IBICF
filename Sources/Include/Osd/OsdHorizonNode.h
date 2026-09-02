#pragma once
#include <KalmanLib/Quaternion.hpp>
#include "OsdRenderer.h"

class OsdHorizonNode : public OsdRenderer<OsdHorizonNode, Quaternion<float>>
{
public:
    explicit OsdHorizonNode(OsdController& osd)
        : OsdRenderer<OsdHorizonNode, Quaternion<float>>(osd)
    {
    }

    void Render(const Quaternion<float>& quat)
    {
        constexpr float CharAspect = 12.0f / 18.0f;
        constexpr float VerticalMarginFrac = 0.2f;

        const auto attitude = quat.ToAttitude();
        const auto rows = GetOsdController().Rows();
        constexpr auto totalCols = OsdController::Cols();
        constexpr auto cols = totalCols / 2;
        constexpr auto colOffset = (totalCols - cols) / 2;

        const float centerRow = rows / 2.0f;
        const float centerCol = cols / 2.0f;
        const float pitchOffsetRows = (attitude.pitchRad / (static_cast<float>(M_PI) / 2.0f)) * centerRow;

        const uint8_t marginRows = static_cast<uint8_t>(std::lround(rows * VerticalMarginFrac));
        const uint8_t rowMin = marginRows;
        const uint8_t rowMax = rows - marginRows; // exclusive

        uint8_t count = 0;
        std::array<OsdCell, cols> cells;
        for (uint8_t c = 0; c < cols; c++)
        {
            const float dx = static_cast<float>(c) - centerCol;
            const float rowF = centerRow + pitchOffsetRows + dx * std::tan(attitude.rollRad) * CharAspect;

            int row = static_cast<int>(std::floor(rowF));
            float fraction = rowF - static_cast<float>(row);

            if (row < rowMin || row >= rowMax)
                continue;

            uint8_t sub = static_cast<uint8_t>(std::lround((1.0f - fraction) * 9.0f));
            cells[count++] = {static_cast<uint8_t>(row), static_cast<uint8_t>(c + colOffset),
                            static_cast<uint8_t>(HorizonCharCode + sub)};
        }
        GetOsdController().DrawCells(cells);
    }

private:
    static constexpr uint8_t HorizonCharCode = 0xF0;
};
