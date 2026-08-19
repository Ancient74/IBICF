#pragma once
#include <stdint.h>
#include <array>
#include "KalmanLib/Quaternion.hpp"

template<uint8_t MaxCols = 30>
class HorizonRenderer
{
public:
    struct Cell { uint8_t row; uint8_t col; uint8_t charCode; };
    using CellList = std::array<Cell, MaxCols>;

    HorizonRenderer(uint8_t rows, uint8_t cols) : m_rows(rows), m_cols(cols) {}

    uint8_t Compute(const Attitude& att, CellList& out) const
    {
        const float centerRow = m_rows / 2.0f;
        const float centerCol = m_cols / 2.0f;
        const float pitchOffsetRows = (att.pitchRad / (static_cast<float>(M_PI) / 2.0f)) * centerRow;

        uint8_t count = 0;
        for (uint8_t c = 0; c < m_cols; ++c)
        {
            const float dx = static_cast<float>(c) - centerCol;
            const float rowF = centerRow + pitchOffsetRows + dx * std::tan(att.rollRad);
            const int row = static_cast<int>(std::lround(rowF));
            if (row < 0 || row >= m_rows) continue;
            out[count++] = {static_cast<uint8_t>(row), c, HorizonCharCode};
        }
        return count;
    }

private:
    uint8_t m_rows, m_cols;
    static constexpr uint8_t HorizonCharCode = 0x8F;
};
