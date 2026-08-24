#pragma once
#include <stdint.h>
#include "CrsfLib/Frame.hpp"

template<size_t Index>
struct Channel
{
    static uint16_t Read(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        return rc.m_channels[Index];
    }
};

template<typename Source,
         uint16_t InputMin,
         uint16_t InputMax,
         int32_t OutputMin,
         int32_t OutputMax>
struct Scaled
{
    static int32_t Get(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        auto value = Source::Read(rc);

        return OutputMin +
            ((static_cast<int32_t>(value) - InputMin) *
             (OutputMax - OutputMin))
            / (InputMax - InputMin);
    }
};

template<typename Source, uint16_t Boundary = 992, uint16_t Margin = 150>
class DoubleSwitch
{
public:
    bool Update(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        auto value = Source::Read(rc);

        if (value > Boundary + Margin)
        {
            m_state = true;
        }
        else if (value < Boundary - Margin)
        {
            m_state = false;
        }

        return m_state;
    }

    bool Get() const { return m_state; }

private:
    bool m_state = false;
};

template<typename Source,
         uint16_t LowMidBoundary = 682,
         uint16_t MidHighBoundary = 1301,
         uint16_t Margin = 50>
class TriSwitch
{
public:
    enum class Position : uint8_t { Low, Mid, High };

    Position Update(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        auto value = Source::Read(rc);

        switch (m_state)
        {
        case Position::Low:
            if (value > LowMidBoundary + Margin)
            {
                m_state = Position::Mid;
            }
            break;

        case Position::Mid:
            if (value < LowMidBoundary - Margin)
            {
                m_state = Position::Low;
            }
            else if (value > MidHighBoundary + Margin)
            {
                m_state = Position::High;
            }
            break;

        case Position::High:
            if (value < MidHighBoundary - Margin)
            {
                m_state = Position::Mid;
            }
            break;
        }

        return m_state;
    }

    Position Get() const { return m_state; }

private:
    Position m_state = Position::Low;
};
