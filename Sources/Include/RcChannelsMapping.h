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

template<typename Source, uint16_t Boundary = 992, uint16_t Margin = 150, uint8_t ConfirmFrames = 5>
class DoubleSwitch
{
public:
    bool Update(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        auto value = Source::Read(rc);

        bool candidate = m_state;
        bool hasCandidate = false;

        if (value > Boundary + Margin)
        {
            candidate = true;
            hasCandidate = true;
        }
        else if (value < Boundary - Margin)
        {
            candidate = false;
            hasCandidate = true;
        }

        if (!hasCandidate || candidate == m_state)
        {
            m_confirmCount = 0;
            return m_state;
        }

        if (candidate == m_pendingCandidate)
        {
            ++m_confirmCount;
        }
        else
        {
            m_pendingCandidate = candidate;
            m_confirmCount = 1;
        }

        if (m_confirmCount >= ConfirmFrames)
        {
            m_state = candidate;
            m_confirmCount = 0;
        }

        return m_state;
    }

    bool Get() const { return m_state; }

private:
    bool m_state = false;
    bool m_pendingCandidate = false;
    uint8_t m_confirmCount = 0;
};

enum class TriSwitchPosition : uint8_t { Low, Mid, High };
template<typename Source,
         uint16_t LowMidBoundary = 682,
         uint16_t MidHighBoundary = 1301,
         uint16_t Margin = 50,
         uint8_t ConfirmFrames = 5>
class TriSwitch
{
public:

    TriSwitchPosition Update(const CrsfLib::RcChannelsPackedFrame& rc)
    {
        auto value = Source::Read(rc);

        TriSwitchPosition candidate = m_state;
        bool hasCandidate = false;

        switch (m_state)
        {
        case TriSwitchPosition::Low:
            if (value > LowMidBoundary + Margin)
            {
                candidate = TriSwitchPosition::Mid;
                hasCandidate = true;
            }
            break;

        case TriSwitchPosition::Mid:
            if (value < LowMidBoundary - Margin)
            {
                candidate = TriSwitchPosition::Low;
                hasCandidate = true;
            }
            else if (value > MidHighBoundary + Margin)
            {
                candidate = TriSwitchPosition::High;
                hasCandidate = true;
            }
            break;

        case TriSwitchPosition::High:
            if (value < MidHighBoundary - Margin)
            {
                candidate = TriSwitchPosition::Mid;
                hasCandidate = true;
            }
            break;
        }

        if (!hasCandidate)
        {
            m_confirmCount = 0;
            return m_state;
        }

        if (candidate == m_pendingCandidate)
        {
            ++m_confirmCount;
        }
        else
        {
            m_pendingCandidate = candidate;
            m_confirmCount = 1;
        }

        if (m_confirmCount >= ConfirmFrames)
        {
            m_state = candidate;
            m_confirmCount = 0;
        }

        return m_state;
    }

    TriSwitchPosition Get() const { return m_state; }

private:
    TriSwitchPosition m_state = TriSwitchPosition::Low;
    TriSwitchPosition m_pendingCandidate = TriSwitchPosition::Low;
    uint8_t m_confirmCount = 0;
};
