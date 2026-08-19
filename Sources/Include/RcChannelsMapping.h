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

