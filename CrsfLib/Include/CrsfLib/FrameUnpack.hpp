#pragma once
#include "CrsfLib/Frame.hpp"
#include <cstring> // memcpy for some reason...

namespace CrsfLib
{
    template <typename T>
    T Unpack(std::span<const uint8_t> payload);

    template <>
    inline RcChannelsPackedFrame Unpack<RcChannelsPackedFrame>(std::span<const uint8_t> payload)
    {
        RcChannelsPackedFrame frame{};
        uint32_t bits = 0;
        size_t bitCount = 0;
        size_t byteIndex  = 0;

        for (size_t channel = 0; channel < frame.m_channels.size(); channel++)
        {
            while (bitCount < 11)
            {
                bits |= static_cast<uint32_t>(payload[byteIndex++]) << bitCount;
                bitCount += 8;
            }
            frame.m_channels[channel] = static_cast<uint16_t>(bits & 0x07FF);
            bits >>= 11;
            bitCount -= 11;
        }
        return frame;
    }

    template <>
    inline LinkStatisticsFrame Unpack<LinkStatisticsFrame>(std::span<const uint8_t> payload)
    {
        LinkStatisticsFrame frame{};
        const auto n = std::min(payload.size(), sizeof(frame));
        std::memcpy(&frame, payload.data(), n);
        return frame;
    }
}
