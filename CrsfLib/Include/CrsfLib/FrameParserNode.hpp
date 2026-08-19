#pragma once
#include "CrsfLib/Frame.hpp"
#include <array>
#include <functional>
#include "PubSubLib/Node.hpp"

namespace CrsfLib
{
    class FrameParserNode
        : public NodeInput<FrameParserNode, uint8_t>
        , public NodeOutput<FrameParserNode, FrameView, 1>
    {
    public:
        FrameParserNode() = default;
        void Process(uint8_t byte);

    private:
        enum class Phase { Sync, Length, Body } m_phase = Phase::Sync;
        std::array<uint8_t, MAX_FRAME_SIZE> m_buffer{};
        size_t m_index = 0;
        size_t m_total = 0;
    };
}
