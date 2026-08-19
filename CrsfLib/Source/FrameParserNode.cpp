#include "CrsfLib/FrameParserNode.hpp"

using namespace CrsfLib;

void FrameParserNode::Process(uint8_t byte)
{
    switch (m_phase)
    {
    case Phase::Sync:
        if (!IsValidAddress(byte))
            return;
        m_buffer[0] = byte;
        m_index = 1;
        m_phase = Phase::Length;
        return;

    case Phase::Length:
        if (byte < 2 || byte > 62)
        {
            m_phase = Phase::Sync;
            return;
        }
        m_buffer[1] = byte;
        m_total = byte + 2;                      // sync + len + (len) bytes
        m_index = 2;
        m_phase = Phase::Body;
        return;

    case Phase::Body:
        m_buffer[m_index++] = byte;
        if (m_index == m_total)
        {
            RawFrame frame({m_buffer.data(), m_total});
            if (frame.CheckCrc())
                Publish(frame.GetFrameView());
            m_phase = Phase::Sync;
        }
        return;
    }
}
