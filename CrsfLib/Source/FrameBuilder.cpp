#include "CrsfLib/FrameBuilder.hpp"
#include "CrsfLib/Crc.hpp"

namespace CrsfLib
{
    FrameBuilder::FrameBuilder(FrameType type, Address sync)
    {
        m_buffer[0] = static_cast<uint8_t>(sync);
        m_buffer[2] = static_cast<uint8_t>(type);

        m_isExtended = IsExtendedHeader(type);

        m_index = 3; // Skip length byte for now;
    }

    FrameBuilder& FrameBuilder::Addresses(Address destination, Address origin)
    {
        // call only after Type() if extended
        m_buffer[3] = static_cast<uint8_t>(destination);
        m_buffer[4] = static_cast<uint8_t>(origin);
        m_index = 5;
        return *this;
    }

    FrameBuilder& FrameBuilder::U8(uint8_t value)   { m_buffer[m_index++] = value; return *this; }
    FrameBuilder& FrameBuilder::U16(uint16_t value) { return U8(value >> 8).U8(value & 0xFF); }
    FrameBuilder& FrameBuilder::U32(uint32_t value)  { return U8(value >> 24).U8(value >> 16).U8(value >> 8).U8(value & 0xFF); }
    FrameBuilder& FrameBuilder::U64(uint64_t value)  { return U8(value >> 32).U8(value >> 24).U8(value >> 16).U8(value >> 8).U8(value & 0xFF); }

    FrameBuilder& FrameBuilder::Text(std::string_view text)
    {
        for (const auto c : text)
            U8(static_cast<uint8_t>(c));
        return *this;
    }

    std::span<const uint8_t> FrameBuilder::Finalize()
    {
        const uint8_t length = static_cast<uint8_t>(m_index - 1);  // bytes after Length, +1 for CRC
        m_buffer[1] = length;
        m_buffer[m_index++] = CalculateCrc({ m_buffer.data() + 2, static_cast<size_t>(length - 1) });
        return { m_buffer.data(), m_index };
    }
}
