#include "CrsfLib/Frame.hpp"
#include "CrsfLib/Crc.hpp"

using namespace CrsfLib;

RawFrame::RawFrame(std::span<const uint8_t> data)
    : m_data(data)
{}

Address RawFrame::SyncByte() const
{
    return static_cast<Address>(m_data[0]);
}

uint8_t RawFrame::Length() const
{
    return m_data[1];
}

FrameType RawFrame::Type() const
{
    return static_cast<FrameType>(m_data[2]);
}

bool RawFrame::IsExtended() const
{
    return IsExtendedHeader(Type());
}

bool RawFrame::CheckCrc() const
{
    return  m_data[1 + Length()]
        == CalculateCrc({ m_data.data() + 2, static_cast<size_t>(Length() - 1) });
}

FrameView RawFrame::GetFrameView() const
{
    FrameView view{};
    view.type = Type();

    const auto length = Length();

    if (IsExtended())
    {
        view.destination = static_cast<Address>(m_data[3]);
        view.origin = static_cast<Address>(m_data[4]);
        view.payload = { m_data.data() + 5, static_cast<size_t>(length - 4) };
    }
    else
    {
        view.payload = { m_data.data() + 3, static_cast<size_t>(length - 2) };
    }

    return view;
}