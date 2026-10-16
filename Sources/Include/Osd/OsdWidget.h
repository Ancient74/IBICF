#pragma once
#include "OsdSymbols.h"
#include <concepts>
#include <cstddef>
#include <utility>
#include <array>

template <std::size_t Rows, std::size_t Cols>
using OsdChars = std::array<std::array<OsdChar, Cols>, Rows>;

template <typename Derived, typename InputData, std::size_t Rows, std::size_t Cols>
class OsdWidget
{
    static_assert(Rows > 0);
    static_assert(Cols > 0);

public:
    using Content = OsdChars<Rows, Cols>;

    explicit OsdWidget(InputData initialData)
        : m_data(std::move(initialData))
    {}

    void Set(InputData data)
    {
        m_data = std::move(data);
        m_inputStale = true;
    }

    const Content& Render() noexcept
    {
        Refresh();
        return m_content;
    }

private:
    void Refresh()
    {
        static_assert(requires(Derived& widget, const InputData& data) {
            { widget.RenderContent(data) } -> std::same_as<Content>;
        });

        if (!m_inputStale)
            return;
        m_inputStale = false;

        m_content = static_cast<Derived&>(*this).RenderContent(m_data);
    }

    InputData m_data;
    Content m_content{};
    bool m_inputStale = true;
};
