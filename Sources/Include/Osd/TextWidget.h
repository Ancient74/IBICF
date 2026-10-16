#pragma once
#include "Config.h"
#include "OsdWidget.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <string_view>

struct LeftAligned {};
struct CenterAligned {};
struct RightAligned {};

struct OsdTextData
{
    std::array<char, Config::OSD_MAX_TEXT_SIZE> text{};
    std::size_t length = 0;
};

template <typename Alignment = LeftAligned>
class TextWidget : public OsdWidget<TextWidget<Alignment>, OsdTextData, 1, Config::OSD_MAX_TEXT_SIZE>
{
    using Base = OsdWidget<TextWidget<Alignment>, OsdTextData, 1, Config::OSD_MAX_TEXT_SIZE>;
    static_assert(std::same_as<Alignment, LeftAligned> ||
                  std::same_as<Alignment, CenterAligned> ||
                  std::same_as<Alignment, RightAligned>,
                  "TextWidget alignment must be LeftAligned, CenterAligned, or RightAligned");

public:
    using Content = typename Base::Content;

    TextWidget()
        : Base({})
    {
    }

    explicit TextWidget(std::string_view text)
        : Base(MakeTextData(text))
    {
    }

    void Set(std::string_view text)
    {
        Base::Set(MakeTextData(text));
    }

    Content RenderContent(const OsdTextData& data) const
    {
        Content content{};
        content[0].fill(OsdChar::Space);

        const std::size_t length = std::min(data.length, data.text.size());
        std::size_t start = 0;
        if constexpr (std::same_as<Alignment, CenterAligned>)
            start = (data.text.size() - length) / 2;
        else if constexpr (std::same_as<Alignment, RightAligned>)
            start = data.text.size() - length;

        for (std::size_t i = 0; i < length; ++i)
            content[0][start + i] = OsdCharFromChar(data.text[i]);

        return content;
    }

private:
    static OsdTextData MakeTextData(std::string_view text)
    {
        OsdTextData data{};
        data.length = std::min(text.size(), data.text.size());
        std::copy_n(text.begin(), data.length, data.text.begin());
        return data;
    }
};
