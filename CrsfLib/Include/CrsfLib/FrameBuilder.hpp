#pragma once
#include "Frame.hpp"
#include <string>

namespace CrsfLib
{
    class FrameBuilder {
    public:
        explicit FrameBuilder(FrameType type, Address sync = Address(0xC8));

        FrameBuilder& Addresses(Address destination, Address origin);

        FrameBuilder& U8(uint8_t value);
        FrameBuilder& U16(uint16_t value);
        FrameBuilder& U32(uint32_t value);
        FrameBuilder& U64(uint64_t value);
        FrameBuilder& Text(std::string_view text);

        std::span<const uint8_t> Finalize();

    private:
        std::array<uint8_t, MAX_FRAME_SIZE> m_buffer{};
        size_t m_index = 0;
        bool m_isExtended = false;
    };

    struct any_t {
        template<typename T> constexpr operator T() const;
    };

    template<typename T, typename... A>
    consteval std::size_t MemberCount() {
        if constexpr (requires { T{ A{}..., any_t{} }; })
            return MemberCount<T, A..., any_t>();
        else
            return sizeof...(A);
    }

    template<typename T>
    constexpr auto ToTuple(T&& obj) {
        constexpr auto N = MemberCount<std::remove_cvref_t<T>>();
        if      constexpr (N == 1) { auto& [a]           = obj; return std::tie(a); }
        else if constexpr (N == 2) { auto& [a,b]         = obj; return std::tie(a,b); }
        else if constexpr (N == 3) { auto& [a,b,c]       = obj; return std::tie(a,b,c); }
        else if constexpr (N == 4) { auto& [a,b,c,d]     = obj; return std::tie(a,b,c,d); }
        else if constexpr (N == 5) { auto& [a,b,c,d,e]   = obj; return std::tie(a,b,c,d,e); }
        else if constexpr (N == 6) { auto& [a,b,c,d,e,f] = obj; return std::tie(a,b,c,d,e,f); }

        else static_assert(N <= 6, "extend ToTuple");
    }

    template<typename> inline constexpr bool always_false_v = false;
    template<typename F>
    void WriteField(FrameBuilder& b, const F& v) {
        using T = std::remove_cvref_t<F>;
        if      constexpr (std::is_same_v<T, uint8_t>)         b.U8(v);
        else if constexpr (std::is_same_v<T, uint16_t>)        b.U16(v);
        else if constexpr (std::is_same_v<T, uint32_t>)        b.U32(v);
        else if constexpr (std::is_same_v<T, uint64_t>)        b.U64(v);
        else if constexpr (std::is_same_v<T, const char*> ||
                           std::is_same_v<T, char*>       ||
                           std::is_same_v<T, std::string_view>) b.Text(v);
        else static_assert(always_false_v<T>, "Unsupported field type");
    }

    template<typename T>
    concept FrameT = requires {
        { T::type } -> std::convertible_to<FrameType>;
        requires std::is_same_v<std::remove_cv_t<decltype(T::type)>, FrameType>;
    };

    template<FrameT T, Address address = Address::FlightController>
    std::span<const uint8_t> BuildFrame(const T& obj)
    {
        FrameBuilder builder(T::type, address);
        std::apply([&](const auto&... fs) { (WriteField(builder, fs), ...); },
                ToTuple(obj));
        return builder.Finalize();
    }

    template<FrameT T, Address address = Address::FlightController>
    std::span<const uint8_t> BuildFrame(Address destination, Address origin, const T& obj)
    {
        static_assert(!IsExtendedHeader(T::type), "This override must be called for extended headers");

        FrameBuilder builder(T::type, address);
        builder.Addresses(destination, origin);
        std::apply([&](const auto&... fs) { (WriteField(builder, fs), ...); },
                ToTuple(obj));
        return builder.Finalize();
    }
}
