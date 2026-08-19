#pragma once
#include <concepts>
#include <array>
#include <cassert>

template<typename T>
concept MessageType = std::is_trivially_copyable_v<T>;

template<MessageType T>
inline constexpr bool PassByValue = sizeof(T) <= 3 * sizeof(void*);

template<MessageType T>
using Param = std::conditional_t<PassByValue<T>, T, const T&>;

template<MessageType Message, uint8_t MaxSubs>
class Topic
{
public:
    using Arg = Param<Message>;
    using Callback = void(*)(void*, Arg);

    void Subscribe(void* context, Callback callback)
    {
        assert(m_count < MaxSubs);
        m_slots[m_count++] = {context, callback};
    }

    void Publish(Arg message)
    {
        for(uint8_t i = 0; i < m_count; i++)
            m_slots[i].callback(m_slots[i].context, message);
    }

private:
    struct Slot
    {
        void* context;
        Callback callback;
    };
    std::array<Slot, MaxSubs> m_slots;
    uint8_t m_count = 0;
};
