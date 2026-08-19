#pragma once
#include "Topic.hpp"

template<typename Derived, MessageType Out, uint8_t MaxSubs>
class NodeOutput
{
public:
    using OutputType = Out;
    using Arg = Param<OutputType>;
    Topic<Out, MaxSubs>& GetTopic() { return m_topic; }

protected:
    void Publish(Arg msg) { m_topic.Publish(msg); }

private:
    Topic<Out, MaxSubs> m_topic;
};

template<typename Derived, MessageType In>
class NodeInput
{
public:
    using InputType = In;
    using Arg = Param<InputType>;

    void OnMessage(Arg msg) { static_cast<Derived*>(this)->Process(msg); }
};

template<typename PublisherT, typename SubscriberT>
    requires std::same_as<typename PublisherT::OutputType, typename SubscriberT::InputType>
void Connect(PublisherT& publisher, SubscriberT& subscriber)
{
    using Arg = Param<typename SubscriberT::InputType>;
    publisher.GetTopic().Subscribe(&subscriber, [](void* context, Arg message)
    {
        static_cast<SubscriberT*>(context)->OnMessage(message);
    });
}
