#pragma once
#include <PubSubLib/Node.hpp>
#include <KalmanLib/Quaternion.hpp>

template<std::size_t MaxSubs>
class RotationTransformNode
    : public NodeInput<RotationTransformNode<MaxSubs>, Quaternion<float>>
    , public NodeOutput<RotationTransformNode<MaxSubs>, Quaternion<float>, MaxSubs>
{
public:
    RotationTransformNode(const Quaternion<float>& worldFixRotation, const Quaternion<float>& rotationToBody)
        : m_worldFixRotation(worldFixRotation)
        , m_rotationToBody(rotationToBody)
    {
    }

    void Process(const Quaternion<float>& input)
    {
        Quaternion<float> transformed = m_worldFixRotation * input * m_rotationToBody;
        this->Publish(transformed);
    }

private:
    Quaternion<float> m_worldFixRotation;
    Quaternion<float> m_rotationToBody;
};
