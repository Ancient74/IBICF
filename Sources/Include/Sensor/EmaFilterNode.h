#pragma once
#include "SensorConcepts.h"

template<SampleValue O, std::size_t MaxSubs>
class EmaFilterNode : public NodeInput<EmaFilterNode<O, MaxSubs>, O>,
                      public NodeOutput<EmaFilterNode<O, MaxSubs>, O, MaxSubs>
{
public:
    using InputType = O;
    using OutputType = O;

    EmaFilterNode(float emaAlpha, uint8_t oversamplingRatio)
        : m_emaAlpha(emaAlpha), m_oversamplingRatio(oversamplingRatio == 0 ? 1 : oversamplingRatio) {}

    void Process(const O& raw)
    {
        m_sum = m_sum + raw;
        if (++m_count < m_oversamplingRatio) return;

        const O oversampled = m_sum * (1.0f / m_oversamplingRatio);
        m_accumulator = oversampled * m_emaAlpha + m_accumulator * (1.0f - m_emaAlpha);
        m_sum = O{}; m_count = 0;

        this->Publish(m_accumulator);
    }

private:
    float m_emaAlpha;
    uint8_t m_oversamplingRatio;
    uint8_t m_count = 0;
    O m_sum{};
    O m_accumulator{};
};
