#pragma once
#include <algorithm>

class PidController
{
public:
    PidController(float kp, float ki, float kd, float outputMin, float outputMax, float integralMin, float integralMax)
        : m_kp(kp)
        , m_ki(ki)
        , m_kd(kd)
        , m_outputMin(outputMin)
        , m_outputMax(outputMax)
        , m_integralMin(integralMin)
        , m_integralMax(integralMax)
    {
    }

    float Update(float error, float dt)
    {
        m_integral += error * dt;
        m_integral = std::clamp(m_integral, m_integralMin, m_integralMax);

        const float derivative = (error - m_prevError) / dt;
        m_prevError = error;

        const float output = m_kp * error
                            + m_ki * m_integral
                            + m_kd * derivative;

        return std::clamp(output, m_outputMin, m_outputMax);
    }

    void Reset()
    {
        m_integral = 0.0f;
        m_prevError = 0.0f;
    }

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_outputMin;
    float m_outputMax;
    float m_integralMin;
    float m_integralMax;

    float m_integral = 0.0f;
    float m_prevError = 0.0f;
};
