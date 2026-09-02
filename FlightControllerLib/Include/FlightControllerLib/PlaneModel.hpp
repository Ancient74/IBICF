#pragma once
#include <stdint.h>

class ISurfaceController
{
public:
    virtual ~ISurfaceController() = default;
    virtual void SetSurface(int16_t deviation) = 0;
};

class IThrustController
{
public:
    virtual ~IThrustController() = default;
    virtual void SetThrust(uint16_t value) = 0;
};

class PlaneModel
{
public:
    PlaneModel(
          ISurfaceController& leftAileron
        , ISurfaceController& rightAileron
        , ISurfaceController& leftRuddervator
        , ISurfaceController& rightRuddervator
        , IThrustController& thrust);

    void Roll(int16_t deviation);
    void Pitch(int16_t deviation);
    void Yaw(int16_t deviation);

    void Thrust(uint16_t thrust);

private:
    void UpdateRuddervators();

    ISurfaceController& m_leftAileron;
    ISurfaceController& m_rightAileron;
    ISurfaceController& m_leftRuddervator;
    ISurfaceController& m_rightRuddervator;
    IThrustController& m_thrust;
    int16_t m_pitchCommand = 0;
    int16_t m_yawCommand   = 0;
};
