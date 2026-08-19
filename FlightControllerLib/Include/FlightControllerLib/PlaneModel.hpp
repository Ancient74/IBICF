#pragma once
#include <stdint.h>
#include <memory>

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
          std::unique_ptr<ISurfaceController> leftAileron
        , std::unique_ptr<ISurfaceController> rightAileron
        , std::unique_ptr<ISurfaceController> leftRuddervator
        , std::unique_ptr<ISurfaceController> rightRuddervator
        , std::unique_ptr<IThrustController> thrust);

    void Roll(int16_t deviation);
    void Pitch(int16_t deviation);
    void Yaw(int16_t deviation);

    void Thrust(uint16_t thrust);

private:
    void UpdateRuddervators();

    std::unique_ptr<ISurfaceController> m_leftAileron;
    std::unique_ptr<ISurfaceController> m_rightAileron;
    std::unique_ptr<ISurfaceController> m_leftRuddervator;
    std::unique_ptr<ISurfaceController> m_rightRuddervator;
    std::unique_ptr<IThrustController> m_thrust;
    int16_t m_pitchCommand = 0;
    int16_t m_yawCommand   = 0;
};
