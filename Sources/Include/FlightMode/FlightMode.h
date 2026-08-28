#pragma once
#include <cstdint>

struct PitchRollYawThrust
{
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
    int32_t thrust;
};

class IFlightMode
{
public:
    virtual ~IFlightMode() = default;
    virtual PitchRollYawThrust Translate(const PitchRollYawThrust& command) = 0;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual const char* GetName() const = 0;
};

class ManualFlightMode : public IFlightMode
{
public:
    PitchRollYawThrust Translate(const PitchRollYawThrust& command) override;
    void OnEnter() override {};
    void OnExit() override {};
    const char* GetName() const override;
};
