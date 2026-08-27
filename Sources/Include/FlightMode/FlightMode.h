#pragma once
#include <cstdint>

struct PitchRollYaw
{
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
};

class IFlightMode
{
public:
    virtual ~IFlightMode() = default;
    virtual PitchRollYaw Translate(const PitchRollYaw& command) = 0;
    virtual void OnEnter() = 0;
    virtual void OnExit() = 0;
    virtual const char* GetName() const = 0;
};

class ManualFlightMode : public IFlightMode
{
public:
    PitchRollYaw Translate(const PitchRollYaw& command) override;
    void OnEnter() override {};
    void OnExit() override {};
    const char* GetName() const override;
};
