#pragma once
#include <stdint.h>
#include <span>

namespace CrsfLib
{
    uint8_t CalculateCrc(std::span<const uint8_t> data);
}
