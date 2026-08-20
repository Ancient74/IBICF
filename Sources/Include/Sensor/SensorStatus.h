#pragma once
#include "stm32f4xx_hal.h"

enum class Status
{
    Ok,
    Error,
    Busy,
    Timeout
};

inline Status FromHAL(HAL_StatusTypeDef halStatus)
{
    switch (halStatus)
    {
    case HAL_StatusTypeDef::HAL_OK:
        return Status::Ok;
    case HAL_StatusTypeDef::HAL_BUSY:
        return Status::Busy;
    case HAL_StatusTypeDef::HAL_ERROR:
        return Status::Error;
    case HAL_StatusTypeDef::HAL_TIMEOUT:
        return Status::Timeout;

    default:
        return Status::Error;
    }
}
