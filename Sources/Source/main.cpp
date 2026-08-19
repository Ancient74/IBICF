#include "main.h"
#include "cmsis_os.h"
#include "Mpu6050.h"

extern I2C_HandleTypeDef hi2c2;

extern "C" void main_cpp()
{
    auto bus = std::make_unique<I2cBus>(hi2c2);
    auto mpuSensor = bus->CreateSensor<Mpu6050>();
    mpuSensor->Start();

    while (true)
    {
        osDelay(1);
    }
}


