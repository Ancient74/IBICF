#include "main.h"
#include "cmsis_os.h"
#include "Mpu6050.h"
#include "Qmc5883L.h"

extern I2C_HandleTypeDef hi2c2;

extern "C" void main_cpp()
{
    auto bus = std::make_unique<I2cBus>(hi2c2);
    auto mpuSensor = bus->CreateSensor<Mpu6050>();
    mpuSensor->Start();

    auto qmcSensor = bus->CreateSensor<Qmc5883L>();
    qmcSensor->Start();

    while (true)
    {
        osDelay(1);
    }
}


