#include "main.h"
#include "cmsis_os.h"
#include "SensorSamplingNode.h"
#include "SensorFusionNode.h"
#include "Mpu6050.h"
#include "Qmc5883L.h"
#include <PubSubLib/Node.hpp>
#include "EmaFilterNode.h"

extern I2C_HandleTypeDef hi2c2;

extern "C" void main_cpp()
{
    auto bus = std::make_unique<I2cBus>(hi2c2);
    auto mpuSensor = bus->CreateSensor<Mpu6050>();
    mpuSensor->Start();

    Matrix3f accelR{};
    Matrix3f magnetR{};
    Matrix3f N{};
    auto samplingRate = mpuSensor->GetSamplingRate();

    mpuSensor->Calibrate(accelR, N);
    auto mpuNode = std::make_unique<SensorSamplingNode<Mpu6050, GyroscopeAccelerometer, 1>>(std::move(mpuSensor), "MPU");
    auto mpuFilterNode = std::make_unique<EmaFilterNode<GyroscopeAccelerometer, 1>>(0.1f, 5);
    SensorFusionNode<1> fusion{1.0f / samplingRate, accelR, magnetR, N};

    Connect(*mpuNode, *mpuFilterNode);
    Connect(*mpuFilterNode, fusion.ImuInput());

    std::unique_ptr<Qmc5883L> qmcSensor = nullptr;
    std::unique_ptr<SensorSamplingNode<Qmc5883L, Magnetometer, 1>> qmcNode = nullptr;
    std::unique_ptr<EmaFilterNode<Magnetometer, 1>> qmcFilterNode = nullptr;

    if (Config::ENABLE_MAGNETOMETER_CORRECTION)
    {
        qmcSensor = bus->CreateSensor<Qmc5883L>();
        qmcNode = std::make_unique<SensorSamplingNode<Qmc5883L, Magnetometer, 1>>(std::move(qmcSensor), "QMC");
        qmcSensor->Start();
        qmcSensor->GetSensorVariance(magnetR);
        qmcFilterNode = std::make_unique<EmaFilterNode<Magnetometer, 1>>(0.1f, 1);

        Connect(*qmcNode, *qmcFilterNode);
        Connect(*qmcFilterNode, fusion.MagInput());
        qmcNode->Start();
    }
    mpuNode->Start();

    fusion.Start();

    while (true)
    {
        osDelay(1);
    }
}


