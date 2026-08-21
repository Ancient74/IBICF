#include "main.h"
#include "AdcSensor.h"
#include "Config.h"
#include "cmsis_os.h"
#include "CrsfLink.h"
#include <ThrustController.h>
#include <SurfaceController.h>
#include "SensorSamplingNode.h"
#include "SensorFusionNode.h"
#include "Mpu6050.h"
#include "Qmc5883L.h"
#include <PubSubLib/Node.hpp>
#include "EmaFilterNode.h"
#include "OsdModule.h"
#include "OsdHorizonNode.h"
#include "BatterySensorNode.h"
#include "OsdBatteryVoltageNode.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c2;

using namespace std::chrono_literals;

extern "C" void main_cpp()
{
    auto leftAileronControl = std::make_unique<ServoPwmControl>(htim4, TIM_CHANNEL_1, 500us, 2500us);
    auto rightAileronControl = std::make_unique<ServoPwmControl>(htim4, TIM_CHANNEL_2, 500us, 2500us);
    auto leftRuddervatorControl = std::make_unique<ServoPwmControl>(htim4, TIM_CHANNEL_3, 1000us, 2000us);
    auto rightRuddervatorControl = std::make_unique<ServoPwmControl>(htim4, TIM_CHANNEL_4, 1000us, 2000us);

    auto thrust = std::make_unique<ThrustController>(htim3, TIM_CHANNEL_1);

    auto leftAileron = std::make_unique<SurfaceController>(std::move(leftAileronControl));
    auto rightAileron = std::make_unique<SurfaceController>(std::move(rightAileronControl));
    auto leftRuddervator = std::make_unique<SurfaceController>(std::move(leftRuddervatorControl));
    auto rightRuddervator = std::make_unique<SurfaceController>(std::move(rightRuddervatorControl));
    auto model = std::make_shared<PlaneModel>(
        std::move(leftAileron),
        std::move(rightAileron),
        std::move(leftRuddervator),
        std::move(rightRuddervator),
        std::move(thrust)
    );

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

    auto adcSensor = std::make_unique<AdcSensor>(hadc1, ADC_CHANNEL_1);
    auto batterySensor = std::make_unique<BatterySensorNode<Config::BATTERY_MAX_VOLTAGE, Config::BATTERY_ADC_MAX_VOLTAGE>>(std::move(adcSensor), "BatterySensor", Config::BATTERY_SAMPLING_RATE_HZ);

    auto osdReader = std::make_shared<OsdResponseReader>();
    auto osdUart = std::make_unique<Uart>(huart2, osdReader);
    auto osdModule = std::make_unique<OsdModule>(std::move(osdUart), osdReader);
    auto osdController = std::make_shared<OsdController>(std::move(osdModule));
    auto osdHorizonNode = std::make_unique<OsdHorizonNode>(osdController, "OsdHorizon", Config::OSD_HORIZON_LINE_UPDATE_RATE_HZ);
    auto osdBatteryInfoNode = std::make_unique<OsdBatteryVoltageNode>(osdController, "BatteryVoltage", Config::BATTERY_SAMPLING_RATE_HZ);
    Connect(fusion, *osdHorizonNode);
    Connect(*batterySensor, *osdBatteryInfoNode);
    fusion.Start();
    batterySensor->Start();

    CrsfLink link(huart1, model);
    while (true)
    {
        osDelay(1);
    }
}


