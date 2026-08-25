#include "main.h"
#include "AdcSensor.h"
#include "BME280.h"
#include "Config.h"
#include "HalfDuplexSoftwareUart.h"
#include "cmsis_os.h"
#include "CrsfUartWriteNodes.h"
#include <ThrustController.h>
#include "TrampProtocol.h"
#include <SurfaceController.h>
#include "SensorSamplingNode.h"
#include "SensorFusionNode.h"
#include "Mpu6050.h"
#include "Qmc5883L.h"
#include <PubSubLib/Node.hpp>
#include "EmaFilterNode.h"
#include "OsdModule.h"
#include "OsdCompositorTask.h"
#include "OsdHorizonNode.h"
#include "BatterySensorNode.h"
#include "OsdBatteryVoltageNode.h"
#include "PressureToAltitudeConverterNode.h"
#include "OsdAltitudeNode.h"
#include "CrsfLib/FrameParserNode.hpp"
#include "CrsfFrameReaderNode.h"
#include "CrsfFrameProcessorNode.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
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

    std::unique_ptr<BME280> bmeSensor = bus->CreateSensor<BME280>();
    std::unique_ptr<SensorSamplingNode<BME280, BME280::Measurements, 1>> bmeNode = nullptr;
    std::unique_ptr<EmaFilterNode<BME280::Measurements, 1>> bmeFilterNode = nullptr;
    std::unique_ptr<PressureToAltitudeConverterNode<2>> altitudeNode = nullptr;
    std::unique_ptr<OsdAltitudeNode> osdAltitudeNode = nullptr;

    if(bmeSensor->Start() == Status::Ok)
    {
        BME280::Measurements baseMeasurements{};
        bmeSensor->Calibrate(baseMeasurements);
        bmeNode = std::make_unique<SensorSamplingNode<BME280, BME280::Measurements, 1>>(std::move(bmeSensor), "BME");
        bmeFilterNode = std::make_unique<EmaFilterNode<BME280::Measurements, 1>>(0.1f, 5);
        altitudeNode = std::make_unique<PressureToAltitudeConverterNode<2>>(baseMeasurements.pressure, baseMeasurements.temperature);
        bmeNode->Start();
    }

    auto adcSensor = std::make_unique<AdcSensor>(hadc1, ADC_CHANNEL_1);
    auto batterySensor = std::make_unique<BatterySensorNode<Config::BATTERY_MAX_VOLTAGE, Config::BATTERY_ADC_MAX_VOLTAGE>>(std::move(adcSensor), "BatterySensor", Config::BATTERY_SAMPLING_RATE_HZ);

    auto osdReader = std::make_shared<OsdResponseReader>();
    auto osdUart = std::make_unique<Uart>(huart2, osdReader);
    auto osdModule = std::make_unique<OsdModule>(std::move(osdUart), osdReader);
    auto osdController = std::make_shared<OsdController>(std::move(osdModule));
    auto osdHorizonNode = std::make_unique<OsdHorizonNode>(osdController);
    auto osdBatteryInfoNode = std::make_unique<OsdBatteryVoltageNode>(osdController);
    auto osdCompositorTask = std::make_unique<OsdCompositorTask<3>>("OsdRenderer", Config::OSD_REFRESH_RATE_HZ);
    osdCompositorTask->Register(*osdHorizonNode);
    osdCompositorTask->Register(*osdBatteryInfoNode);

    if (altitudeNode)
    {
        osdAltitudeNode = std::make_unique<OsdAltitudeNode>(osdController);
        Connect(*altitudeNode, *osdAltitudeNode);
        osdCompositorTask->Register(*osdAltitudeNode);
    }

    Connect(fusion, *osdHorizonNode);
    Connect(*batterySensor, *osdBatteryInfoNode);

    auto swUart = std::make_unique<HalfDuplexSoftwareUart>(TRAMP_Pin, TRAMP_GPIO_Port, htim10);
    auto trampParser = std::make_unique<TrampFrameParser>("TrampFrameParser");
    auto trampProtocolController = std::make_unique<TrampProtocolController>("TrampProtocol");
    Connect(*swUart, *trampParser);
    Connect(*trampProtocolController, *swUart);

    fusion.Start();
    batterySensor->Start();
    osdCompositorTask->StartCompositor();

    auto crsfFrameReader = std::make_shared<CrsfFrameReaderNode>();
    auto crsfFrameParser = std::make_unique<CrsfLib::FrameParserNode>();
    auto crsfFrameProcessor = std::make_unique<CrsfFrameProcessorNode<2>>();
    auto crsfUart = std::make_shared<Uart>(huart1, crsfFrameReader);
    auto crsfDeviceInfo = std::make_unique<DeviceInfoWriteNode>(crsfUart);

    Connect(*crsfFrameReader, *crsfFrameParser);
    Connect(*crsfFrameParser, *crsfFrameProcessor);
    Connect(crsfFrameProcessor->GetDeviceInfoNode(), *crsfDeviceInfo);
    crsfFrameReader->Start();


    while (true)
    {
        osDelay(1);
    }
}


