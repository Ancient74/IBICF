#include "main.h"
#include "AdcSensor.h"
#include "BME280.h"
#include "Config.h"
#include "FailsafeFlightMode.h"
#include "HalfDuplexSoftwareUart.h"
#include "cmsis_os.h"
#include "CrsfUartWriteNodes.h"
#include "CruiseFlightMode.h"
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
#include "PlaneControllerNode.h"
#include "OsdAltitudeNode.h"
#include "OsdFlightModeNode.h"
#include "CrsfLib/FrameParserNode.hpp"
#include "CrsfFrameReaderNode.h"
#include "CrsfFrameProcessorNode.h"
#include <OsdVtxInfo.h>
#include "VtxControllerNode.h"
#include <OsdRcLinkNode.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim10;
extern I2C_HandleTypeDef hi2c2;

using namespace std::chrono_literals;

extern "C" void main_cpp()
{
    ServoPwmControl leftAileronControl{htim4, TIM_CHANNEL_1, 500us, 2500us};
    ServoPwmControl rightAileronControl{htim4, TIM_CHANNEL_2, 500us, 2500us};
    ServoPwmControl leftRuddervatorControl{htim4, TIM_CHANNEL_3, 500us, 2500us};
    ServoPwmControl rightRuddervatorControl{htim4, TIM_CHANNEL_4, 500us, 2500us};

    ThrustController thrust{htim3, TIM_CHANNEL_1};

    SurfaceController leftAileron{leftAileronControl};
    SurfaceController rightAileron{rightAileronControl};
    SurfaceController leftRuddervator{leftRuddervatorControl};
    SurfaceController rightRuddervator{rightRuddervatorControl};
    PlaneModel model{
        leftAileron,
        rightAileron,
        leftRuddervator,
        rightRuddervator,
        thrust
    };

    I2cBus bus{hi2c2};
    Mpu6050 mpuSensor{hi2c2, bus.Mutex()};
    mpuSensor.Start();

    Matrix3f accelR{};
    Matrix3f magnetR{};
    Matrix3f N{};
    auto samplingRate = mpuSensor.GetSamplingRate();

    mpuSensor.Calibrate(accelR, N);
    SensorSamplingNode<Mpu6050, GyroscopeAccelerometer, 1> mpuNode{mpuSensor, "MPU"};
    EmaFilterNode<GyroscopeAccelerometer, 1> mpuFilterNode{0.1f, 5};


    Qmc5883L qmcSensor{hi2c2, bus.Mutex()};
    SensorSamplingNode<Qmc5883L, Magnetometer, 1> qmcNode{qmcSensor, "QMC"};
    EmaFilterNode<Magnetometer, 1> qmcFilterNode{0.1f, 1};
    if (Config::ENABLE_MAGNETOMETER_CORRECTION)
    {
        qmcSensor.Start();
        qmcSensor.GetSensorVariance(magnetR);
    }
    SensorFusionNode<2> fusion{"Fusion", 1.0f / samplingRate, accelR, magnetR, N};

    if (Config::ENABLE_MAGNETOMETER_CORRECTION)
    {
        Connect(qmcNode, qmcFilterNode);
        Connect(qmcFilterNode, fusion.MagInput());
        qmcNode.Start();
    }

    Connect(mpuNode, mpuFilterNode);
    Connect(mpuFilterNode, fusion.ImuInput());
    mpuNode.Start();

    BME280 bmeSensor{hi2c2, bus.Mutex()};

    SensorSamplingNode<BME280, BME280::Measurements, 1> bmeNode{bmeSensor, "BME"};
    EmaFilterNode<BME280::Measurements, 1> bmeFilterNode{0.1f, 5};
    PressureToAltitudeConverterNode<2> altitudeNode;
    bool useBme = false;
    if(bmeSensor.Start() == Status::Ok)
    {
        useBme = true;
        BME280::Measurements baseMeasurements{};
        bmeSensor.Calibrate(baseMeasurements);
        altitudeNode.SetBaseMeasurements(baseMeasurements.pressure, baseMeasurements.temperature);
        bmeNode.Start();
    }

    AdcSensor adcSensor{hadc1, ADC_CHANNEL_1};
    BatterySensorNode<Config::BATTERY_MAX_VOLTAGE, Config::BATTERY_ADC_MAX_VOLTAGE> batterySensor{adcSensor, "BatterySensor", Config::BATTERY_SAMPLING_RATE_HZ};

    OsdResponseReader osdReader{};
    Uart osdUart{huart2, osdReader};
    OsdModule osdModule{osdUart, osdReader};
    OsdController osdController{osdModule};
    OsdHorizonNode osdHorizonNode{osdController};
    OsdBatteryVoltageNode osdBatteryInfoNode{osdController};
    OsdFlightModeNode osdFlightModeNode{osdController};
    OsdRcLinkNode osdRcLinkNode{osdController};
    OsdAltitudeNode osdAltitudeNode{osdController};

    OsdCompositorTask<6> osdCompositorTask{"OsdRenderer", Config::OSD_REFRESH_RATE_HZ};
    osdCompositorTask.Register(osdHorizonNode);
    osdCompositorTask.Register(osdBatteryInfoNode);
    osdCompositorTask.Register(osdFlightModeNode);
    osdCompositorTask.Register(osdRcLinkNode);

    ManualFlightMode lowPosFlightMode{};
    ManualFlightMode midPosFlightMode{};
    CruiseFlightMode highPosFlightMode{"CruiseFlightMode"};
    FailsafeFlightMode failsafeFlightMode{"FailSafeMode"};

    Connect(fusion, static_cast<NodeInput<CruiseFlightMode, Quaternion<float>>&>(highPosFlightMode));

    if (useBme)
    {
        Connect(altitudeNode, osdAltitudeNode);
        Connect(altitudeNode, static_cast<NodeInput<CruiseFlightMode, float>&>(highPosFlightMode));
        osdCompositorTask.Register(osdAltitudeNode);
    }

    Connect(fusion, osdHorizonNode);
    Connect(batterySensor, osdBatteryInfoNode);

    HalfDuplexSoftwareUart swUart{TRAMP_Pin, TRAMP_GPIO_Port, htim10};
    TrampFrameParser trampParser{"TrampFrameParser"};
    TrampProtocolController trampProtocolController{"TrampProtocol"};
    Connect(swUart, trampParser);
    Connect(trampProtocolController, swUart);
    OsdVtxInfo osdVtxNode{osdController, trampProtocolController};
    osdCompositorTask.Register(osdVtxNode);

    fusion.Start();
    batterySensor.Start();

    CrsfFrameReaderNode crsfFrameReader{"CrsfReader"};
    CrsfLib::FrameParserNode crsfFrameParser{};
    CrsfFrameProcessorNode<2> crsfFrameProcessor{};
    Uart crsfUart{huart1, crsfFrameReader};
    DeviceInfoWriteNode crsfDeviceInfo{crsfUart};

    Connect(crsfFrameReader, crsfFrameParser);
    Connect(crsfFrameParser, crsfFrameProcessor);
    Connect(crsfFrameProcessor.GetDeviceInfoNode(), crsfDeviceInfo);
    crsfFrameReader.Start();

    VtxControllerNode vtxControllerNode{trampProtocolController};
    PlaneControllerNode planeControllerNode{model,
        lowPosFlightMode,
        midPosFlightMode,
        highPosFlightMode,
        failsafeFlightMode};
    Connect(crsfFrameProcessor.GetRcPacketNode(), vtxControllerNode);
    Connect(crsfFrameProcessor.GetRcPacketNode(), planeControllerNode);
    Connect(crsfFrameProcessor.GetLinkInfoNode(), osdRcLinkNode);
    Connect(planeControllerNode, osdFlightModeNode);

    osdCompositorTask.StartCompositor();

    vTaskDelete(nullptr);
}
