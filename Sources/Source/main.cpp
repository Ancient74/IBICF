#include "main.h"
#include "AdcSensor.h"
#include "BME280.h"
#include "Config.h"
#include "FailsafeFlightMode.h"
#include "HalfDuplexSoftwareUart.h"
#include "Log.h"
#include "Logger.h"
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
#include "TaskDelegate.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_tim1_up;

TaskDelegate watchdog{};

using namespace std::chrono_literals;

namespace
{
    constexpr const char* MODULE = "Main";
}

extern "C" void main_cpp()
{
    static Logger logger{"Logger"};
    LogInfo(MODULE, "System start");

    watchdog.Bind();

    static ServoPwmControl leftAileronControl{htim4, TIM_CHANNEL_1, 500us, 2500us};
    static ServoPwmControl rightAileronControl{htim4, TIM_CHANNEL_2, 500us, 2500us};
    static ServoPwmControl leftRuddervatorControl{htim4, TIM_CHANNEL_3, 500us, 2500us};
    static ServoPwmControl rightRuddervatorControl{htim4, TIM_CHANNEL_4, 500us, 2500us};

    static ThrustController thrust{htim3, TIM_CHANNEL_1};

    static SurfaceController leftAileron{leftAileronControl};
    static SurfaceController rightAileron{rightAileronControl};
    static SurfaceController leftRuddervator{leftRuddervatorControl};
    static SurfaceController rightRuddervator{rightRuddervatorControl};
    static PlaneModel model{
        leftAileron,
        rightAileron,
        leftRuddervator,
        rightRuddervator,
        thrust
    };

    static I2cBus bus{hi2c2};
    bus.ScanBus();
    static Mpu6050 mpuSensor{bus};
    const auto mpuStatus = mpuSensor.Start();
    if (mpuStatus != Status::Ok)
        LogError(MODULE, "MPU6050 initialization failed (status=%d)", static_cast<int>(mpuStatus));
    else
        LogInfo(MODULE, "MPU6050 initialized");

    Matrix3f accelR{};
    Matrix3f magnetR{};
    Matrix3f N{};
    auto samplingRate = mpuSensor.GetSamplingRate();

    if (mpuStatus == Status::Ok)
    {
        LogInfo(MODULE, "IMU calibration in progress");
        mpuSensor.Calibrate(accelR, N);
        LogInfo(MODULE, "IMU calibration complete; sampling rate=%u Hz", static_cast<unsigned>(samplingRate));
    }
    else
    {
        LogInfo(MODULE, "IMU calibration skipped due to previous errors");
    }
    static SensorSamplingNode<Mpu6050, GyroscopeAccelerometer, 1> mpuNode{mpuSensor, "MPU"};

    static Qmc5883L qmcSensor{bus};
    static SensorSamplingNode<Qmc5883L, Magnetometer, 1> qmcNode{qmcSensor, "QMC"};
    bool useQmc = false;
    if (Config::ENABLE_MAGNETOMETER_CORRECTION)
    {
        const auto qmcStatus = qmcSensor.Start();
        if (qmcStatus == Status::Ok)
        {
            qmcSensor.GetSensorVariance(magnetR);
            useQmc = true;
            LogInfo(MODULE, "QMC5883L initialized; calibration sampling complete");
        }
        else
        {
            LogWarning(MODULE, "QMC5883L initialization failed (status=%d)", static_cast<int>(qmcStatus));
        }
    }
    else
        LogInfo(MODULE, "Magnetometer correction is disabled");
    static SensorFusionNode<2> fusion{"Fusion", 1.0f / samplingRate, accelR, magnetR, N};

    if (useQmc)
    {
        Connect(qmcNode, fusion.MagInput());
        qmcNode.Start();
    }

    Connect(mpuNode, fusion.ImuInput());
    mpuNode.Start();

    static BME280 bmeSensor{bus};

    static SensorSamplingNode<BME280, BME280::Measurements, 1> bmeNode{bmeSensor, "BME"};
    static EmaFilterNode<BME280::Measurements, 1> bmeFilterNode{0.1f, 5};
    static PressureToAltitudeConverterNode<2> altitudeNode;
    bool useBme = false;
    const auto bmeStatus = bmeSensor.Start();
    if (bmeStatus == Status::Ok)
    {
        useBme = true;
        BME280::Measurements baseMeasurements{};
        bmeSensor.Calibrate(baseMeasurements);
        altitudeNode.SetBaseMeasurements(baseMeasurements.pressure, baseMeasurements.temperature);
        bmeNode.Start();
        LogInfo(MODULE, "BME280 initialized; altitude input enabled");
    }
    else
        LogWarning(MODULE, "BME280 unavailable (status=%d); altitude input disabled", static_cast<int>(bmeStatus));

    static AdcSensor adcSensor{hadc1, ADC_CHANNEL_1};
    static BatterySensorNode<Config::BATTERY_MAX_VOLTAGE, Config::BATTERY_MIN_VOLTAGE, Config::BATTERY_ADC_DIVIDER_RATIO> batterySensor{adcSensor, "BatterySensor"};

    static OsdResponseReader osdReader{};
    static Uart osdUart{huart2, osdReader, watchdog};
    static OsdModule osdModule{osdUart, osdReader};
    static OsdController osdController{osdModule};
    static OsdHorizonNode osdHorizonNode{osdController};
    static OsdBatteryVoltageNode osdBatteryInfoNode{osdController};
    static OsdFlightModeNode osdFlightModeNode{osdController};
    static OsdRcLinkNode osdRcLinkNode{osdController};
    static OsdAltitudeNode osdAltitudeNode{osdController};

    static OsdCompositorTask<6> osdCompositorTask{"OsdRenderer"};
    osdCompositorTask.Register(osdHorizonNode);
    osdCompositorTask.Register(osdBatteryInfoNode);
    osdCompositorTask.Register(osdFlightModeNode);
    osdCompositorTask.Register(osdRcLinkNode);

    static ManualFlightMode lowPosFlightMode{};
    static ManualFlightMode midPosFlightMode{};
    static CruiseFlightMode highPosFlightMode{"CruiseFlightMode"};
    static FailsafeFlightMode failsafeFlightMode{"FailSafeMode"};

    Connect(fusion, static_cast<NodeInput<CruiseFlightMode, Quaternion<float>>&>(highPosFlightMode));

    if (useBme)
    {
        Connect(altitudeNode, osdAltitudeNode);
        Connect(altitudeNode, static_cast<NodeInput<CruiseFlightMode, float>&>(highPosFlightMode));
        osdCompositorTask.Register(osdAltitudeNode);
    }

    Connect(fusion, osdHorizonNode);
    Connect(batterySensor, osdBatteryInfoNode);

    static HalfDuplexSoftwareUart swUart{TRAMP_Pin, TRAMP_GPIO_Port, htim1, hdma_tim1_up};
    static TrampFrameParser trampParser{"TrampFrameParser"};
    static TrampProtocolController trampProtocolController{"TrampProtocol"};
    Connect(swUart, trampParser);
    Connect(trampProtocolController, swUart);
    Connect(trampParser, trampProtocolController);
    swUart.Start();
    trampParser.Start();
    trampProtocolController.Start();
    LogInfo(MODULE, "VTX communication started");

    static OsdVtxInfo osdVtxNode{osdController, trampProtocolController};
    osdCompositorTask.Register(osdVtxNode);

    fusion.Start();
    LogInfo(MODULE, "Sensor fusion started");
    batterySensor.Start();
    LogInfo(MODULE, "Battery sensor started");

    static CrsfFrameReaderNode crsfFrameReader{"CrsfReader"};
    static CrsfLib::FrameParserNode crsfFrameParser{};
    static CrsfFrameProcessorNode<2> crsfFrameProcessor{};
    static Uart crsfUart{huart1, crsfFrameReader, watchdog};
    static DeviceInfoWriteNode crsfDeviceInfo{crsfUart};

    Connect(crsfFrameReader, crsfFrameParser);
    Connect(crsfFrameParser, crsfFrameProcessor);
    Connect(crsfFrameProcessor.GetDeviceInfoNode(), crsfDeviceInfo);

    static VtxControllerNode vtxControllerNode{trampProtocolController};
    static PlaneControllerNode planeControllerNode{model,
        lowPosFlightMode,
        midPosFlightMode,
        highPosFlightMode,
        failsafeFlightMode};
    Connect(crsfFrameProcessor.GetRcPacketNode(), vtxControllerNode);
    Connect(crsfFrameProcessor.GetRcPacketNode(), planeControllerNode);
    Connect(crsfFrameProcessor.GetLinkInfoNode(), osdRcLinkNode);
    Connect(planeControllerNode, osdFlightModeNode);

    crsfFrameReader.Start();
    LogInfo(MODULE, "CRSF receiver started");

    osdCompositorTask.StartCompositor();
    LogInfo(MODULE, "OSD started");

    while (true)
    {
        watchdog.WaitAndProcess();
    }
}
