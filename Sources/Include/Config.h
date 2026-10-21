#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include <KalmanLib/Matrix.hpp>
#include <KalmanLib/Quaternion.hpp>

namespace Config
{
    constexpr bool ENABLE_MAGNETOMETER_CORRECTION = false;

    constexpr bool ENABLE_CALIBRATION
#ifdef DEBUG
        = true;
#else
        = false;
#endif
    constexpr uint32_t ACCELEROMETER_TO_BODY_CALIBRATION_NODE_TIMEOUT_MS = 200;
    constexpr uint16_t ACCELEROMETER_TO_BODY_CALIBRATION_NODE_BUFFER_SIZE = 32;

    constexpr uint32_t ADC_READ_TIMEOUT_MS = 200;

    constexpr uint32_t BATTERY_ADC_OVERSAMPLING_RATIO = 16;
    constexpr float BATTERY_VREF_CALIBRATION_MAX_VALUE = 4095.0f;
    constexpr float BATTERY_MAX_VOLTAGE = 25.2f;
    constexpr float BATTERY_MIN_VOLTAGE = 22.2f;
    constexpr float BATTERY_DIVIDER_ERROR_COMPENSATION_RATIO = 1.04f;
    constexpr float BATTERY_ADC_DIVIDER_RATIO = ((75000.0f + 10000.0f) / 10000.0f) * BATTERY_DIVIDER_ERROR_COMPENSATION_RATIO;
    constexpr uint32_t BATTERY_SAMPLING_RATE_HZ = 20;

    constexpr uint16_t UART_RX_BUFFER_SIZE = 256;
    constexpr uint16_t UART_RESTART_INTERVAL_MS = 500;
    constexpr uint16_t UART_REINIT_AFTER_ATTEMPTS = 10;

    constexpr uint16_t MPU_RESET_TIMEOUT = 100;

    constexpr uint16_t I2C_READ_WRITE_TIMEOUT = 100;
    constexpr uint8_t I2C_RETRY_ATTEMPTS = 3;

    constexpr uint16_t SW_UART_BUFFER_SIZE = 256;
    constexpr uint16_t SW_UART_CHUNK_SIZE = 16;
    constexpr uint16_t SW_UART_BYTE_SIZE = 10; // 8N1
    constexpr uint32_t SW_UART_TIMEOUT_MS = 200;

    constexpr uint16_t CRSF_READER_BUFFER_SIZE = 256;
    constexpr uint16_t CRSF_READER_CHUNK_SIZE = 32;
    constexpr uint16_t SENSOR_FUSION_NODE_QUEUE_SIZE = 16;
    constexpr uint16_t SENSOR_FUSION_NODE_TIMEOUT_MS = 200;
    constexpr uint16_t OSD_RESPONSE_READER_BUFFER_SIZE = 64;
    constexpr uint16_t OSD_MODULE_COMMAND_SEQUENCE_SIZE = 64;
    constexpr uint32_t OSD_COMMAND_RESPONSE_TIMEOUT_MS = 100;

    constexpr uint8_t NTSC_ROWS = 12;
    constexpr uint8_t PAL_ROWS = 16;
    constexpr uint8_t NTSC_MODE = 0;
    constexpr uint8_t PAL_MODE = 1;
    constexpr uint8_t OSD_OPERATION_MODE = PAL_MODE;
    constexpr uint8_t OSD_ROWS = OSD_OPERATION_MODE == NTSC_MODE ? NTSC_ROWS : PAL_ROWS;
    constexpr uint8_t OSD_COLS = 29;
    constexpr uint8_t OSD_MAX_TEXT_SIZE = OSD_COLS;

    constexpr uint8_t OSD_REFRESH_RATE_HZ = 20;

    constexpr uint32_t CRSF_READER_TIMEOUT_MS = 200;

    constexpr uint32_t TRAMP_PROTOCOL_POLLING_RATE_MS = 500;
    constexpr uint8_t TRAMP_PROTOCOL_MAX_SET_RETRIES = 20;
    constexpr uint32_t TRAMP_PROTOCOL_RESPONSE_TIMEOUT_MS = TRAMP_PROTOCOL_POLLING_RATE_MS * 5;

    constexpr uint16_t VTX_PIT_MODE_POWER = 25;
    constexpr uint16_t VTX_MID_POS_POWER = 200;
    constexpr uint16_t VTX_HIGH_POS_POWER = 400;

    constexpr uint8_t CRUISE_FLIGHT_MODE_REFRESH_RATE_HZ = 200;

    constexpr float ROLL_Kp = 1.2f;
    constexpr float ROLL_Ki = 0.0f;
    constexpr float ROLL_Kd = 0.15f;
    constexpr float ROLL_OutMin = -30.0f;
    constexpr float ROLL_OutMax = 30.0f;
    constexpr float ROLL_IntMin = -10.0f;
    constexpr float ROLL_IntMax = 10.0f;

    constexpr float ALT_Kp = 0.8f;
    constexpr float ALT_Ki = 0.0f;
    constexpr float ALT_Kd = 1.5f;
    constexpr float ALT_OutMin = -15.0f;
    constexpr float ALT_OutMax = 15.0f;
    constexpr float ALT_IntMin = -5.0f;
    constexpr float ALT_IntMax = 5.0f;

    constexpr float CRUISE_TRIM_PITCH_DEG = 3.0f;

    constexpr uint32_t FAILSAFE_LINK_TIMEOUT_MS = 3000;
    constexpr int32_t  FAILSAFE_HOLD_THRUST = 30;
    constexpr uint32_t RC_LINK_TIMEOUT_MS = 800;
    constexpr uint32_t FAILSAFE_OUTPUT_PUSH_PERIOD_MS = 20;

    constexpr uint16_t LOGGER_QUEUE_SIZE = 10;
    constexpr uint16_t LOGGER_QUEUE_TIMEOUT_MS = 200;
    constexpr uint16_t LOGGER_BUSY_RETRY_INTERVAL_MS = 10;
    constexpr uint16_t LOGGER_BUSY_RETRY_ATTEMPTS = 10;

    constexpr Vector3f WORLD_UP_VECTOR = []
    {
        Vector3f vector{};
        vector(1, 0) = 1.0f;
        return vector;
    }();
    constexpr Vector3f WORLD_RIGHT_VECTOR = []
    {
        Vector3f vector{};
        vector(2, 0) = 1.0f;
        return vector;
    }();

    constexpr Vector3f IMU_WORLD_UP_VECTOR = []
    {
        Vector3f vector{};
        vector(2, 0) = 1.0f;
        return vector;
    }();

    // These vectors are obtained by performing calibration using AccelerometerToBodyCalibrationNode
    constexpr Vector3f IMU_TO_BODY_UP_VECTOR = []
    {
        Vector3f vector{};
        vector(2, 0) = 1.0f;
        return vector;
    }();
    constexpr Vector3f IMU_TO_BODY_RIGHT_VECTOR = []
    {
        Vector3f vector{};
        vector(0, 0) = 1.0f;
        return vector;
    }();
    static_assert(NormSquared(IMU_TO_BODY_UP_VECTOR) == 1.0f);
    static_assert(NormSquared(IMU_TO_BODY_RIGHT_VECTOR) == 1.0f);

    constexpr Vector3f IMU_TO_BODY_FORWARD_VECTOR = CrossProduct(IMU_TO_BODY_UP_VECTOR, IMU_TO_BODY_RIGHT_VECTOR);
    static_assert(NormSquared(IMU_TO_BODY_FORWARD_VECTOR) == 1.0f);

    constexpr Matrix3f BODY_TO_IMU_BODY_ROTATION_MATRIX = []
    {
        Matrix3f matrix{};
        matrix(0, 0) = IMU_TO_BODY_FORWARD_VECTOR(0, 0);
        matrix(1, 0) = IMU_TO_BODY_FORWARD_VECTOR(1, 0);
        matrix(2, 0) = IMU_TO_BODY_FORWARD_VECTOR(2, 0);
        matrix(0, 1) = IMU_TO_BODY_UP_VECTOR(0, 0);
        matrix(1, 1) = IMU_TO_BODY_UP_VECTOR(1, 0);
        matrix(2, 1) = IMU_TO_BODY_UP_VECTOR(2, 0);
        matrix(0, 2) = IMU_TO_BODY_RIGHT_VECTOR(0, 0);
        matrix(1, 2) = IMU_TO_BODY_RIGHT_VECTOR(1, 0);
        matrix(2, 2) = IMU_TO_BODY_RIGHT_VECTOR(2, 0);
        return matrix;
    }();
    constexpr Quaternion<float> BODY_TO_IMU_BODY_QUATERNION = Quaternion<float>::FromRotationMatrix(BODY_TO_IMU_BODY_ROTATION_MATRIX);
    constexpr Quaternion<float> IMU_WORLD_FIX_QUATERNION  = Quaternion<float>::FromTwoVectors(IMU_WORLD_UP_VECTOR, WORLD_UP_VECTOR);

    namespace Task
    {
        constexpr uint8_t DELEGATE_TASK_DEPTH = 5;
        constexpr uint8_t DELEGATE_TASK_SLOTS = 5;

        constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;

        constexpr UBaseType_t LOGGER_PRIORITY                                 = 1;
        constexpr UBaseType_t OSD_RENDERER_TASK_PRIORITY                      = 2;
        constexpr UBaseType_t TRAMP_FRAME_PARSER_PRIORITY                     = 3;
        constexpr UBaseType_t TRAMP_PROTOCOL_CONTROLLER_PRIORITY              = 3;
        constexpr UBaseType_t HALF_DUPLEX_SOFTWARE_UART_PRIORITY              = 4;
        constexpr UBaseType_t BATTERY_SENSOR_NODE_PRIORITY                    = 5;
        constexpr UBaseType_t ACCELEROMETER_TO_BODY_CALIBRATION_NODE_PRIORITY = 5;
        constexpr UBaseType_t CRUISE_FLIGHT_MODE_PRIORITY                     = 6;
        constexpr UBaseType_t FAILSAFE_FLIGHT_MODE_PRIORITY                   = 7;
        constexpr UBaseType_t CRSF_FRAME_READER_NODE_PRIORITY                 = 8;
        constexpr UBaseType_t SENSOR_FUSION_NODE_PRIORITY                     = 9;
        constexpr UBaseType_t SENSOR_SAMPLING_NODE_PRIORITY                   = 10;

        constexpr uint16_t HALF_DUPLEX_SOFTWARE_UART_STACK_SIZE              = 512;
        constexpr uint16_t CRUISE_FLIGHT_MODE_STACK_SIZE                     = 512;
        constexpr uint16_t CRSF_FRAME_READER_NODE_STACK_SIZE                 = 512;
        constexpr uint16_t FAILSAFE_FLIGHT_MODE_STACK_SIZE                   = 512;
        constexpr uint16_t BATTERY_SENSOR_NODE_STACK_SIZE                    = 512;
        constexpr uint16_t TRAMP_FRAME_PARSER_STACK_SIZE                     = 512;
        constexpr uint16_t TRAMP_PROTOCOL_CONTROLLER_STACK_SIZE              = 512;
        constexpr uint16_t SENSOR_FUSION_NODE_STACK_SIZE                     = 512;
        constexpr uint16_t SENSOR_SAMPLING_NODE_STACK_SIZE                   = 512;
        constexpr uint16_t ACCELEROMETER_TO_BODY_CALIBRATION_NODE_STACK_SIZE = 512;
        constexpr uint16_t OSD_RENDERER_TASK_STACK_SIZE                      = 1024;
        constexpr uint16_t LOGGER_STACK_SIZE                                 = 2048;
    }
}
