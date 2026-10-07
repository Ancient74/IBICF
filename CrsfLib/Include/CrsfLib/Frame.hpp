#pragma once
#include <stdint.h>
#include <array>
#include <span>
#include <optional>

namespace CrsfLib
{
    enum class Address : uint8_t
    {
        BroadcastAddress                 = 0x00,
        Cloud                            = 0x0E,
        UsbDevice                        = 0x10,
        BluetoothOrWifiModule            = 0x12,
        WifiReceiver                     = 0x13,
        VideoReceiver                    = 0x14,
        OsdOrTbsCorePnpPro               = 0x80,
        Esc1                             = 0x90,
        Esc2                             = 0x91,
        Esc3                             = 0x92,
        Esc4                             = 0x93,
        Esc5                             = 0x94,
        Esc6                             = 0x95,
        Esc7                             = 0x96,
        Esc8                             = 0x97,
        Reserved8A                       = 0x8A,
        CrossfireReservedB0              = 0xB0,
        CrossfireReservedB2              = 0xB2,
        VoltageCurrentSensor             = 0xC0,
        Gps                              = 0xC2,
        TbsBlackbox                      = 0xC4,
        FlightController                 = 0xC8,
        ReservedCA                       = 0xCA,
        RaceTag                          = 0xCC,
        Vtx                              = 0xCE,
        RemoteControl                    = 0xEA,
        RepeaterReceiver                 = 0xEB,
        RcReceiver                       = 0xEC,
        RepeaterTransmitterModule        = 0xED,
        RcTransmitterModule              = 0xEE,
        ReservedF0                       = 0xF0,
        ReservedF2                       = 0xF2
    };

    constexpr bool IsValidAddress(uint8_t value)
    {
        switch (static_cast<Address>(value))
        {
            case Address::BroadcastAddress:
            case Address::Cloud:
            case Address::UsbDevice:
            case Address::BluetoothOrWifiModule:
            case Address::WifiReceiver:
            case Address::VideoReceiver:
            case Address::OsdOrTbsCorePnpPro:
            case Address::Esc1:
            case Address::Esc2:
            case Address::Esc3:
            case Address::Esc4:
            case Address::Esc5:
            case Address::Esc6:
            case Address::Esc7:
            case Address::Esc8:
            case Address::Reserved8A:
            case Address::CrossfireReservedB0:
            case Address::CrossfireReservedB2:
            case Address::VoltageCurrentSensor:
            case Address::Gps:
            case Address::TbsBlackbox:
            case Address::FlightController:
            case Address::ReservedCA:
            case Address::RaceTag:
            case Address::Vtx:
            case Address::RemoteControl:
            case Address::RepeaterReceiver:
            case Address::RcReceiver:
            case Address::RepeaterTransmitterModule:
            case Address::RcTransmitterModule:
            case Address::ReservedF0:
            case Address::ReservedF2:
                return true;

            default:
                return false;
        }
    }

    enum class FrameType : uint8_t {
        // Broadcast frames (simple header)
        Gps                          = 0x02,
        GpsTime                      = 0x03,
        GpsExtended                  = 0x06,
        VariometerSensor             = 0x07,
        BatterySensor                = 0x08,
        BaroAltitudeVspeed           = 0x09,
        Airspeed                     = 0x0A,
        Heartbeat                    = 0x0B,
        Rpm                          = 0x0C,
        Temperature                  = 0x0D,
        Voltages                     = 0x0E,
        VtxTelemetry                 = 0x10,
        Barometer                    = 0x11,
        Magnetometer                 = 0x12,
        AccelGyro                    = 0x13,
        LinkStatistics               = 0x14,
        LinkStatisticsRepeater       = 0x15,
        RcChannelsPacked             = 0x16,
        SubsetRcChannelsPacked       = 0x17,  // discouraged
        RcChannelsPacked11Bit        = 0x18,  // unused
        ReservedCrossfire19          = 0x19,
        ReservedCrossfire1A          = 0x1A,
        ReservedCrossfire1B          = 0x1B,
        LinkStatisticsRx             = 0x1C,
        LinkStatisticsTx             = 0x1D,
        Attitude                     = 0x1E,
        MavlinkFc                    = 0x1F,
        FlightMode                   = 0x21,
        EspNowMessages               = 0x22,
        Reserved27                   = 0x27,

        //Extended frames (have destination + origin)
        ParamPingDevices             = 0x28,
        ParamDeviceInformation       = 0x29,
        ParamSettingsEntry           = 0x2B,
        ParamSettingsRead            = 0x2C,
        ParamValueWrite              = 0x2D,
        DirectCommand                = 0x32,  // has its own inner CRC (poly 0xBA)

        Logging                      = 0x34, // 0x34 is >= 0x28 but uses the SIMPLE header (debug logging)

        Reserved36                   = 0x36,
        Reserved38                   = 0x38,
        RemoteRelated                = 0x3A,
        Game                         = 0x3C,
        Reserved3E                   = 0x3E,
        Reserved40                   = 0x40,
        KissFcReserved78             = 0x78,
        KissFcReserved79             = 0x79,
        MspRequest                   = 0x7A,
        MspResponse                  = 0x7B,
        ArduPilotLegacyReserved      = 0x7F,
        ArduPilotPassthrough         = 0x80,
        MlrsReserved81               = 0x81,
        MlrsReserved82               = 0x82,
        RotorflightTelemetryEnvelope = 0x88,
        MavlinkEnvelope              = 0xAA,
        MavlinkSysStatusSensor       = 0xAC,
    };

    constexpr bool IsExtendedHeader(FrameType type)
    {
        const auto rawValue = static_cast<uint8_t>(type);
        if (rawValue < 0x28) return false;
        switch (type) {
            case FrameType::Logging:              return false;
            case FrameType::ArduPilotPassthrough: return false;  // "broadcast frame" per spec
            default:                              return true;
        }
    }

    constexpr size_t MAX_FRAME_SIZE = 64;

    struct FrameView
    {
        FrameType type;
        std::optional<Address> destination;
        std::optional<Address> origin;
        std::span<const uint8_t> payload;
    };

    class RawFrame
    {
    public:
        RawFrame(std::span<const uint8_t> data);

        Address SyncByte() const;
        uint8_t Length() const;
        FrameType Type() const;
        bool IsExtended() const;
        bool CheckCrc() const;

        FrameView GetFrameView() const;

    private:
        std::span<const uint8_t> m_data;
    };

    struct ParamDeviceInformationFrame
    {
        static constexpr FrameType type = FrameType::ParamDeviceInformation;

        const char* deviceName;
        uint32_t serialNumber;
        uint32_t hardwareId;
        uint32_t firmwareId;
        uint8_t parametersTotal;
        uint8_t parameterVersionNumber;
    };
    static_assert(std::is_aggregate_v<ParamDeviceInformationFrame>);

    struct LinkStatisticsFrame
    {
        static constexpr FrameType type = FrameType::LinkStatistics;

        uint8_t upRssiAnt1;       // Uplink RSSI Antenna 1 (dBm * -1)
        uint8_t upRssiAnt2;       // Uplink RSSI Antenna 2 (dBm * -1)
        uint8_t upLinkQuality;    // Uplink Package success rate / Link quality (%)
        int8_t upSnr;             // Uplink SNR (dB)
        uint8_t activeAntenna;    // number of currently best antenna
        uint8_t rfProfile;        // enum {4fps = 0 , 50fps, 150fps}
        uint8_t upRfPower;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                                  // 500mW, 1000mW, 2000mW, 250mW, 50mW}
        uint8_t downRssi;         // Downlink RSSI (dBm * -1)
        uint8_t downLinkQuality;  // Downlink Package success rate / Link quality (%)
        int8_t downSnr;           // Downlink SNR (dB)
    };
    static_assert(std::is_aggregate_v<LinkStatisticsFrame>);

    struct RcChannelsPackedFrame
    {
        static constexpr FrameType type = FrameType::RcChannelsPacked;

        std::array<uint16_t, 16> m_channels;
    };
    static_assert(std::is_aggregate_v<RcChannelsPackedFrame>);

}
