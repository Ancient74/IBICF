#pragma once
#include "CrsfLib/Frame.hpp"
#include "CrsfLib/FrameBuilder.hpp"
#include "CrsfLib/FrameUnpack.hpp"
#include "Uart.h"
#include "PubSubLib/Node.hpp"
#include "KalmanLib/MultiplicativeExtendedKalmanFilter.hpp"
#include "RcChannelsMapping.h"

struct RcPacket
{
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
    int32_t thrust;
    bool isArmed;
    TriSwitchPosition vtxMode;
    TriSwitchPosition flightMode;
};

template<typename CrsfOut, uint8_t MaxSubs>
class CrsfOutNode : public NodeOutput<CrsfOutNode<CrsfOut, MaxSubs>, CrsfOut, MaxSubs>
{
public:
    using NodeOutput<CrsfOutNode<CrsfOut, MaxSubs>, CrsfOut, MaxSubs>::Publish;
};

template<uint8_t MaxSubs>
class CrsfFrameProcessorNode : public NodeInput<CrsfFrameProcessorNode<MaxSubs>, CrsfLib::FrameView>
{
public:
    void Process(CrsfLib::FrameView frame)
    {
        switch (frame.type)
        {
        case CrsfLib::FrameType::ParamPingDevices:
        {
            CrsfLib::ParamDeviceInformationFrame paramDeviceInformationFrame{};
            const auto buffer = CrsfLib::BuildFrame(paramDeviceInformationFrame);
            m_deviceInfoNode.Publish(buffer);
            break;
        }
        case CrsfLib::FrameType::LinkStatistics:
        {
            const auto linkStats = CrsfLib::Unpack<CrsfLib::LinkStatisticsFrame>(frame.payload);
            m_linkInfo.Publish(linkStats);
            break;
        }
        case CrsfLib::FrameType::RcChannelsPacked:
        {
            using Roll = Scaled<Channel<0>, 172, 1811, -90, 90>;
            using Pitch = Scaled<Channel<1>, 172, 1811, -45, 45>;
            using Yaw = Scaled<Channel<3>, 172, 1811, -45, 45>;
            using Thrust = Scaled<Channel<2>, 172, 1811, 0, 100>;

            const auto packetFrame = CrsfLib::Unpack<CrsfLib::RcChannelsPackedFrame>(frame.payload);
            RcPacket packet{};
            packet.thrust = Thrust::Get(packetFrame);
            packet.pitch = Pitch::Get(packetFrame);
            packet.yaw = Yaw::Get(packetFrame);
            packet.roll = Roll::Get(packetFrame);
            packet.isArmed = m_armSwitch.Update(packetFrame);
            packet.vtxMode = m_vtxSwitch.Update(packetFrame);
            packet.flightMode = m_flightModeSwitch.Update(packetFrame);
            m_rcPacketNode.Publish(packet);
            if constexpr (Config::ENABLE_CALIBRATION)
            {
                m_calibrationNode.Publish(m_calibrationSwitch.Update(packetFrame));
            }
            break;
        }
        default:
            break;
        }
    }

    auto& GetRcPacketNode()
    {
        return m_rcPacketNode;
    }

    auto& GetDeviceInfoNode()
    {
        return m_deviceInfoNode;
    }

    auto& GetLinkInfoNode()
    {
        return m_linkInfo;
    }

    auto& GetCalibrationNode()
    {
        return m_calibrationNode;
    }

private:
    DoubleSwitch<Channel<4>> m_armSwitch{};
    TriSwitch<Channel<5>> m_vtxSwitch{};
    TriSwitch<Channel<6>> m_flightModeSwitch{};
    DoubleSwitch<Channel<7>> m_calibrationSwitch{};

    CrsfOutNode<RcPacket, MaxSubs> m_rcPacketNode;
    CrsfOutNode<std::span<const uint8_t>, MaxSubs> m_deviceInfoNode;
    CrsfOutNode<CrsfLib::LinkStatisticsFrame, MaxSubs> m_linkInfo;
    CrsfOutNode<bool, MaxSubs> m_calibrationNode;
};
