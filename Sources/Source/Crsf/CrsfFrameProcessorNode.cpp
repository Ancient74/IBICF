#include "CrsfFrameProcessorNode.h"
#include "CrsfLib/Frame.hpp"
#include "CrsfLib/FrameBuilder.hpp"
#include "CrsfLib/FrameUnpack.hpp"
#include "KalmanLib/Matrix.hpp"
#include "ServoPwmControl.h"
#include "SurfaceController.h"
#include "ThrustController.h"
#include "RcChannelsMapping.h"
#include <cmath>

namespace
{
    using Roll = Scaled<Channel<0>, 172, 1811, -90, 90>;
    using Pitch = Scaled<Channel<1>, 172, 1811, -45, 45>;
    using Yaw = Scaled<Channel<3>, 172, 1811, -45, 45>;
    using Thrust = Scaled<Channel<2>, 172, 1811, 0, 100>;
    static DoubleSwitch<Channel<4>> armSwitch{};
    static TriSwitch<Channel<5>> vtxSwitch{};
}

using namespace std::chrono_literals;

CrsfFrameProcessorNode::CrsfFrameProcessorNode(IWriter& writer, std::shared_ptr<PlaneModel> model)
    : m_writer(writer)
    , m_model(model)
{
}

void CrsfFrameProcessorNode::Process(CrsfLib::FrameView frame)
{
    switch (frame.type)
    {
    case CrsfLib::FrameType::ParamPingDevices:
    {
        CrsfLib::ParamDeviceInformationFrame paramDeviceInformationFrame{};
        const auto buffer = CrsfLib::BuildFrame(paramDeviceInformationFrame);
        m_writer.Write(buffer);
        break;
    }
    case CrsfLib::FrameType::LinkStatistics:
    {
        // const auto linkStats = CrsfLib::Unpack<CrsfLib::LinkStatisticsFrame>(frame.payload);
        // printf("%d\n", linkStats->upSnr);
        break;
    }
    case CrsfLib::FrameType::RcChannelsPacked:
    {
        const auto packet = CrsfLib::Unpack<CrsfLib::RcChannelsPackedFrame>(frame.payload);

        m_model->Thrust(Thrust::Get(packet));
        m_model->Pitch(Pitch::Get(packet));
        m_model->Yaw(Yaw::Get(packet));
        m_model->Roll(Roll::Get(packet));
        bool isArmed = armSwitch.Update(packet);
        auto vtxMode = vtxSwitch.Update(packet);

        break;
    }
    default:
        break;
    }
}
