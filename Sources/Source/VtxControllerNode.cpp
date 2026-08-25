#include "VtxControllerNode.h"
#include "Config.h"

VtxControllerNode::VtxControllerNode(std::shared_ptr<TrampProtocolController> tramp)
        : m_tramp(std::move(tramp))
{
}

void VtxControllerNode::Process(const RcPacket& packet)
{
    if (m_currentPos.has_value() && *m_currentPos == packet.vtxMode)
        return;
    m_currentPos = packet.vtxMode;
    switch (packet.vtxMode)
    {
    case TriSwitchPosition::Low:
        m_tramp->SetPitMode(true);
        m_tramp->SetPower(Config::VTX_PIT_MODE_POWER);
        break;
    case TriSwitchPosition::Mid:
        m_tramp->SetPitMode(false);
        m_tramp->SetPower(Config::VTX_MID_POS_POWER);
        break;
    case TriSwitchPosition::High:
        m_tramp->SetPitMode(false);
        m_tramp->SetPower(Config::VTX_HIGH_POS_POWER);
        break;
    }
}
