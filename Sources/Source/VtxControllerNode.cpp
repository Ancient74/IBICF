#include "VtxControllerNode.h"
#include "Config.h"

namespace
{
    constexpr const char* MODULE = "VTX";
}

VtxControllerNode::VtxControllerNode(TrampProtocolController& tramp)
        : m_tramp(tramp)
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
        LogInfo(MODULE, "VTX power set to %d", Config::VTX_PIT_MODE_POWER);
        m_tramp.SetPitMode(true);
        m_tramp.SetPower(Config::VTX_PIT_MODE_POWER);
        break;
    case TriSwitchPosition::Mid:
        m_tramp.SetPitMode(false);
        m_tramp.SetPower(Config::VTX_MID_POS_POWER);
        LogInfo(MODULE, "VTX power set to %d", Config::VTX_MID_POS_POWER);
        break;
    case TriSwitchPosition::High:
        m_tramp.SetPitMode(false);
        m_tramp.SetPower(Config::VTX_HIGH_POS_POWER);
        LogInfo(MODULE, "VTX power set to %d", Config::VTX_HIGH_POS_POWER);
        break;
    }
}
