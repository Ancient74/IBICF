#pragma once
#include "CrsfFrameProcessorNode.h"
#include "PubSubLib/Node.hpp"
#include "RcChannelsMapping.h"
#include "TrampProtocol.h"

class VtxControllerNode : public NodeInput<VtxControllerNode, RcPacket>
{
public:
    explicit VtxControllerNode(TrampProtocolController& tramp);
    void Process(const RcPacket& packet);

private:
    std::optional<TriSwitchPosition> m_currentPos;
    TrampProtocolController& m_tramp;
};
