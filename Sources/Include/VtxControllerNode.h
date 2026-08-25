#pragma once
#include "CrsfFrameProcessorNode.h"
#include "PubSubLib/Node.hpp"
#include "RcChannelsMapping.h"
#include "TrampProtocol.h"

class VtxControllerNode : public NodeInput<VtxControllerNode, RcPacket>
{
public:
    VtxControllerNode(std::shared_ptr<TrampProtocolController> tramp);
    void Process(const RcPacket& packet);

private:
    std::optional<TriSwitchPosition> m_currentPos;
    std::shared_ptr<TrampProtocolController> m_tramp;
};
