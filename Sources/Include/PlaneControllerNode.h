#pragma once
#include "CrsfFrameProcessorNode.h"
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"

class PlaneControllerNode : public NodeInput<PlaneControllerNode, RcPacket>
{
public:
    PlaneControllerNode(std::shared_ptr<PlaneModel> model);
    void Process(const RcPacket& packet);

private:
    bool m_isInitialized = false;
    std::shared_ptr<PlaneModel> m_model;
};
