#pragma once
#include "CrsfFrameProcessorNode.h"
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"
#include "FlightMode.h"
#include <memory>

class PlaneControllerNode
    : public NodeInput<PlaneControllerNode, RcPacket>
    , public NodeOutput<PlaneControllerNode, std::span<const char>, 1>
{
public:
    PlaneControllerNode(std::shared_ptr<PlaneModel> model
        , std::unique_ptr<IFlightMode> lowPosFlightMode
        , std::unique_ptr<IFlightMode> midPosFlightMode
        , std::unique_ptr<IFlightMode> highPosFlightMode);
    void Process(const RcPacket& packet);

private:
    bool m_isInitialized = false;
    IFlightMode* m_currentFlightMode = nullptr;
    std::unique_ptr<IFlightMode> m_lowPosFlightMode;
    std::unique_ptr<IFlightMode> m_midPosFlightMode;
    std::unique_ptr<IFlightMode> m_highPosFlightMode;
    std::shared_ptr<PlaneModel> m_model;
};
