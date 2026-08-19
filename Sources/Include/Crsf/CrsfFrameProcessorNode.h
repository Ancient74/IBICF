#pragma once
#include "CrsfLib/Frame.hpp"
#include <memory>
#include "Uart.h"
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"
#include "KalmanLib/MultiplicativeExtendedKalmanFilter.hpp"

class CrsfFrameProcessorNode : public NodeInput<CrsfFrameProcessorNode, CrsfLib::FrameView>
{
public:
    explicit CrsfFrameProcessorNode(IWriter& writer, std::shared_ptr<PlaneModel> model);

    void Process(CrsfLib::FrameView frame);
private:
    IWriter& m_writer;
    std::shared_ptr<PlaneModel> m_model = nullptr;
};
