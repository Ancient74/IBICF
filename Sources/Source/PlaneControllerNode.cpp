#include "PlaneControllerNode.h"

PlaneControllerNode::PlaneControllerNode(std::shared_ptr<PlaneModel> model)
        : m_model(std::move(model))
{}

void PlaneControllerNode::Process(const RcPacket& packet)
{
    m_model->Pitch(packet.pitch);
    m_model->Roll(packet.roll);
    m_model->Yaw(packet.yaw);

    if (!m_isInitialized && packet.isArmed)
    {
        m_model->Thrust(0);
        return;
    }
    m_isInitialized = true;
    uint8_t allowedThrust = packet.isArmed ? static_cast<uint8_t>(packet.thrust) : 0;
    m_model->Thrust(allowedThrust);
}
