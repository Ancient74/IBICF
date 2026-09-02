#pragma once
#include "CrsfFrameProcessorNode.h"
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"
#include "FlightMode.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <atomic>
#include <memory>

class PlaneControllerNode
    : public NodeInput<PlaneControllerNode, RcPacket>
    , public NodeOutput<PlaneControllerNode, std::span<const char>, 1>
{
public:
    PlaneControllerNode(std::shared_ptr<PlaneModel> model
        , std::unique_ptr<IFlightMode> lowPosFlightMode
        , std::unique_ptr<IFlightMode> midPosFlightMode
        , std::unique_ptr<IFlightMode> highPosFlightMode
        , std::unique_ptr<IFlightMode> failSafeFlightMode);
    void Process(const RcPacket& packet);

private:
    static void WatchdogCallback(TimerHandle_t timer);
    static void OutputPushCallback(TimerHandle_t timer);
    void OnLinkLost();
    void PushFailsafeOutputs();

    std::atomic<bool> m_isInitialized;
    std::atomic<bool> m_isArmed;
    std::atomic<IFlightMode*> m_currentFlightMode;
    std::unique_ptr<IFlightMode> m_lowPosFlightMode;
    std::unique_ptr<IFlightMode> m_midPosFlightMode;
    std::unique_ptr<IFlightMode> m_highPosFlightMode;
    std::unique_ptr<IFlightMode> m_failSafeFlightMode;
    std::shared_ptr<PlaneModel> m_model;

    TimerHandle_t m_watchdogTimer = nullptr;
    TimerHandle_t m_outputPushTimer = nullptr;
    StaticTimer_t m_watchdogTimerBuffer{};
    StaticTimer_t m_outputPushTimerBuffer{};
};
