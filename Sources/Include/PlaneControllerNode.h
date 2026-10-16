#pragma once
#include "CrsfFrameProcessorNode.h"
#include "FlightControllerLib/PlaneModel.hpp"
#include "PubSubLib/Node.hpp"
#include "FlightMode.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <atomic>

class PlaneControllerNode
    : public NodeInput<PlaneControllerNode, RcPacket>
    , public NodeOutput<PlaneControllerNode, std::span<const char>, 1>
{
public:
    PlaneControllerNode(PlaneModel& model
        , IFlightMode& disarmedFlightMode
        , IFlightMode& lowPosFlightMode
        , IFlightMode& midPosFlightMode
        , IFlightMode& highPosFlightMode
        , IFlightMode& failSafeFlightMode);
    void Process(const RcPacket& packet);

private:
    static void WatchdogCallback(TimerHandle_t timer);
    static void OutputPushCallback(TimerHandle_t timer);
    void OnLinkLost();
    void PushFailsafeOutputs();

    std::atomic<IFlightMode*> m_currentFlightMode;
    IFlightMode& m_disarmedFlightMode;
    IFlightMode& m_lowPosFlightMode;
    IFlightMode& m_midPosFlightMode;
    IFlightMode& m_highPosFlightMode;
    IFlightMode& m_failSafeFlightMode;
    PlaneModel& m_model;

    TimerHandle_t m_watchdogTimer = nullptr;
    TimerHandle_t m_outputPushTimer = nullptr;
    StaticTimer_t m_watchdogTimerBuffer{};
    StaticTimer_t m_outputPushTimerBuffer{};
};
