#pragma once
#include <KalmanLib/Quaternion.hpp>
#include "BatterySensorNode.h"
#include "OsdController.h"
#include "CommonTask.h"
#include <optional>
#include <PubSubLib/Node.hpp>

class OsdBatteryVoltageNode
    : public CommonTask<OsdBatteryVoltageNode, 512, configMAX_PRIORITIES / 2 - 2>
    , public NodeInput<OsdBatteryVoltageNode, BatteryInfo>
{
    using Base = CommonTask<OsdBatteryVoltageNode, 512, configMAX_PRIORITIES / 2 - 2>;

public:
    OsdBatteryVoltageNode(std::shared_ptr<OsdController> osd, const char* taskName, uint16_t updateRateHz)
        : Base(taskName, pdMS_TO_TICKS(1000 / updateRateHz))
        , m_osd(osd)
        , m_xPos(m_osd->Cols() - 6) // -6 for every character before last col (I25.5V)
        , m_yPos(0)
    {
        m_mutex = xSemaphoreCreateMutex();
        configASSERT(m_mutex != nullptr);
        Start();
    }

    ~OsdBatteryVoltageNode()
    {
        Stop();
        vSemaphoreDelete(m_mutex);
    }

    void Process(BatteryInfo batteryInfo)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_latestInfo = batteryInfo;
        xSemaphoreGive(m_mutex);
    }

    void RunTaskIteration()
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        auto voltage = m_latestInfo;
        m_latestInfo = std::nullopt;
        xSemaphoreGive(m_mutex);
        if (!voltage.has_value())
            return;

        char buffer[6];
        const auto value = voltage.value().chargeLevel;
        if (value < 10.0f)
            std::snprintf(buffer, sizeof(buffer), "%.2fv", value);
        else if (value < 100.0f)
            std::snprintf(buffer, sizeof(buffer), "%.1fv", value);
        else
            std::snprintf(buffer, sizeof(buffer), "%.0fv", value);

        const auto percent = voltage.value().percentLevel;
        uint8_t chargeLevelIcon = Charge100CharCode;
        if (percent < 10)
            chargeLevelIcon = Charge0CharCode;
        else if (percent < 15)
            chargeLevelIcon = Charge10CharCode;
        else if (percent < 30)
            chargeLevelIcon = Charge15CharCode;
        else if (percent < 50)
            chargeLevelIcon = Charge30CharCode;
        else if (percent < 80)
            chargeLevelIcon = Charge50CharCode;
        else if (percent < 90)
            chargeLevelIcon = Charge80CharCode;

        std::array<OsdCell, 6> cells;
        cells[0].charCode = chargeLevelIcon;
        cells[0].col = m_xPos;
        cells[0].row = m_yPos;
        for (uint8_t i = 1; i < cells.size(); i++)
        {
            cells[i].charCode = s_symbols[buffer[i-1]];
            cells[i].col = m_xPos + i;
            cells[i].row = m_yPos;
        }
        m_osd->DrawCells(cells);
    }

private:
    static constexpr uint8_t Charge100CharCode = 0xE9;
    static constexpr uint8_t Charge80CharCode = 0xEA;
    static constexpr uint8_t Charge50CharCode = 0xEB;
    static constexpr uint8_t Charge30CharCode = 0xEC;
    static constexpr uint8_t Charge15CharCode = 0xED;
    static constexpr uint8_t Charge10CharCode = 0xEE;
    static constexpr uint8_t Charge0CharCode = 0xEF;

    static constexpr std::array<uint8_t, 256> s_symbols = [] {
        std::array<uint8_t, 256> result{};

        result['0'] = 0x01;
        result['1'] = 0x02;
        result['2'] = 0x03;
        result['3'] = 0x04;
        result['4'] = 0x05;
        result['5'] = 0x06;
        result['6'] = 0x07;
        result['7'] = 0x08;
        result['8'] = 0x09;
        result['9'] = 0x0A;
        result['.'] = 0x80;
        result['v'] = 0x3A;
        result[' '] = 0x00;

        return result;
    }();

    std::shared_ptr<OsdController> m_osd;
    const uint8_t m_xPos;
    const uint8_t m_yPos;
    std::optional<BatteryInfo> m_latestInfo{};
    SemaphoreHandle_t m_mutex{};
};
