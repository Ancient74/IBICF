#pragma once
#include "AltitudeWidget.h"
#include "BatteryWidget.h"
#include "CrosshairWidget.h"
#include "HorizonLineWidget.h"
#include "LqWidget.h"
#include "OsdController.h"
#include "RssiWidget.h"
#include "TextWidget.h"
#include "ThrustWidget.h"
#include "VtxInfoWidget.h"
#include "BatterySensorNode.h"
#include "Config.h"
#include "CrsfLib/Frame.hpp"
#include "KalmanLib/Quaternion.hpp"
#include "PubSubLib/Node.hpp"
#include "TrampProtocol.h"
#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <tuple>

template <typename Widget>
inline constexpr std::size_t OsdWidgetCellCount = std::tuple_size_v<typename Widget::Content> * std::tuple_size_v<typename Widget::Content::value_type>;

class OsdLayout
    : public NodeInput<OsdLayout, Quaternion<float>>
    , public NodeInput<OsdLayout, BatteryInfo>
    , public NodeInput<OsdLayout, std::span<const char>>
    , public NodeInput<OsdLayout, CrsfLib::LinkStatisticsFrame>
    , public NodeInput<OsdLayout, float>
    , public NodeInput<OsdLayout, std::uint8_t>
    , public NodeInput<OsdLayout, VtxInfo>
{
    static constexpr std::size_t MaxCells =
        OsdWidgetCellCount<CrosshairWidget>
        + OsdWidgetCellCount<HorizonLineWidget>
        + OsdWidgetCellCount<TextWidget<CenterAligned>>
        + OsdWidgetCellCount<LqWidget>
        + OsdWidgetCellCount<RssiWidget>
        + OsdWidgetCellCount<ThrustWidget>
        + OsdWidgetCellCount<BatteryWidget>
        + OsdWidgetCellCount<VtxInfoWidget>
        + OsdWidgetCellCount<AltitudeWidget>;

public:
    explicit OsdLayout(OsdController& controller)
        : m_controller(controller)
    {
        m_mutex = xSemaphoreCreateMutexStatic(&m_mutexBuffer);
        configASSERT(m_mutex != nullptr);
    }

    ~OsdLayout()
    {
        vSemaphoreDelete(m_mutex);
    }

    void Process(const Quaternion<float>& orientation)
    {
        MutexLock lock(m_mutex);
        if (m_horizon)
            m_horizon->Set(orientation);
        else
            m_horizon.emplace(orientation);
        m_dirty = true;
    }

    void Process(const BatteryInfo& battery)
    {
        MutexLock lock(m_mutex);
        if (m_battery)
            m_battery->Set(battery);
        else
            m_battery.emplace(battery);
        m_dirty = true;
    }

    void Process(std::span<const char> flightMode)
    {
        MutexLock lock(m_mutex);
        const auto text = flightMode.empty()
            ? std::string_view{}
            : std::string_view(flightMode.data(), flightMode.size());
        if (m_flightMode)
            m_flightMode->Set(text);
        else
            m_flightMode.emplace(text);
        m_dirty = true;
    }

    void Process(const CrsfLib::LinkStatisticsFrame& linkStatistics)
    {
        MutexLock lock(m_mutex);
        const LqData lqData{linkStatistics.rfProfile, linkStatistics.upLinkQuality};
        if (m_lq)
            m_lq->Set(lqData);
        else
            m_lq.emplace(lqData);
        if (m_rssi)
            m_rssi->Set(linkStatistics.upRssiAnt1);
        else
            m_rssi.emplace(linkStatistics.upRssiAnt1);
        m_dirty = true;
    }

    void Process(float altitude)
    {
        MutexLock lock(m_mutex);
        if (m_altitude)
            m_altitude->Set(altitude);
        else
            m_altitude.emplace(altitude);
        m_dirty = true;
    }

    void Process(std::uint8_t thrust)
    {
        MutexLock lock(m_mutex);
        if (m_thrust)
            m_thrust->Set(thrust);
        else
            m_thrust.emplace(thrust);
        m_dirty = true;
    }

    void Process(const VtxInfo& vtxInfo)
    {
        MutexLock lock(m_mutex);
        if (m_vtxInfo)
            m_vtxInfo->Set(vtxInfo);
        else
            m_vtxInfo.emplace(vtxInfo);
        m_dirty = true;
    }

    void RenderIfPending()
    {
        std::optional<HorizonLineWidget> horizon;
        CrosshairWidget crosshair;
        std::optional<TextWidget<CenterAligned>> flightMode;
        std::optional<LqWidget> lq;
        std::optional<RssiWidget> rssi;
        std::optional<BatteryWidget> battery;
        std::optional<VtxInfoWidget> vtxInfo;
        std::optional<AltitudeWidget> altitude;
        std::optional<ThrustWidget> thrust;
        {
            MutexLock lock(m_mutex);
            if (!m_dirty)
                return;

            m_dirty = false;
            horizon = m_horizon;
            crosshair = m_crosshair;
            flightMode = m_flightMode;
            lq = m_lq;
            rssi = m_rssi;
            battery = m_battery;
            vtxInfo = m_vtxInfo;
            altitude = m_altitude;
            thrust = m_thrust;
        }

        std::array<OsdCell, MaxCells> cells{};
        std::size_t cellCount = 0;

        constexpr auto rows = OsdController::Rows();
        constexpr auto cols = OsdController::Cols();

        if (horizon)
        {
            const auto content = horizon->Render();
            Append(cells, cellCount, content,
                    (rows - static_cast<int16_t>(HorizonLineWidget::VisibleRows)) / 2,
                    (cols - static_cast<int16_t>(HorizonLineWidget::Cols)) / 2);
        }

        if (lq && rssi)
        {
            Append(cells, cellCount, lq->Render(), 0, 0);
            Append(cells, cellCount, rssi->Render(), 1, 0);
        }
        if (thrust)
            Append(cells, cellCount, thrust->Render(), 2, 0);
        if (battery)
            Append(cells, cellCount, battery->Render(), 0, cols - 6);
        if (vtxInfo)
            Append(cells, cellCount, vtxInfo->Render(), 2, cols - 7);
        if (altitude)
            Append(cells, cellCount, altitude->Render(), rows - 2, cols - 7);
        if (flightMode)
            Append(cells, cellCount, flightMode->Render(), rows - 1, 0);

        Append(cells, cellCount, crosshair.Render(), rows / 2, cols / 2);

        m_controller.DrawCells(std::span<const OsdCell>(cells.data(), cellCount));
    }

private:
    class MutexLock
    {
    public:
        explicit MutexLock(SemaphoreHandle_t mutex)
            : m_mutex(mutex)
        {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
        }

        ~MutexLock()
        {
            xSemaphoreGive(m_mutex);
        }

        MutexLock(const MutexLock&) = delete;
        MutexLock& operator=(const MutexLock&) = delete;

    private:
        SemaphoreHandle_t m_mutex;
    };

    template <std::size_t Rows, std::size_t Cols>
    void Append(std::array<OsdCell, MaxCells>& cells, std::size_t& cellCount, const OsdChars<Rows, Cols>& content, int16_t startRow, int16_t startCol)
    {
        for (std::size_t row = 0; row < Rows; ++row)
        {
            const int16_t screenRow = startRow + static_cast<int16_t>(row);
            if (screenRow < 0 || screenRow >= OsdController::Rows())
                continue;

            for (std::size_t col = 0; col < Cols; ++col)
            {
                const int16_t screenCol = startCol + static_cast<int16_t>(col);
                if (screenCol < 0 || screenCol >= OsdController::Cols())
                    continue;

                std::size_t existingIndex = 0;
                while (existingIndex < cellCount && (cells[existingIndex].row != screenRow || cells[existingIndex].col != screenCol))
                    ++existingIndex;

                const auto charCode = static_cast<std::uint8_t>(content[row][col]);
                if (existingIndex < cellCount)
                    cells[existingIndex].charCode = charCode;
                else
                    cells[cellCount++] = {
                        static_cast<std::uint8_t>(screenRow),
                        static_cast<std::uint8_t>(screenCol),
                        charCode
                    };
            }
        }
    }

    OsdController& m_controller;
    std::optional<HorizonLineWidget> m_horizon;
    CrosshairWidget m_crosshair;
    std::optional<TextWidget<CenterAligned>> m_flightMode;
    std::optional<LqWidget> m_lq;
    std::optional<RssiWidget> m_rssi;
    std::optional<BatteryWidget> m_battery;
    std::optional<VtxInfoWidget> m_vtxInfo;
    std::optional<AltitudeWidget> m_altitude;
    std::optional<ThrustWidget> m_thrust;
    SemaphoreHandle_t m_mutex{};
    StaticSemaphore_t m_mutexBuffer{};
    bool m_dirty = true;
};
