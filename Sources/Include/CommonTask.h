#pragma once

#include <atomic>
#include <concepts>

#include "FreeRTOS.h"
#include "semphr.h"

template<typename Derived, uint16_t StackDepth, UBaseType_t Priority, bool Periodic = true>
class CommonTask
{
public:
    CommonTask(const char* taskName, TickType_t period)
        : m_taskName(taskName), m_period(period) {}

    ~CommonTask()
    {
        configASSERT(!m_started.load(std::memory_order_acquire) &&
                     "Derived::~Derived() must call Stop() before returning");
        Stop();
    }

    CommonTask(const CommonTask&) = delete;
    CommonTask& operator=(const CommonTask&) = delete;

    void Start()
    {
        bool expected = false;
        configASSERT(m_started.compare_exchange_strong(expected, true) &&
                     "Start() called twice / concurrently");

        static_assert(requires(Derived& task) {
            { task.RunTaskIteration() } -> std::same_as<void>;
        });

        m_isFinishing.store(false, std::memory_order_relaxed); // allow restart
        configASSERT((m_doneSem = xSemaphoreCreateBinary()) != nullptr);
        configASSERT(xTaskCreate(&CommonTask::TaskLoop, m_taskName, StackDepth, this, Priority, &m_task) == pdPASS);
    }

protected:
    void Stop()
    {
        bool expected = true;
        if (!m_started.compare_exchange_strong(expected, false))
            return;

        m_isFinishing.store(true, std::memory_order_release);
        xSemaphoreTake(m_doneSem, portMAX_DELAY);
        vSemaphoreDelete(m_doneSem);
        m_doneSem = nullptr;
        m_task = nullptr;
    }

private:
    static void TaskLoop(void* arg)
    {
        auto* self = static_cast<CommonTask*>(arg);
        auto* derived = static_cast<Derived*>(self);

        TickType_t lastWakeTime = xTaskGetTickCount();
        while (!self->m_isFinishing.load(std::memory_order_acquire))
        {
            derived->RunTaskIteration();
            if constexpr (Periodic)
                vTaskDelayUntil(&lastWakeTime, self->m_period);
        }

        xSemaphoreGive(self->m_doneSem);
        vTaskDelete(nullptr);
    }

    const char* m_taskName;
    TickType_t m_period;
    TaskHandle_t m_task{};
    SemaphoreHandle_t m_doneSem{};
    std::atomic_bool m_isFinishing{false};
    std::atomic_bool m_started{false};
};
