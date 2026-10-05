#pragma once
#include <atomic>
#include <concepts>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

template<typename Logic, uint16_t StackDepth, UBaseType_t Priority, bool Periodic = true>
class TaskThread
{
public:
    TaskThread(const char* taskName, TickType_t period, Logic& logic)
        : m_taskName(taskName), m_period(period), m_logic(logic) {}

    ~TaskThread()
    {
        configASSERT(!m_started.load(std::memory_order_acquire) &&
                     "Owner must call Stop() before its own members are torn down");
    }

    TaskThread(const TaskThread&) = delete;
    TaskThread& operator=(const TaskThread&) = delete;
    TaskThread(TaskThread&&) = delete;
    TaskThread& operator=(TaskThread&&) = delete;

    const char* GetName() const { return m_taskName; }

    void Start()
    {
        static_assert(requires(Logic& l) {
            { l.RunTaskIteration() } -> std::same_as<void>;
        }, "Logic must provide void RunTaskIteration()");

        bool expected = false;
        configASSERT(m_started.compare_exchange_strong(expected, true) &&
                     "Start() called twice / concurrently");

        m_isFinishing.store(false, std::memory_order_relaxed); // allow restart

        m_doneSem = xSemaphoreCreateBinaryStatic(&m_semBuffer);
        configASSERT(m_doneSem != nullptr);

        m_task = xTaskCreateStatic(&TaskThread::TaskLoop, m_taskName, StackDepth,
                                    this, Priority, m_stack, &m_taskTCB);
        configASSERT(m_task != nullptr);
    }

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
        auto* self = static_cast<TaskThread*>(arg);

        TickType_t lastWakeTime = xTaskGetTickCount();
        while (!self->m_isFinishing.load(std::memory_order_acquire))
        {
            self->m_logic.RunTaskIteration();
            if constexpr (Periodic)
                vTaskDelayUntil(&lastWakeTime, self->m_period);
        }

        xSemaphoreGive(self->m_doneSem);
        vTaskDelete(nullptr);
    }

    const char* m_taskName;
    TickType_t m_period;
    Logic& m_logic;

    TaskHandle_t m_task{};
    SemaphoreHandle_t m_doneSem{};
    std::atomic_bool m_isFinishing = false;
    std::atomic_bool m_started = false;

    alignas(portBYTE_ALIGNMENT) StackType_t m_stack[StackDepth]{};
    StaticTask_t m_taskTCB{};
    StaticSemaphore_t m_semBuffer{};
};
