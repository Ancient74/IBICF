#pragma once
#include <atomic>
#include <concepts>
#include "Config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Log.h"

template<typename Logic, uint16_t StackDepth, UBaseType_t Priority, TickType_t Period = 0>
class TaskThread final
{
public:
    TaskThread(const char* taskName, Logic& logic)
        : m_taskName(taskName), m_logic(logic)
    {
        m_doneSem = xSemaphoreCreateBinaryStatic(&m_semBuffer);
        configASSERT(m_doneSem != nullptr);
    }

    ~TaskThread()
    {
        Stop();
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

        auto expected = State::Stopped;
        const bool ok = m_state.compare_exchange_strong(expected, State::Starting);
        configASSERT(ok && "Start() called twice / concurrently");
        if (!ok)
            return;

        m_isFinishing.store(false, std::memory_order_relaxed); // allow restart

        m_task = xTaskCreateStatic(&TaskThread::TaskLoop, m_taskName, StackDepth,
                                    this, Priority, m_stack, &m_taskTCB);
        configASSERT(m_task != nullptr);

        m_state.store(State::Running, std::memory_order_release);
        LogInfo(m_taskName, "module started");
    }

    void Stop()
    {
        while (true)
        {
            auto state = m_state.load(std::memory_order_acquire);
            if (state == State::Stopped)
                return;
            if (state == State::Starting || state == State::Stopping)
            {
                vTaskDelay(1);
                continue;
            }

            if (m_state.compare_exchange_weak(state, State::Stopping))
                break;
        }
        configASSERT(xTaskGetCurrentTaskHandle() != m_task);
        m_isFinishing.store(true, std::memory_order_release);
        xSemaphoreTake(m_doneSem, portMAX_DELAY);
        vTaskDelete(m_task);
        m_task = nullptr;
        LogInfo(m_taskName, "module stopped");
        m_state.store(State::Stopped, std::memory_order_release);
    }

    TaskHandle_t GetTaskHandle()
    {
        return m_task;
    }

private:
    enum class State : uint8_t
    {
        Stopped,
        Starting,
        Running,
        Stopping
    };

    static void TaskLoop(void* arg)
    {
        auto* self = static_cast<TaskThread*>(arg);

        TickType_t lastWakeTime = xTaskGetTickCount();
#ifdef DEBUG
        TickType_t lastHeartbeatTime = lastWakeTime;
        constexpr TickType_t heartbeatInterval = pdMS_TO_TICKS(Config::Task::HEARTBEAT_INTERVAL_MS);
#endif

        while (!self->m_isFinishing.load(std::memory_order_acquire))
        {
#ifdef DEBUG
            if ((xTaskGetTickCount() - lastHeartbeatTime) >= heartbeatInterval)
            {
                LogHeartbeat(self->m_taskName);
                lastHeartbeatTime = xTaskGetTickCount();
            }
#endif
            self->m_logic.RunTaskIteration();
            if constexpr (Period > 0)
                vTaskDelayUntil(&lastWakeTime, Period);
        }

        xSemaphoreGive(self->m_doneSem);
        vTaskSuspend(nullptr);
    }

    const char* m_taskName;
    Logic& m_logic;

    TaskHandle_t m_task{};
    SemaphoreHandle_t m_doneSem{};
    std::atomic_bool m_isFinishing = false;
    std::atomic<State> m_state{State::Stopped};

    alignas(portBYTE_ALIGNMENT) StackType_t m_stack[StackDepth];
    StaticTask_t m_taskTCB{};
    StaticSemaphore_t m_semBuffer{};
};
