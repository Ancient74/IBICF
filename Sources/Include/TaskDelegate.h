#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include "Config.h"

class TaskDelegate
{
public:
    using Fn = void (*)(void*);

    static constexpr uint32_t WakeBit = 1u << 0;

    TaskDelegate() : m_owner(xTaskGetCurrentTaskHandle())
    {
        m_queue = xQueueCreateStatic(Config::Task::DELEGATE_TASK_DEPTH, sizeof(Job), m_storage, &m_queueBuffer);
        configASSERT(m_queue != nullptr);
    }

    TaskDelegate(const TaskDelegate &) = delete;
    TaskDelegate &operator=(const TaskDelegate &) = delete;

    void Bind()
    {
        m_owner = xTaskGetCurrentTaskHandle();
    }

    bool PostToOwner(Fn fn, void* arg = nullptr, TickType_t timeout = 0)
    {
        if (fn == nullptr)
            return false;
        return Enqueue(Job{fn, arg, nullptr, 0, false}, timeout);
    }

    bool PostDelayed(Fn fn, void*arg, TickType_t delayTicks, TickType_t timeout = 0)
    {
        if (fn == nullptr)
            return false;
        if (delayTicks == 0)
            return PostToOwner(fn, arg, timeout);
        return Enqueue(Job{fn, arg, nullptr, Now() + delayTicks, true}, timeout);
    }

    template <typename T, void (T::*Method)()>
    bool PostMember(T* obj, TickType_t timeout = 0)
    {
        return PostToOwner(&Thunk<T, Method>, obj, timeout);
    }

    template <typename T, void (T::*Method)()>
    bool PostMemberDelayed(T* obj, TickType_t delayTicks, TickType_t timeout = 0)
    {
        return PostDelayed(&Thunk<T, Method>, obj, delayTicks, timeout);
    }

    bool RunOnOwner(Fn fn, void* arg = nullptr, TickType_t enqueueTimeout = portMAX_DELAY)
    {
        if (fn == nullptr || InIsr())
            return false;

        if (m_owner != nullptr && xTaskGetCurrentTaskHandle() == m_owner)
        {
            fn(arg);
            return true;
        }

        StaticSemaphore_t semBuf;
        SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&semBuf);

        if (!Enqueue(Job{fn, arg, done, 0, false}, enqueueTimeout))
        {
            vSemaphoreDelete(done);
            return false;
        }

        xSemaphoreTake(done, portMAX_DELAY);
        vSemaphoreDelete(done);
        return true;
    }

    bool Process()
    {
        if (m_owner == nullptr)
            Bind();

        bool any = false;
        while (true)
        {
            bool progress = false;

            Job job;
            while (xQueueReceive(m_queue, &job, 0) == pdTRUE)
            {
                progress = true;
                if (job.delayed && !IsDue(job.due))
                {
                    Park(job);
                }
                else
                {
                    Execute(job);
                    any = true;
                }
            }

            if (RunDueTimers())
            {
                progress = true;
                any = true;
            }

            if (!progress)
                break;
        }
        return any;
    }

    bool WaitAndProcess(TickType_t ticks = portMAX_DELAY)
    {
        if (m_owner == nullptr)
            Bind();

        bool notified = false;
        if (uxQueueMessagesWaiting(m_queue) == 0)
        {
            TickType_t wait = ticks;
            const TickType_t next = NextTimerDelay();
            if (next < wait)
                wait = next;

            if (wait != 0)
            {
                uint32_t bits = 0;
                notified = (xTaskNotifyWait(0, WakeBit, &bits, wait) == pdTRUE);
            }
        }

        const bool worked = Process();
        return notified || worked;
    }

    uint32_t Dropped() const
    {
        return m_dropped;
    }

private:
    using SignedTick = typename std::make_signed<TickType_t>::type;

    struct Job
    {
        Fn fn;
        void*arg;
        SemaphoreHandle_t done;
        TickType_t due;
        bool delayed;
    };

    template <typename T, void (T::*Method)()>
    static void Thunk(void* p)
    {
        (static_cast<T*>(p)->*Method)();
    }

    static bool InIsr()
    {
        return __get_IPSR() != 0;
    }

    static TickType_t Now()
    {
        return InIsr() ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
    }

    static bool IsDue(TickType_t due, TickType_t now)
    {
        return static_cast<SignedTick>(now - due) >= 0;
    }

    static bool IsDue(TickType_t due)
    {
        return IsDue(due, xTaskGetTickCount());
     }

    static void Execute(const Job& job)
    {
        job.fn(job.arg);
        if (job.done != nullptr)
            xSemaphoreGive(job.done);
    }

    void Park(const Job& job)
    {
        for (auto& slot : m_timers)
        {
            if (slot.fn == nullptr)
            {
                slot = job;
                return;
            }
        }
        ++m_dropped;
        configASSERT(false); // timer list full: increase DELEGATE_TASK_SLOTS
    }

    bool RunDueTimers()
    {
        bool ran = false;
        while (true)
        {
            const TickType_t now = xTaskGetTickCount();
            Job* best = nullptr;
            for (auto& slot : m_timers)
            {
                if (slot.fn == nullptr || !IsDue(slot.due, now))
                    continue;
                if (best == nullptr ||
                    static_cast<SignedTick>(slot.due - best->due) < 0)
                    best = &slot;
            }
            if (best == nullptr)
                return ran;

            const Job job = *best;
            best->fn = nullptr;
            Execute(job);
            ran = true;
        }
    }

    TickType_t NextTimerDelay() const
    {
        const TickType_t now = xTaskGetTickCount();
        bool found = false;
        TickType_t best = 0;
        for (const auto& slot : m_timers)
        {
            if (slot.fn == nullptr)
                continue;
            const TickType_t rem = IsDue(slot.due, now) ? 0 : slot.due - now;
            if (!found || rem < best)
            {
                best = rem;
                found = true;
            }
        }
        return found ? best : portMAX_DELAY;
    }

    bool Enqueue(const Job& job, TickType_t timeout)
    {
        if (InIsr())
        {
            BaseType_t woken = pdFALSE;
            if (xQueueSendFromISR(m_queue, &job, &woken) != pdTRUE)
                return false;
            if (m_owner != nullptr)
                xTaskNotifyFromISR(m_owner, WakeBit, eSetBits, &woken);
            portYIELD_FROM_ISR(woken);
            return true;
        }

        if (xQueueSend(m_queue, &job, timeout) != pdTRUE)
            return false;
        if (m_owner != nullptr)
            xTaskNotify(m_owner, WakeBit, eSetBits);
        return true;
    }

    volatile TaskHandle_t m_owner;
    QueueHandle_t m_queue = nullptr;
    StaticQueue_t m_queueBuffer;
    uint8_t m_storage[Config::Task::DELEGATE_TASK_DEPTH * sizeof(Job)];
    Job m_timers[Config::Task::DELEGATE_TASK_SLOTS] = {};
    uint32_t m_dropped = 0;
};
