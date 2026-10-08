#include "Logger.h"
#include "projdefs.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "queue.h"
#include "usbd_def.h"

extern "C" void LoggerTransmitCompleteFromISR()
{
    if (auto* logger = Logger::GetInstance())
        logger->TransmitCompleteFromISR();
}

extern volatile uint8_t CdcDtr;

#ifdef DEBUG
bool SubmitLog(const LogMessage& message)
{
    return Logger::GetInstance()->Log(message);
}
#endif

namespace
{
    void WaitForUsb()
    {
        while (!CdcDtr)
            HAL_Delay(10);
    }
}

Logger::Logger(const char* taskName)
    : m_thread(taskName, *this)
{
    if (s_instance)
        return;
    s_instance = this;

    WaitForUsb();

    m_queue = xQueueCreateStatic(Config::LOGGER_QUEUE_SIZE, sizeof(LogMessage), m_queueStorage, &m_queueBuffer);
    configASSERT(m_queue != nullptr);
    m_txCompleteSemaphore = xSemaphoreCreateBinaryStatic(&m_txCompleteSemaphoreBuffer);
    configASSERT(m_txCompleteSemaphore != nullptr);
    m_thread.Start();
}

Logger* Logger::GetInstance()
{
    return s_instance;
}

bool Logger::Log(const LogMessage& message)
{
    return xQueueSend(m_queue, &message, 0) == pdTRUE;
}

void Logger::TransmitCompleteFromISR()
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(m_txCompleteSemaphore, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void Logger::RunTaskIteration()
{
    if (!m_pendingMessage.has_value())
    {
        LogMessage message{};
        if (xQueueReceive(m_queue, &message, Config::LOGGER_QUEUE_TIMEOUT_MS) != pdTRUE)
            return;
        m_pendingMessage = message;
    }

    if (!m_txInFlight)
    {
        uint16_t attempt = 0;
        auto& message = *m_pendingMessage;
        uint8_t result = USBD_BUSY;
        while ((result = CDC_Transmit_FS(reinterpret_cast<uint8_t*>(&message), sizeof(message))) == USBD_BUSY)
        {
            if (attempt++ >= Config::LOGGER_BUSY_RETRY_ATTEMPTS)
                return;
            vTaskDelay(pdMS_TO_TICKS(Config::LOGGER_BUSY_RETRY_INTERVAL_MS));
        }
        if (result != USBD_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(Config::LOGGER_BUSY_RETRY_INTERVAL_MS));
            return;
        }
        m_txInFlight = true;
    }

    if (xSemaphoreTake(m_txCompleteSemaphore, pdMS_TO_TICKS(Config::LOGGER_QUEUE_TIMEOUT_MS)) == pdTRUE)
    {
        m_pendingMessage = std::nullopt;
        m_txInFlight = false;
    }
}
