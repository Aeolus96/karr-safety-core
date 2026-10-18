#include "karr/diagnostics.h"

#include <Arduino.h>
#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace karr
{

static QueueHandle_t g_log_queue = NULL;
static std::atomic<uint32_t> g_log_dropped_count{0};

bool initializeLogQueue()
{
    g_log_queue = xQueueCreate(16, sizeof(LogEvent));
    return g_log_queue != NULL;
}

void enqueueLog(LogCategory category, uint8_t code, uint32_t value)
{
    if (!g_log_queue)
    {
        return;
    }
    LogEvent event{millis(), static_cast<uint8_t>(category), code, value};
    if (xQueueSend(g_log_queue, &event, 0) != pdTRUE)
    {
        g_log_dropped_count.fetch_add(1, std::memory_order_relaxed);
    }
}

uint32_t drainLogQueue()
{
    if (!g_log_queue)
    {
        return 0;
    }
    constexpr uint8_t DRAIN_MAX_EVENTS = 16;
    uint32_t drained = 0;
    LogEvent event;
    while (drained < DRAIN_MAX_EVENTS && xQueueReceive(g_log_queue, &event, 0) == pdTRUE)
    {
        const char *cat_text = "?";
        switch (static_cast<LogCategory>(event.category))
        {
        case LogCategory::RcReceiver:
            cat_text = "RC";
            break;
        case LogCategory::RelayBoard:
            cat_text = "RELAY";
            break;
        case LogCategory::FeatureController:
            cat_text = "SAFETY";
            break;
        case LogCategory::I2c:
            cat_text = "I2C";
            break;
        default:
            break;
        }
        Serial.printf("[LOG %lu %s c=%u v=%lu]\n",
                      (unsigned long)event.timestamp_ms, cat_text,
                      event.code, (unsigned long)event.value);
        ++drained;
    }
    return drained;
}

uint32_t logDroppedCount()
{
    return g_log_dropped_count.load(std::memory_order_relaxed);
}

} // namespace karr
