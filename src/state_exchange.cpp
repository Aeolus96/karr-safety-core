#include "karr/state_exchange.h"

#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace karr
{

static QueueHandle_t g_snapshot_queue = NULL;
static std::atomic<bool> g_agent_connected{false};

bool initializeSnapshotQueue()
{
    // Coherent telemetry snapshot queue (length one, overwrite latest).
    // Control task publishes; loop() on core 1 consumes.
    g_snapshot_queue = xQueueCreate(1, sizeof(ControlSnapshot));
    return g_snapshot_queue != NULL;
}

void publishSnapshot(const ControlSnapshot &snap)
{
    if (!g_snapshot_queue)
    {
        return;
    }
    // Length-one latest-value: overwrite replaces the queued item even when
    // full, so the consumer always gets the newest snapshot. Zero-wait for
    // the producer; ROS may block — its next consumed snapshot is current.
    xQueueOverwrite(g_snapshot_queue, &snap);
}

bool receiveSnapshot(ControlSnapshot &snap)
{
    if (!g_snapshot_queue)
    {
        return false;
    }
    return xQueueReceive(g_snapshot_queue, &snap, 0) == pdTRUE;
}

void storeAgentConnected(bool connected)
{
    g_agent_connected.store(connected, std::memory_order_release);
}

bool loadAgentConnected()
{
    return g_agent_connected.load(std::memory_order_acquire);
}

} // namespace karr
