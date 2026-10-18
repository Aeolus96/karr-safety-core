#pragma once

// ============================================================================
// Queued log event channel: control task enqueues with zero wait.
// Control-task reachable code (RC service, feature evaluation, I2C writes)
// enqueues events here. loop() on core 1 drains the queue (bounded per
// iteration — see drainLogQueue) and writes Serial.
// Zero-wait send: if the queue fills, events are dropped and the counter
// increments. Logging never blocks the control path. The dropped counter
// is an atomic: written by control, snapshotted by loop() for diagnostics.
// Queue handle + dropped counter are private to diagnostics.cpp.
// ============================================================================

#include <cstdint>

namespace karr
{

enum class LogCategory : uint8_t
{
    None = 0,
    RcReceiver = 1,
    RelayBoard = 2,
    FeatureController = 3,
    I2c = 4,
};

struct LogEvent
{
    uint32_t timestamp_ms;
    uint8_t category;
    uint8_t code;
    uint32_t value;
};

// Creates the log queue. Returns false on allocation failure (caller decides
// the boot policy — normal operation must not start without logging).
bool initializeLogQueue();

// Zero-wait enqueue — safe from the control task; never blocks on Serial.
// No-op while the queue does not exist.
void enqueueLog(LogCategory category, uint8_t code, uint32_t value);

// Bounded drain: at most DRAIN_MAX_EVENTS per loop() iteration so a
// sustained event burst cannot indefinitely postpone bridge servicing.
// Returns the number of events drained.
uint32_t drainLogQueue();

// Atomic dropped-event counter snapshot (diagnostics only).
uint32_t logDroppedCount();

} // namespace karr
