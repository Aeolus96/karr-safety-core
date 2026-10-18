#pragma once

// ============================================================================
// Synchronized handoff between the control task (core 0) and the ROS task
// (loop/core 1):
//  - Snapshot queue (length one): control task publishes; loop() consumes.
//    Synchronized FreeRTOS queue with a zero-wait producer: the control task
//    never blocks on the ROS task; the ROS task may block, and its next
//    consumed snapshot is then the newest. The control task is the sole
//    writer; the ROS task is the sole reader.
//  - Agent-availability atomic flag: written by loop() after bridge
//    servicing, read by control with acquire/release ordering.
// Queue handle and atomic are private to state_exchange.cpp.
// ============================================================================

#include "karr/control_snapshot.h"

namespace karr
{

// Creates the snapshot queue. Returns false on allocation failure (caller
// decides the boot policy).
bool initializeSnapshotQueue();

// Length-one latest-value: overwrite replaces the queued item even when
// full, so the consumer always gets the newest snapshot. Zero-wait for
// the producer; ROS may block — its next consumed snapshot is current.
void publishSnapshot(const ControlSnapshot &snap);

// Zero-wait receive. Returns false when no snapshot is queued (or the
// queue does not exist).
bool receiveSnapshot(ControlSnapshot &snap);

// Agent availability handoff (loop() stores after bridge servicing;
// control reads before each feature evaluation).
void storeAgentConnected(bool connected); // memory_order_release
bool loadAgentConnected();                // memory_order_acquire

} // namespace karr
