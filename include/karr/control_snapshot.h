#pragma once

#include <cstdint>

// ============================================================================
// Coherent telemetry snapshot: control task fills once per safety evaluation
// cycle (20 ms); loop() (core 1) copies it for publishing / status.
// This replaces live cross-core reads of RcReceiver / RelayBoard.
// Value data only — no pointers to subsystem internals, no queue or ROS
// handles. Handoff mechanics live in state_exchange.h.
// ============================================================================

namespace karr
{

struct ControlSnapshot
{
    uint32_t captured_ms;
    uint32_t frame_sequence;
    uint32_t last_valid_frame_ms;
    uint16_t channels[16];     // RC_CHANNEL_COUNT is 10; IBusBM supports 14
    uint16_t signature_mask;   // 16-bit: bit N = channel N in narrow window
    uint8_t receiver_state;    // ReceiverOperatingState
    bool receiver_local_stop;      // authoritative stop, computed in control
    uint8_t receiver_stop_reasons; // RECV_STOP_* bits (receiver scope only)
    uint8_t desired_mask;          // feature-layer target (relay-output mask)
    uint8_t last_written_mask; // relay-output mask last written to hardware
    uint8_t digital_inputs;
    bool relay_state_known;
};

} // namespace karr
