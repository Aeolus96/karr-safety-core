#pragma once

// ============================================================================
// RcReceiver — Turnigy iA6B receiver (TGY-i6S transmitter) over i-BUS +
// the receiver-facing safety state machine. Contains NO ROS types and NO
// board hardware access — its only library dependency is IBusBM.
//
// RECEIVER BEHAVIOR TABLE (stop conditions, grace, permission; matches
// RcReceiver::service and NOTES.md "Failsafe detection"):
//   valid endpoint (LOW/HIGH)        -> own relay request updates next ~20 ms eval
//   brief intermediate                -> retain last valid endpoint, no global
//                                        cutoff, bounded by INTERMEDIATE_GRACE_MS
//   3-pos MID-zone rest past grace    -> effective value resolves to MID
//                                        (INACTIVE); never failsafe evidence
//                                        and never an invalid-input stop
//   narrow signature in fresh frames
//     for FAILSAFE_CONFIRM_MS         -> confirmed failsafe for that channel
//   valid CH6 LOW                     -> stop at next control evaluation
//                                        (no 200 ms wait)
//   valid CH6 HIGH                    -> release switch-stop (other permissions
//                                        may still apply)
//   CH6 intermediate (no endpoint)    -> previous decoded stop retained, at
//                                        most INTERMEDIATE_GRACE_MS
//   CH6 implausible / no prior state  -> stop, no grace (boots stopped)
//   2-pos intermediate past grace     -> RECV_STOP_INVALID_INPUT stop
//   no fresh frames for timeout       -> RECV_STOP_SIGNAL_LOSS stop
//   one channel clearing              -> never clears other channels, CH6
//                                        stop, invalid input, or frame loss
// Permission (isLocallyInhibited / /relay_board/estop / LED) reads ONLY the
// authoritative receiver-local stop; brief intermediate or unconfirmed
// signature input NEVER inhibits by itself.
// ============================================================================

#include <cstdint>
// IBusBM.h uses NULL without including a header that defines it — provide
// it here so this header stays self-contained (previously Arduino.h was
// included first in the single-file firmware).
#include <stddef.h>

#include <IBusBM.h>

#include "karr/rc_config.h"

namespace karr
{

// --- Receiver link + failsafe detection (pin shared with app boot) ---
// CRITICAL DECISION — receive pin: the tested prototype called
// Serial1.begin(..., RX=21) and then IBusBM.begin(Serial1, NOTIMER, rxPin=1),
// which re-begins UART1 internally — so the pin that actually ran was GPIO1.
// We reproduce the effective tested behavior; change to 21 if your wire
// lands there instead.
constexpr int RC_RECEIVE_PIN = 1;

// Receiver-local stop reason bits (snapshot + logs; receiver scope only —
// board/agent/config reasons stay distinguishable, see service()).
constexpr uint8_t RECV_STOP_SIGNAL_LOSS = 1u;
constexpr uint8_t RECV_STOP_FAILSAFE = 2u;
constexpr uint8_t RECV_STOP_CH6 = 4u;
constexpr uint8_t RECV_STOP_INVALID_INPUT = 8u;

// --- Receiver operating state -----------------------------------------------
// SignalActive is the receiver-liveness prerequisite for RC-driven
// features; it is NOT permission on its own — isLocallyInhibited() adds
// CONFIRMED failsafe, decoded CH6 stop, and persistent invalid input.
enum class ReceiverOperatingState
{
    SignalLost,       // no frames, or e-stop channel below "no signal" value
    FailsafeEngaged,  // confirmed receiver failsafe signature (per-channel 200 ms)
    SignalActive,     // healthy operation
};

// Per-channel decoded endpoint from one fresh frame.
enum class ChannelDecode : uint8_t
{
    Low,              // valid LOW endpoint  (raw < SWITCH_LOW_THRESHOLD)
    High,             // valid HIGH endpoint (raw > SWITCH_HIGH_THRESHOLD)
    Intermediate,     // no valid endpoint, inside plausible range
    Implausible,      // outside 900..2100 — never valid input
};

class RcReceiver
{
public:
    void begin(int receive_pin);

    // Pump the i-BUS byte state machine + refresh link/failsafe state.
    // MUST run every loop iteration (protocol byte-gap timing).
    void service(uint32_t now_ms);

    ReceiverOperatingState state() const { return state_; }
    // Frame state machine: TRUE = fresh frames AND no CONFIRMED failsafe
    // signature. This is liveness only — a decoded CH6 stop or persistent
    // invalid input can still stop operation (see isLocallyInhibited), so
    // "active" is NOT permission to operate.
    bool isSignalActive() const { return state_ == ReceiverOperatingState::SignalActive; }
    // Persisted CONFIRMED receiver-failsafe state (narrow signature window
    // present in fresh frames for FAILSAFE_CONFIRM_MS) — NOT the wireless
    // e-stop switch position (see decodedCh6Stop()) and NOT every stop
    // condition.
    bool isFailsafeEngaged() const { return state_ == ReceiverOperatingState::FailsafeEngaged; }

    // WIRELESS E-STOP: the dedicated e-stop switch (ch6), DECODED with
    // intermediate grace. ENGAGED on valid LOW. A valid HIGH releases it.
    // Brief intermediate input preserves the previous decoded stop state for
    // at most INTERMEDIATE_GRACE_MS (timer starts on ENTRY, never restarts
    // on further intermediate samples). No prior valid state, an implausible
    // value, or grace expiry all read ENGAGED — only a deliberate valid HIGH
    // releases the e-stop. Boots ENGAGED (raw starts at 0).
    bool isEstopSwitchEngaged() const { return decoded_ch6_stop_; }

    // ONE predicate for ALL receiver-local inhibition: signal loss,
    // CONFIRMED failsafe, decoded CH6 stop, or persistent invalid switch
    // input. Brief intermediate/ambiguous input NEVER inhibits by itself —
    // only confirmation, valid-LOW CH6, frame timeout, or grace expiry does.
    // The RC-feature permission check, /relay_board/estop and the LED
    // policy all consult this so they can never disagree about
    // receiver-local state. SCOPE: it speaks ONLY for the receiver — not
    // for board-wide relay state, and not for future non-RC features.
    bool isLocallyInhibited() const { return receiver_local_stop_; }

    // Authoritative receiver-local stop + reason mask, computed once per
    // service() — copied into the snapshot, published directly by ROS.
    // Receiver scope only; board/agent/config reasons stay separate.
    bool receiverLocalStop() const { return receiver_local_stop_; }
    uint8_t receiverStopReasons() const { return receiver_stop_reasons_; }

    // TRUE while any configured 2-position channel's signature confirmation
    // is RUNNING (narrow window, not yet confirmed) — informational only,
    // never a permission input.
    bool isSignatureConfirming() const { return signature_confirming_mask_ != 0; }

    uint16_t rawChannel(uint8_t index) const { return raw_channels_[index]; }
    const ReceiverChannelConfiguration &channelConfiguration(uint8_t index) const
    {
        return RC_CHANNEL_CONFIGURATION[index];
    }
    uint8_t channelCount() const { return RC_CHANNEL_COUNT; }
    uint32_t lastValidFrameMs() const { return last_valid_frame_ms_; }
    uint8_t lastFrameCounter() const { return last_frame_counter_; }
    // 16-bit mask: bit N = channel N inside the narrow signature window on
    // the latest processed frame (informational; confirmation decides).
    uint16_t signatureMask() const { return latest_frame_signature_mask_; }
    // Effective switch value for feature requests: the retained last valid
    // endpoint while a channel is briefly intermediate, else the live raw.
    // Bounded by INTERMEDIATE_GRACE_MS per channel — see service(). For
    // 3-pos channels a sustained MID-zone rest resolves to canonical MID
    // (1500, INACTIVE) at the same bound instead of retaining forever.
    uint16_t effectiveChannel(uint8_t index) const { return effective_channels_[index]; }

private:
    // Decode one raw sample into an endpoint category (definitions in the
    // .cpp, next to the thresholds they use).
    static ChannelDecode decodeRaw(uint16_t raw_value);
    // TRUE while raw sits inside the narrow failsafe signature window.
    static bool inSignatureWindow(uint16_t raw_value);

    IBusBM ibus_protocol_;
    ReceiverOperatingState state_ = ReceiverOperatingState::SignalLost;
    ReceiverOperatingState previous_state_ = ReceiverOperatingState::SignalLost;
    uint16_t raw_channels_[RC_CHANNEL_COUNT] = {0};
    // Retained last valid endpoint per channel (LOW or HIGH raw value) for
    // brief-intermediate feature requests. Updated only on valid endpoints.
    uint16_t effective_channels_[RC_CHANNEL_COUNT] = {0};
    uint8_t last_frame_counter_ = 0;
    uint32_t last_frame_seen_ms_ = 0;
    uint32_t last_valid_frame_ms_ = 0; // time of last valid decoded frame
    uint16_t latest_frame_signature_mask_ = 0; // channels in narrow window, latest frame
    uint16_t signature_confirming_mask_ = 0;   // confirmations currently running
    // Per-channel signature confirmation: timer starts on ENTRY into the
    // narrow window, never restarts on further signature samples; leaving
    // the window resets it. Latched confirmed flags clear only on fresh
    // VALID replacement evidence for that channel (a valid endpoint) —
    // moving to non-signature intermediate does NOT clear confirmation and
    // does NOT grant permission (recovery correction C).
    uint32_t signature_since_ms_[RC_CHANNEL_COUNT] = {0};
    bool signature_timer_active_[RC_CHANNEL_COUNT] = {false};
    bool signature_confirmed_[RC_CHANNEL_COUNT] = {false};
    // Per-channel intermediate episode: timer starts on ENTRY into
    // intermediate (no valid endpoint), never restarts on further
    // intermediate samples or on signature/other category changes. A valid
    // endpoint ends the episode. Past INTERMEDIATE_GRACE_MS with no valid
    // endpoint -> persistent invalid input fault for that channel.
    uint32_t intermediate_since_ms_[RC_CHANNEL_COUNT] = {0};
    bool intermediate_active_[RC_CHANNEL_COUNT] = {false};
    bool invalid_input_latched_[RC_CHANNEL_COUNT] = {false};
    // Decoded CH6 stop state with intermediate grace. Valid LOW engages
    // immediately; valid HIGH releases; intermediate preserves the previous
    // decoded state for at most INTERMEDIATE_GRACE_MS; implausible engages
    // without grace; no prior valid state reads ENGAGED (boots stopped).
    bool decoded_ch6_stop_ = true;
    uint32_t ch6_intermediate_since_ms_ = 0;
    bool ch6_intermediate_active_ = false;
    // Authoritative receiver-local stop + reasons, recomputed every
    // service() from confirmed evidence only.
    bool receiver_local_stop_ = true;
    uint8_t receiver_stop_reasons_ = RECV_STOP_SIGNAL_LOSS;
};

} // namespace karr
