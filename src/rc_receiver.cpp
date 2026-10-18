#include "karr/rc_receiver.h"

#include <Arduino.h>

#include "karr/diagnostics.h"

namespace karr
{

// Receiver-private thresholds and timing constants.
namespace
{

constexpr uint32_t RECEIVER_FRAME_TIMEOUT_MS = 200; // i-BUS frames arrive every
                                                    // ~7-14 ms; none for 200 ms
                                                    // = receiver is gone

// CRITICAL DECISION — failsafe signature, scoped to the TESTED setup (not
// a universal i-BUS failsafe property): the tested transmitter
// configuration outputs 1500 on ALL channels when its failsafe fires
// (observed on the i6S/iA6B pair). For a 2-position switch 1500 is IMPOSSIBLE
// as a steady healthy value (it only emits 1000 or 2000, briefly passing
// through the middle while bouncing). A configured 2-position channel inside
// the NARROW signature window starts per-channel confirmation; only
// confirmation (or a valid CH6 LOW, or frame timeout, or persistent invalid
// input) inhibits RC-dependent operation. Brief transitions never cut
// outputs. Joysticks and 3-position switches are excluded — 1500 is
// legitimate there.
// IBusBM keeps delivering receiver frames after RF loss, so the frame
// counter alone cannot replace configured failsafe-value detection.
constexpr uint16_t NO_SIGNAL_RAW_VALUE = 980;   // below = receiver unbound
// Switch endpoint thresholds (EXCLUSIVE boundaries, unchanged semantics):
// valid LOW endpoint is raw < SWITCH_LOW_THRESHOLD; valid HIGH endpoint is
// raw > SWITCH_HIGH_THRESHOLD. These drive mapping/activation only — they
// are deliberately separate from the failsafe signature window below.
constexpr uint16_t SWITCH_LOW_THRESHOLD = 1250;
constexpr uint16_t SWITCH_HIGH_THRESHOLD = 1750;
// Narrow receiver-failsafe signature window (PROVISIONAL bench-test values,
// not a protocol guarantee): 1490..1510. Verify transmitter-off behavior,
// startup, and reconnection before treating the tolerance as committed.
constexpr uint16_t FAILSAFE_CENTER = 1500;
constexpr uint16_t FAILSAFE_TOLERANCE = 10;
constexpr uint32_t FAILSAFE_CONFIRM_MS = 200;     // signature confirmation
// Intermediate-input grace (PROVISIONAL, separately named policy): a channel
// showing no valid endpoint retains its previous decoded state for at most
// this long. Same initial value as FAILSAFE_CONFIRM_MS but a different
// policy — do not merge them.
constexpr uint32_t INTERMEDIATE_GRACE_MS = 200;

// Switch ZONE boundaries (mapping/activation only — NOT failsafe detection).
//   LOW zone  : raw <  SWITCH_LOW_THRESHOLD   (position 1000)
//   MID zone  : SWITCH_LOW_THRESHOLD .. SWITCH_HIGH_THRESHOLD (reads INACTIVE)
//   HIGH zone : raw >  SWITCH_HIGH_THRESHOLD  (position 2000)
// The narrow FAILSAFE_CENTER +/- FAILSAFE_TOLERANCE window is the only
// signature evidence; ordinary MID samples are intermediate input.

} // namespace

// --- Channel configuration table ---------------------------------------------
// >>> EDIT THE ROWS BELOW to match your transmitter / mode setup <<<
// (Declared in karr/rc_config.h; parsed by docs/diagrams/gen_rc_map.py.)
extern const ReceiverChannelConfiguration RC_CHANNEL_CONFIGURATION[RC_CHANNEL_COUNT] = {
    {"right_stick_vertical", ChannelKind::AnalogAxis, false},         // ch1 elevator
    {"right_stick_horizontal", ChannelKind::AnalogAxis, false},       // ch2 aileron
    {"left_stick_vertical", ChannelKind::AnalogAxis, false},          // ch3 throttle
    {"left_stick_horizontal", ChannelKind::AnalogAxis, false},        // ch4 rudder
    {"two_pos_switch_a", ChannelKind::TwoPositionSwitch, false},      // ch5
    {"wireless_estop", ChannelKind::TwoPositionSwitch, false},        // ch6 — LOW engages e-stop
    {"three_pos_switch_a", ChannelKind::ThreePositionSwitch, false},  // ch7
    {"three_pos_switch_b", ChannelKind::ThreePositionSwitch, false},  // ch8
    {"two_pos_switch_b", ChannelKind::TwoPositionSwitch, false},      // ch9
    {"two_pos_switch_c", ChannelKind::TwoPositionSwitch, false},      // ch10
};

// ---------------------------------------------------------------------------
// RcReceiver implementation
// ---------------------------------------------------------------------------

// Decode one raw sample into an endpoint category.
ChannelDecode RcReceiver::decodeRaw(uint16_t raw_value)
{
    if (raw_value < 900 || raw_value > 2100)
    {
        return ChannelDecode::Implausible;
    }
    if (raw_value < SWITCH_LOW_THRESHOLD)
    {
        return ChannelDecode::Low;
    }
    if (raw_value > SWITCH_HIGH_THRESHOLD)
    {
        return ChannelDecode::High;
    }
    return ChannelDecode::Intermediate;
}

// TRUE while raw sits inside the narrow failsafe signature window.
bool RcReceiver::inSignatureWindow(uint16_t raw_value)
{
    return (raw_value >= FAILSAFE_CENTER - FAILSAFE_TOLERANCE) &&
           (raw_value <= FAILSAFE_CENTER + FAILSAFE_TOLERANCE);
}

void RcReceiver::begin(int receive_pin)
{
    // IBusBM::begin() re-begins UART1 with (rxPin, txPin) on ESP32, so no
    // separate Serial1.begin() is needed. IBUSBM_NOTIMER disables the
    // library's interrupt timer — we poll loop() ourselves every iteration.
    ibus_protocol_.begin(Serial1, IBUSBM_NOTIMER, receive_pin, -1);
    Serial.printf("[RC] i-BUS receiver on UART1 RX=GPIO%d, 115200 baud\n", receive_pin);
}

void RcReceiver::service(uint32_t now_ms)
{
    // 1) Pump the protocol state machine — MUST happen every call.
    ibus_protocol_.loop();

    // 2) Frame freshness via the library's frame counter (volatile, wraps at
    //    256 — inequality compare is safe). IBusBM keeps returning the last
    //    values forever after the receiver dies, so the counter is the only
    //    trustworthy liveness signal. We only treat a frame as "fresh" when
    //    the counter actually changed — a long service gap (e.g. ROS
    //    blocking) must not advance wall-clock persistence timers on stale
    //    buffered bytes.
    uint8_t frame_counter = ibus_protocol_.cnt_rec;
    const bool got_fresh_frame = (frame_counter != last_frame_counter_);
    // Gap check BEFORE refreshing timestamps (correction: a frame arriving
    // after a long service gap must invalidate stale persistence evidence,
    // not smuggle it through by refreshing freshness first).
    const bool long_gap_since_valid =
        (last_valid_frame_ms_ != 0) &&
        ((now_ms - last_valid_frame_ms_) > RECEIVER_FRAME_TIMEOUT_MS);
    if (got_fresh_frame)
    {
        last_frame_counter_ = frame_counter;
        last_frame_seen_ms_ = now_ms;
    }
    bool frames_fresh = (now_ms - last_frame_seen_ms_) <= RECEIVER_FRAME_TIMEOUT_MS;

    // 3) Snapshot raw channel values (1000..2000 µs when healthy).
    for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
    {
        raw_channels_[index] = ibus_protocol_.readChannel(index);
    }
    uint16_t emergency_stop_raw = raw_channels_[ESTOP_CHANNEL_INDEX];

    // 4) Per-channel decode + confirmation (fresh frames only).
    //    TWO distinct timers per channel:
    //      intermediate duration = continuous absence of a valid endpoint
    //        -> bounds how long the previous endpoint may be retained.
    //      signature duration = continuous fresh-frame evidence inside the
    //        narrow window -> confirms the receiver-failsafe signature.
    //    Both start on ENTRY and NEVER restart on further intermediate
    //    samples, signature/other category changes, or 1500<->1450 wobble.
    //    A valid endpoint ends the intermediate episode (and, for that
    //    channel, clears signature confirmation — healthy replacement
    //    evidence). Leaving the narrow window resets signature confirmation
    //    but does NOT restore permission.
    uint16_t signature_mask = 0;
    if (got_fresh_frame)
    {
        if (long_gap_since_valid)
        {
            // Stale evidence from before the gap is NOT continuous
            // confirmation — reset all per-channel timers/flags.
            for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
            {
                signature_since_ms_[index] = 0;
                signature_timer_active_[index] = false;
                signature_confirmed_[index] = false;
                intermediate_since_ms_[index] = 0;
                intermediate_active_[index] = false;
                invalid_input_latched_[index] = false;
            }
            signature_confirming_mask_ = 0;
            ch6_intermediate_since_ms_ = 0;
            ch6_intermediate_active_ = false;
        }
        last_valid_frame_ms_ = now_ms;
        for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
        {
            const ChannelKind kind = RC_CHANNEL_CONFIGURATION[index].kind;
            if (kind == ChannelKind::AnalogAxis)
            {
                continue; // joysticks are telemetry-only — never decoded
            }
            ChannelDecode decode = decodeRaw(raw_channels_[index]);
            if (kind == ChannelKind::ThreePositionSwitch)
            {
                // 3-pos channels maintain ONLY the effective feature value
                // (feature requests read effectiveChannel()). NO signature
                // confirmation — MID sits exactly at the 1500 failsafe
                // center, where 1500 is legitimate input — and NO
                // invalid-input latch — MID is a legitimate resting
                // position, so a sustained MID-zone rest must never stop
                // the system or its other channels.
                if (decode == ChannelDecode::Low || decode == ChannelDecode::High)
                {
                    // Valid endpoint: healthy replacement evidence — updates
                    // the relay request at the next ~20 ms evaluation and
                    // ends any sweep episode.
                    effective_channels_[index] = raw_channels_[index];
                    intermediate_active_[index] = false;
                    intermediate_since_ms_[index] = 0;
                }
                else
                {
                    // Intermediate or implausible: retain the previous
                    // effective value through a brief sweep, then resolve a
                    // sustained rest to canonical MID (1500 -> INACTIVE).
                    if (!intermediate_active_[index])
                    {
                        intermediate_active_[index] = true;
                        intermediate_since_ms_[index] = now_ms;
                    }
                    if ((now_ms - intermediate_since_ms_[index]) >= INTERMEDIATE_GRACE_MS)
                    {
                        effective_channels_[index] = FAILSAFE_CENTER;
                    }
                }
                continue; // signature / invalid-input logic below is 2-pos only
            }
            bool in_window = inSignatureWindow(raw_channels_[index]);
            if (in_window)
            {
                signature_mask |= (1u << index);
            }
            if (decode == ChannelDecode::Low || decode == ChannelDecode::High)
            {
                // Valid endpoint: healthy replacement evidence for THIS
                // channel only. Ends its intermediate episode, clears its
                // invalid latch and its signature confirmation.
                effective_channels_[index] = raw_channels_[index];
                intermediate_since_ms_[index] = 0;
                intermediate_active_[index] = false;
                invalid_input_latched_[index] = false;
                signature_since_ms_[index] = 0;
                signature_timer_active_[index] = false;
                signature_confirmed_[index] = false;
            }
            else if (decode == ChannelDecode::Implausible)
            {
                // Implausible: ordinary channels latch invalid input at
                // once; CH6 is handled in its own decode below. Signature
                // confirmation cannot run on implausible input.
                if (index != ESTOP_CHANNEL_INDEX)
                {
                    invalid_input_latched_[index] = true;
                }
                signature_since_ms_[index] = 0;
                signature_timer_active_[index] = false;
                signature_confirmed_[index] = false;
            }
            else // Intermediate, plausible
            {
                // Episode timer starts on ENTRY only — never restarts.
                if (!intermediate_active_[index])
                {
                    intermediate_active_[index] = true;
                    intermediate_since_ms_[index] = now_ms;
                }
                if ((now_ms - intermediate_since_ms_[index]) >= INTERMEDIATE_GRACE_MS)
                {
                    if (index != ESTOP_CHANNEL_INDEX)
                    {
                        invalid_input_latched_[index] = true;
                    }
                }
                // Signature timer starts on ENTRY into the window only.
                if (in_window)
                {
                    if (!signature_timer_active_[index] && !signature_confirmed_[index])
                    {
                        signature_timer_active_[index] = true;
                        signature_since_ms_[index] = now_ms;
                    }
                    if (signature_timer_active_[index] &&
                        (now_ms - signature_since_ms_[index]) >= FAILSAFE_CONFIRM_MS)
                    {
                        signature_confirmed_[index] = true;
                    }
                }
                else
                {
                    // Left the window: reset signature confirmation, but the
                    // intermediate episode continues — NOT permission.
                    signature_since_ms_[index] = 0;
                    signature_timer_active_[index] = false;
                    signature_confirmed_[index] = false;
                }
            }
        }
        latest_frame_signature_mask_ = signature_mask;
        signature_confirming_mask_ = 0;
        for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
        {
            if (signature_timer_active_[index] && !signature_confirmed_[index])
            {
                signature_confirming_mask_ |= (1u << index);
            }
        }
        // CH6 decoded stop: valid endpoints end the grace episode at once.
        ChannelDecode ch6_decode = decodeRaw(emergency_stop_raw);
        if (ch6_decode == ChannelDecode::Low)
        {
            decoded_ch6_stop_ = true;
            ch6_intermediate_since_ms_ = 0;
            ch6_intermediate_active_ = false;
        }
        else if (ch6_decode == ChannelDecode::High)
        {
            decoded_ch6_stop_ = false;
            ch6_intermediate_since_ms_ = 0;
            ch6_intermediate_active_ = false;
        }
        else if (ch6_decode == ChannelDecode::Implausible)
        {
            decoded_ch6_stop_ = true;
            ch6_intermediate_since_ms_ = 0;
            ch6_intermediate_active_ = false;
        }
        else // Intermediate: retain previous decoded state, bounded
        {
            if (!ch6_intermediate_active_)
            {
                ch6_intermediate_active_ = true;
                ch6_intermediate_since_ms_ = now_ms;
            }
            if ((now_ms - ch6_intermediate_since_ms_) >= INTERMEDIATE_GRACE_MS)
            {
                decoded_ch6_stop_ = true;
            }
        }
    }
    else if (!frames_fresh)
    {
        // Receiver lost: invalidate all per-channel evidence. Confirmation
        // restarts on fresh frames after signal returns. decoded_ch6_stop_
        // is recomputed below from signal loss (stays stopped).
        for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
        {
            signature_since_ms_[index] = 0;
            signature_timer_active_[index] = false;
            signature_confirmed_[index] = false;
            intermediate_since_ms_[index] = 0;
            intermediate_active_[index] = false;
            invalid_input_latched_[index] = false;
        }
        latest_frame_signature_mask_ = 0;
        signature_confirming_mask_ = 0;
        ch6_intermediate_since_ms_ = 0;
        ch6_intermediate_active_ = false;
    }
    // No fresh frame but frames still fresh: all per-channel evidence
    // carries over unchanged (cached latest-valid-frame state).

    // 5) State + authoritative receiver-local stop (computed ONCE here).
    //
    //    Decoded frames fresh AND ch6 not below unbound threshold?
    //        |
    //        +-- No --> SignalLost ---> receiver-local stop (signal loss)
    //        |
    //        +-- Yes --> Any 2-pos channel with CONFIRMED signature?
    //                        |
    //                        +-- Yes --> FailsafeEngaged -> stop (failsafe)
    //                        |
    //                        +-- No --> state is SignalActive (liveness);
    //                                   stop still applies for decoded CH6
    //                                   stop or persistent invalid input.
    //
    //    Brief intermediate / unconfirmed signature NEVER stops by itself.
    //    "Immediate" CH6 LOW means the next 20 ms feature evaluation, not
    //    necessarily this 1 ms iteration.
    bool any_signature_confirmed = false;
    bool any_invalid_input = false;
    for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
    {
        if (signature_confirmed_[index])
        {
            any_signature_confirmed = true;
        }
        if (invalid_input_latched_[index])
        {
            any_invalid_input = true;
        }
    }
    if (!frames_fresh || emergency_stop_raw < NO_SIGNAL_RAW_VALUE)
    {
        state_ = ReceiverOperatingState::SignalLost;
    }
    else if (any_signature_confirmed)
    {
        state_ = ReceiverOperatingState::FailsafeEngaged;
    }
    else
    {
        state_ = ReceiverOperatingState::SignalActive;
    }
    // Authoritative stop: signal loss, confirmed failsafe, decoded CH6
    // stop, or persistent invalid input. Recomputed from confirmed
    // evidence every service() — clearing one reason never clears another.
    receiver_stop_reasons_ = 0;
    if (!frames_fresh || emergency_stop_raw < NO_SIGNAL_RAW_VALUE)
    {
        receiver_stop_reasons_ |= RECV_STOP_SIGNAL_LOSS;
    }
    if (any_signature_confirmed)
    {
        receiver_stop_reasons_ |= RECV_STOP_FAILSAFE;
    }
    if (decoded_ch6_stop_ && frames_fresh && emergency_stop_raw >= NO_SIGNAL_RAW_VALUE)
    {
        receiver_stop_reasons_ |= RECV_STOP_CH6;
    }
    if (any_invalid_input && frames_fresh && emergency_stop_raw >= NO_SIGNAL_RAW_VALUE)
    {
        receiver_stop_reasons_ |= RECV_STOP_INVALID_INPUT;
    }
    receiver_local_stop_ = (receiver_stop_reasons_ != 0);

    // 6) Transition logging (beeps are requested by the feature layer, which
    //    watches these transitions — the receiver stays free of board deps).
    if (state_ != previous_state_)
    {
        switch (state_)
        {
        case ReceiverOperatingState::SignalLost:
            enqueueLog(LogCategory::RcReceiver, 1, 0);
            break;
        case ReceiverOperatingState::FailsafeEngaged:
            enqueueLog(LogCategory::RcReceiver, 2, 0);
            break;
        case ReceiverOperatingState::SignalActive:
            enqueueLog(LogCategory::RcReceiver, 3, 0);
            break;
        }
        previous_state_ = state_;
    }
}

float mapChannelValueToJoyAxis(uint16_t raw_value, const ReceiverChannelConfiguration &configuration)
{
    // Guard: values outside any plausible servo range mean garbage data —
    // publish neutral instead of a wildly wrong number.
    if (raw_value < 900 || raw_value > 2100)
    {
        return 0.0f;
    }

    float mapped_value = 0.0f;
    switch (configuration.kind)
    {
    case ChannelKind::AnalogAxis:
        mapped_value = (static_cast<float>(raw_value) - 1500.0f) / 500.0f;
        if (mapped_value > 1.0f) mapped_value = 1.0f;
        if (mapped_value < -1.0f) mapped_value = -1.0f;
        break;
    case ChannelKind::TwoPositionSwitch:
        // SWITCH POLARITY: TRUE/active when LOW (1000). MID (failsafe 1500)
        // and HIGH (2000) both read False/inactive.
        mapped_value = (raw_value < SWITCH_LOW_THRESHOLD) ? 1.0f : 0.0f;
        break;
    case ChannelKind::ThreePositionSwitch:
        // LOW = 1.0, MID (failsafe) = 0.0, HIGH = -1.0 — the failsafe value
        // sits at neutral 0 for every downstream feature.
        if (raw_value < SWITCH_LOW_THRESHOLD)       mapped_value = 1.0f;
        else if (raw_value > SWITCH_HIGH_THRESHOLD) mapped_value = -1.0f;
        else                                        mapped_value = 0.0f;
        break;
    }

    if (configuration.inverted)
    {
        // Mirror the published AXIS value only. A 2-position switch
        // complements 1<->0; a 3-position switch mirrors 1<->-1 with the
        // middle staying 0. NOTE: feature activation always uses the
        // PHYSICAL TRUE (LOW) position — inversion is a telemetry concern.
        mapped_value = configuration.kind == ChannelKind::TwoPositionSwitch
                           ? 1.0f - mapped_value
                           : -mapped_value;
    }
    return mapped_value;
}

bool channelIsActivated(uint16_t raw_value, const ReceiverChannelConfiguration &configuration)
{
    if (raw_value < 900 || raw_value > 2100)
    {
        return false; // garbage data — never activates
    }
    if (configuration.kind == ChannelKind::TwoPositionSwitch ||
        configuration.kind == ChannelKind::ThreePositionSwitch)
    {
        // TRUE position = LOW zone (1000). MID (failsafe) and HIGH are
        // inactive — the failsafe value can never activate a feature.
        return raw_value < SWITCH_LOW_THRESHOLD;
    }
    return false; // analog axes never "activate"
}

} // namespace karr
