#include "karr/relay_feature_controller.h"

#include <Arduino.h>

#include "karr/diagnostics.h"

namespace karr
{


namespace
{

// Switch -> relay feature bindings. >>> EDIT ME — TYPE PLACEHOLDERS <<<
// The feature names below are PLACEHOLDER TYPES used to exercise the
// mapping — they are NOT the exact features we will build later (only the
// ch6 wireless e-stop is a defined safety function today; lighting,
// beacon, strobe and aux rows will be replaced when real features are
// designed). Each entry says: "when this RC channel is in its TRUE (LOW)
// position, energize that relay".
//
// CH6 (index ESTOP_CHANNEL_INDEX, "wireless_estop") is SAFETY-RESERVED:
// its LOW position engages the wireless e-stop and its mid-zone (1500) is
// the receiver failsafe signature. It must NEVER be bound to a feature —
// validateSwitchToRelayBindings() enforces this at startup.
//
// RECOVERY / REARM: the baseline has NO global rearm latch and no required
// switch-transition sequence — when a feature's dependencies recover, it
// follows its current valid inputs (a switch left TRUE re-energizes as
// soon as permission returns). A future hazardous output gets its OWN
// per-feature rearm rule HERE after a separate review; that possibility
// must not grow into a baseline-wide rearm subsystem.
struct SwitchToRelayBinding
{
    uint8_t rc_channel_index; // 0-based index into the RC channel table
    uint8_t relay_index;      // 0..7 (relay 1..8)
    const char *feature_name;
};

constexpr SwitchToRelayBinding SWITCH_TO_RELAY_BINDINGS[] = {
    {4, 0, "front_lighting"}, // ch5  -> relay 1
    // ch6 intentionally NOT bound — emergency_stop safety-reserved
    {6, 2, "beacon"},         // ch7  -> relay 3
    {7, 3, "work_light"},     // ch8  -> relay 4
    {8, 4, "strobe"},         // ch9  -> relay 5
    {9, 5, "auxiliary_1"},    // ch10 -> relay 6
};
constexpr uint8_t SWITCH_TO_RELAY_BINDING_COUNT =
    sizeof(SWITCH_TO_RELAY_BINDINGS) / sizeof(SWITCH_TO_RELAY_BINDINGS[0]);

// Shared binding-validity verdict — ONE rule set consumed both by the
// runtime guard (computeDesiredRelayState) and by the startup validator
// below (which adds per-reason logs). A binding whose channel is out of
// range can never be DEREFERENCED for a kind lookup, so ChannelOutOfRange
// is checked first; the remaining order matches the startup diagnostic
// order. Duplicate-relay sharing stays a TABLE-level warning in the
// validator (intentional sharing may be legitimate) — never a per-binding
// error, so it is not part of this enum.
enum class BindingError
{
    None,            // usable
    ChannelOutOfRange,
    RelayOutOfRange,
    ReservedChannel, // safety-reserved e-stop channel
    NotASwitch,      // analog axis bound as a feature switch
};

BindingError checkBinding(const SwitchToRelayBinding &binding)
{
    if (binding.rc_channel_index >= RC_CHANNEL_COUNT)
    {
        return BindingError::ChannelOutOfRange;
    }
    if (binding.relay_index >= RELAY_COUNT)
    {
        return BindingError::RelayOutOfRange;
    }
    if (binding.rc_channel_index == ESTOP_CHANNEL_INDEX)
    {
        return BindingError::ReservedChannel; // safety-reserved — never a feature
    }
    if (RC_CHANNEL_CONFIGURATION[binding.rc_channel_index].kind == ChannelKind::AnalogAxis)
    {
        return BindingError::NotASwitch; // axes are not switches
    }
    return BindingError::None;
}

// Runtime usability predicate — the same rules validateSwitchToRelay-
// Bindings() reports at startup, enforced here as well (not just logged).
bool isBindingUsable(const SwitchToRelayBinding &binding)
{
    return checkBinding(binding) == BindingError::None;
}

// CRITICAL DECISION: losing the micro-ROS agent inhibits ROS-dependent
// features by default. An e-stop board must fail de-energized when nobody
// is supervising the bus. SCOPE: today EVERY feature is RC-driven and
// declared ROS-dependent in rcFeaturesPermitted(), so the practical
// effect is "all current features go inactive on agent loss" — flip to
// false if lighting features should survive an agent outage. Receiver-
// local inhibition (RC loss, failsafe signature, wireless stop) remains a
// hard override for RC-driven features either way, and this flag must NOT
// gate future INDEPENDENT features (e.g. a safety indication) — each
// future group declares its own ROS dependency instead of inheriting
// this one.
constexpr bool FORCE_SAFE_ON_AGENT_LOSS = true;

// ---------------------------------------------------------------------------
// validateSwitchToRelayBindings — startup sanity check of the RC BINDING
// TABLE (the table is meant to be hand-edited; catch mistakes early).
// Per-binding verdicts come from the single checkBinding() rule set (logs
// the FIRST failure per binding — still invalid either way); the
// double-bound scan stays here because it is a TABLE-level property
// (warn only — intentional sharing may be legitimate; see RELAY OWNERSHIP
// in the header: OR-combination is a sharing rule, not arbitration).
// SCOPE: a failure here inhibits the RC-driven feature group only — future
// feature groups validate their own configuration.
// ---------------------------------------------------------------------------
bool validateSwitchToRelayBindings()
{
    bool all_valid = true;
    for (uint8_t binding_index = 0; binding_index < SWITCH_TO_RELAY_BINDING_COUNT; ++binding_index)
    {
        const SwitchToRelayBinding &binding = SWITCH_TO_RELAY_BINDINGS[binding_index];

        switch (checkBinding(binding))
        {
        case BindingError::None:
            break;
        case BindingError::ChannelOutOfRange:
            Serial.printf("[CFG] binding '%s': channel index %u out of range (max %u)\n",
                          binding.feature_name, binding.rc_channel_index,
                          RC_CHANNEL_COUNT - 1);
            all_valid = false;
            continue; // no meaningful kind/dup checks for an out-of-range channel
        case BindingError::RelayOutOfRange:
            Serial.printf("[CFG] binding '%s': relay index %u out of range (max %u)\n",
                          binding.feature_name, binding.relay_index, RELAY_COUNT - 1);
            all_valid = false;
            continue; // no meaningful dup check for an out-of-range relay
        case BindingError::ReservedChannel:
            Serial.printf("[CFG] binding '%s': channel index %u is the SAFETY-RESERVED "
                          "e-stop channel — must not drive a feature\n",
                          binding.feature_name, binding.rc_channel_index);
            all_valid = false;
            break;
        case BindingError::NotASwitch:
            Serial.printf("[CFG] binding '%s': channel '%s' is an analog axis, not a switch\n",
                          binding.feature_name,
                          RC_CHANNEL_CONFIGURATION[binding.rc_channel_index].human_name);
            all_valid = false;
            break;
        }

        for (uint8_t other_index = binding_index + 1;
             other_index < SWITCH_TO_RELAY_BINDING_COUNT; ++other_index)
        {
            if (SWITCH_TO_RELAY_BINDINGS[other_index].relay_index == binding.relay_index)
            {
                Serial.printf("[CFG] warning: relay %u bound by both '%s' and '%s' "
                              "(intentional?)\n",
                              binding.relay_index, binding.feature_name,
                              SWITCH_TO_RELAY_BINDINGS[other_index].feature_name);
            }
        }
    }
    return all_valid;
}

} // namespace

bool RelayFeatureController::validateStartupConfiguration()
{
    switch_binding_configuration_valid_ = validateSwitchToRelayBindings();
    return switch_binding_configuration_valid_;
}
bool RelayFeatureController::boardActuationReady() const
{
    // BOARD-LEVEL constraint only — feature dependencies deliberately do
    // NOT belong here. Uninitialized relay hardware may never be
    // energized; RelayBoard ALSO refuses nonzero requests while
    // uninitialized (RelayBoard invariant) — belt and suspenders, and it
    // keeps the status log honest.
    return relay_board_.isRelayInitialized();
}

bool RelayFeatureController::rcFeaturesPermitted(bool agent_connected) const
{
    // Dependency 1 — receiver-local inhibition: signal loss, CONFIRMED
    // failsafe, decoded CH6 stop and persistent invalid switch input are
    // all covered by ONE predicate, so this permission check, the
    // /relay_board/estop topic and the LED policy can never disagree
    // about receiver-local state.
    if (receiver_.isLocallyInhibited())
    {
        return false;
    }
    // Dependency 2 — ROS availability, only for features that declare it
    // (all current ones; see FORCE_SAFE_ON_AGENT_LOSS for the scope rule).
    if (FORCE_SAFE_ON_AGENT_LOSS && !agent_connected)
    {
        return false;
    }
    // Dependency 3 — RC binding table validity: config health, SCOPED to
    // this group (loudly reported at startup) instead of running a
    // half-broken mapping on a safety device. /relay_board/estop
    // intentionally stays receiver-local — config health is surfaced via
    // the LED and the serial status line.
    if (!switch_binding_configuration_valid_)
    {
        return false;
    }
    return true;
}

uint8_t RelayFeatureController::computeRcFeatureRequests() const
{
    // RC-group output requests: switch in its TRUE (LOW) position ->
    // request its bound relay. Uses the receiver's EFFECTIVE channel value
    // (retained last valid endpoint during brief intermediate input), so a
    // switch bouncing through the middle does not chatter its own relay.
    // Bindings failing validation are skipped here as well as flagged at
    // startup — a bad edit degrades gracefully, never reads out of bounds,
    // and never actuates a safety-reserved channel.
    uint8_t desired_state = SAFE_RELAY_STATE;
    for (uint8_t binding_index = 0; binding_index < SWITCH_TO_RELAY_BINDING_COUNT; ++binding_index)
    {
        const SwitchToRelayBinding &binding = SWITCH_TO_RELAY_BINDINGS[binding_index];
        if (!isBindingUsable(binding))
        {
            continue; // invalid/reserved binding — flagged at startup
        }
        uint16_t raw_value = receiver_.effectiveChannel(binding.rc_channel_index);
        const ReceiverChannelConfiguration &configuration =
            receiver_.channelConfiguration(binding.rc_channel_index);
        if (channelIsActivated(raw_value, configuration))
        {
            desired_state |= (1u << binding.relay_index);
        }
    }
    return desired_state;
}

uint8_t RelayFeatureController::computeDesiredRelayState(bool agent_connected) const
{
    // 1) Each feature group evaluates ITS dependencies and ORs the bits
    //    it requests into the shared mask (see header diagram).
    uint8_t desired_state = SAFE_RELAY_STATE;

    // Group 1 — RC-driven switch features: the ONLY group today; its
    // binding names are type placeholders, not the final feature set.
    if (rcFeaturesPermitted(agent_connected))
    {
        desired_state |= computeRcFeatureRequests();
    }

    // EXTENSION POINT — future groups (digital-input lighting, an
    // independent safety indication, ROS-reliant functionality) add ONE
    // dependency-checked block here, each with its own
    // unavailable-dependency behavior. They must NOT inherit this group's
    // dependencies, and their requests simply OR in.

    // 2) Board-level constraints applied AFTER combination, centrally —
    //    an uninitialized expander may not energize anything regardless
    //    of which feature asked (RelayBoard re-enforces this).
    if (!boardActuationReady())
    {
        desired_state = SAFE_RELAY_STATE;
    }
    return desired_state;
}
void RelayFeatureController::service(bool agent_connected)
{
    // --- Per-scope dependency evaluation (see header diagram) ---
    const bool board_actuation_ready = boardActuationReady();
    const bool rc_features_permitted = rcFeaturesPermitted(agent_connected);

    // --- Output composition + board constraints -> the single write site ---
    uint8_t target_state = computeDesiredRelayState(agent_connected);
    last_desired_mask_ = target_state; // record for coherent telemetry snapshot

    // --- Relay actuation (the only relay write site in the firmware) ---
    uint8_t state_before = relay_board_.lastWrittenRelayStates();
    bool write_confirmed = relay_board_.applyRelayStates(target_state);
    if (relay_board_.lastWrittenRelayStates() != state_before)
    {
        enqueueLog(LogCategory::RelayBoard, 2, relay_board_.lastWrittenRelayStates());
    }

    if (board_actuation_ready != previous_board_actuation_ready_ ||
        rc_features_permitted != previous_rc_features_permitted_)
    {
        // Transition log — FIRST matching scope wins so the dominant
        // constraint is what the operator reads; reasons are spelled out
        // rather than a bare open/closed verdict.
        if (!board_actuation_ready)
        {
            enqueueLog(LogCategory::FeatureController, 1, target_state);
        }
        else if (!rc_features_permitted)
        {
            // Scoped reasons for the RC group; config health has no topic,
            // so it is reported here (plus LED + startup log). Receiver
            // reasons come from the authoritative stop mask computed in
            // control — never rebuilt from raw fields here.
            uint8_t reason_code = 0;
            uint8_t recv_reasons = receiver_.receiverStopReasons();
            if (recv_reasons & RECV_STOP_SIGNAL_LOSS) reason_code |= 1u;
            if (recv_reasons & RECV_STOP_FAILSAFE) reason_code |= 2u;
            if (recv_reasons & RECV_STOP_CH6) reason_code |= 4u;
            if (recv_reasons & RECV_STOP_INVALID_INPUT) reason_code |= 32u;
            if (!switch_binding_configuration_valid_) reason_code |= 8u;
            if (!agent_connected) reason_code |= 16u;
            enqueueLog(LogCategory::FeatureController, 2, reason_code);
        }
        else
        {
            enqueueLog(LogCategory::FeatureController, 3, target_state);
        }
        if (!board_actuation_ready || !rc_features_permitted)
        {
            // IMMEDIATE edge beep: the cutoff above is already real (relays
            // went safe this same service call via the confirmed-stop
            // predicate) — the operator hears it now, not after the
            // debounced state confirms seconds later. The
            // debounced transition beeps below stay as the second,
            // classified confirmation (300/500 ms by cause).
            relay_board_.requestStatusBeep(250); // one beep when inhibited, never repeats
        }
        else
        {
            // Permitted again before any debounced state confirmed the
            // cutoff (bounce / glitch that cleared fast): stop the edge beep
            // early. Relays were safe throughout; the truncated chirp itself
            // says "transient — watch signal quality".
            relay_board_.cancelStatusBeep();
        }
        previous_board_actuation_ready_ = board_actuation_ready;
        previous_rc_features_permitted_ = rc_features_permitted;
    }

    // --- Wireless e-stop switch transition reactions (one-shot beep) ---
    bool estop_switch_engaged = receiver_.isEstopSwitchEngaged();
    if (estop_switch_engaged != previous_estop_switch_engaged_)
    {
        enqueueLog(LogCategory::FeatureController, 4, estop_switch_engaged ? 1 : 0);
        if (estop_switch_engaged)
        {
            relay_board_.requestStatusBeep(500); // one beep per engagement
        }
        previous_estop_switch_engaged_ = estop_switch_engaged;
    }

    // --- Receiver transition reactions (one-shot beeps) ---
    if (receiver_.state() != previous_receiver_state_)
    {
        switch (receiver_.state())
        {
        case ReceiverOperatingState::SignalLost:
            relay_board_.requestStatusBeep(300);
            break;
        case ReceiverOperatingState::FailsafeEngaged:
            relay_board_.requestStatusBeep(500);
            break;
        case ReceiverOperatingState::SignalActive:
            break;
        }
        previous_receiver_state_ = receiver_.state();
    }

    // --- Status LED policy ---
    // PRIORITY DIAGRAM (first match wins — ONE selected LED cannot show
    // every simultaneous condition; the serial status line, /relay_board/*
    // topics and the beep log carry the detail):
    //
    //   1. relay bus fault?        --yes--> RED solid
    //      |
    //      no
    //      v
    //   2. agent unavailable?      --yes--> RED blink
    //      |
    //      no
    //      v
    //   3. receiver-local          --yes--> YELLOW blink
    //      inhibition?
    //      |
    //      no
    //      v
    //   4. invalid RC binding      --yes--> PURPLE blink
    //      config?
    //      |
    //      no
    //      v
    //   5. GREEN solid
    //
    //   1 = board-level: uninitialized / last write status unknown /
    //       consecutive I2C failures — software can never confirm contact
    //       position, it only reports writes.
    //   2 = ROS CONNECTIVITY ONLY — does NOT claim local features stopped.
    //   3 = RC-DEPENDENT features inhibited — not every output. With
    //       independent features, /relay_board/estop TRUE may coexist
    //       with an active safety indication — scope, not conflict.
    //   4 = the RC group's config scope (a broken RC map, not a
    //       board-wide configuration fault).
    //   5 = no condition covered by THIS policy detected — NOT physical
    //       verification of relay operation, and not merely "frames
    //       exist".
    if (!relay_board_.isRelayInitialized() || !relay_board_.isRelayStateKnown() ||
        relay_board_.consecutiveWriteFailures() > 0)
    {
        relay_board_.setStatusLed(255, 0, 0, 0); // red solid: relay bus fault
    }
    else if (!agent_connected)
    {
        relay_board_.setStatusLed(255, 0, 0, 1000); // red, slow blink: agent unavailable (ROS connectivity)
    }
    else if (receiver_.isLocallyInhibited())
    {
        relay_board_.setStatusLed(255, 140, 0, 250); // yellow, fast blink: receiver-local inhibition (RC features)
    }
    else if (!switch_binding_configuration_valid_)
    {
        relay_board_.setStatusLed(180, 0, 255, 1000); // purple, slow blink: invalid RC binding config
    }
    else
    {
        relay_board_.setStatusLed(0, 255, 0, 0); // green, solid: no policy condition detected (not contact verification)
    }
}

} // namespace karr
