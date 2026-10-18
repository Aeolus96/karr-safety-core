#pragma once

// ============================================================================
// RelayFeatureController — ALL relay control logic. RC switches are mapped
// to relay FEATURES; nothing from ROS can command a relay — features receive
// inputs (RC channels today, later digital inputs / other sensors) and manage
// the relays through the RelayBoard.
//
// LAYERING NOTE (answer to "should features connect to a RelayBoard
// instance?"): YES — the board is the dependency sink. The feature layer
// holds RelayBoard& and RcReceiver& references, so dependencies flow
// RelayBoard / RcReceiver <- Controller <- app. The graph stays acyclic,
// every module is independently testable, and the board itself knows
// nothing about ROS or features.
//
// DEPENDENCY MODEL (board-wide restrictions vs feature dependencies):
//   BOARD-LEVEL actuation constraint: relay hardware completed its safe
//     initialization — applies to EVERY feature; RelayBoard additionally
//     refuses nonzero writes while uninitialized, so this cannot be
//     bypassed by feature code.
//   FEATURE-GROUP dependencies: each group declares its own (RC health,
//     digital-input validity, ROS availability, a defined stop condition)
//     and its own behavior when a dependency is unavailable. Today there
//     is exactly ONE group — the RC-driven bindings in the .cpp — and it
//     keeps full receiver-local inhibition; nothing else is implemented
//     yet.
//
//   RC state ──────────┐
//   Digital inputs ────┼──> per-group dependency evaluation
//   ROS availability ──┘            │
//                                   v
//                     combine feature output requests (OR)
//                                   │
//                                   v
//                  apply board-level constraints (relay init)
//                                   │
//                                   v
//                     single relay-output writer
//
// RELAY OWNERSHIP: one owner per relay unless sharing is intentional.
//   The OR-combination in the .cpp is a SHARING rule (any owning feature
//   may request ON), NOT an arbitration policy for conflicting priorities —
//   conflicting owners would need an explicit design. The startup
//   validator warns (never fails) on duplicate relay assignments so
//   intentional sharing stays visible.
//
// STOP-CONDITION BOUNDARY: RC loss, wireless e-stop engagement and a
//   future physical stop input need an EXPLICITLY defined output scope
//   when the features that use them are designed. An independent warning
//   light may remain active during a stop while an actuator must not —
//   neither "stop everything" nor "ignore stops for independent features"
//   may be applied silently. Today every output is RC-driven and inherits
//   full receiver-local inhibition; that is today's deliberate policy.
// ============================================================================

#include "karr/relay_board.h"
#include "karr/rc_receiver.h"

namespace karr
{

class RelayFeatureController
{
public:
    RelayFeatureController(RelayBoard &board, RcReceiver &receiver)
        : relay_board_(board), receiver_(receiver) {}

    // Startup sanity check of the RC BINDING TABLE (hand-edited; catch
    // mistakes early). PERSISTS the verdict: while invalid, RC-driven
    // feature operation stays inhibited — scoped to this table, not a
    // board-wide fault. Called from app boot, after receiver init and
    // before transport init (startup timing unchanged).
    bool validateStartupConfiguration();
    // Shared binding-validity verdict — consumed by the runtime guard
    // (computeDesiredRelayState), the LED policy, and the app status line.
    bool isSwitchBindingConfigurationValid() const
    {
        return switch_binding_configuration_valid_;
    }

    // Evaluates each feature group's own dependencies, combines the
    // resulting relay requests, applies board-level constraints and
    // performs the single relay write. agent_connected is injected by
    // the app so this class stays decoupled from the micro-ROS layer.
    // (No clock input: these decisions are purely combinational on the
    // latest receiver/board state.)
    void service(bool agent_connected);
    // Last desired (target) relay mask computed by service() — for the
    // coherent telemetry snapshot. Differs from lastWrittenRelayStates()
    // when an I2C write failed.
    uint8_t lastDesiredMask() const { return last_desired_mask_; }

private:
    // BOARD-LEVEL actuation constraint — applies to every feature, no
    // exceptions (see the header's dependency model).
    bool boardActuationReady() const;
    // FEATURE-GROUP dependency check for today's RC-driven features:
    // healthy local RC input + (configurable) agent connectivity + a
    // valid RC binding table. The config check lives HERE, not in the
    // board constraint, so a broken RC map can never inhibit a future
    // independently configured feature group.
    bool rcFeaturesPermitted(bool agent_connected) const;
    // The RC group's output requests (today's only feature group).
    uint8_t computeRcFeatureRequests() const;
    // Composition: permitted groups OR their bits in, then board
    // constraints — evaluated in exactly the order of the header
    // diagram; the result goes to the single writer.
    uint8_t computeDesiredRelayState(bool agent_connected) const;

    RelayBoard &relay_board_;
    RcReceiver &receiver_;
    bool previous_board_actuation_ready_ = false;
    bool previous_rc_features_permitted_ = false;
    ReceiverOperatingState previous_receiver_state_ = ReceiverOperatingState::SignalLost;
    // Boot default: ENGAGED (matches RcReceiver::isEstopSwitchEngaged()
    // before the first frames — raw values start at 0).
    bool previous_estop_switch_engaged_ = true;
    // Persisted result of the startup validation OF THE RC BINDING TABLE.
    // While false, RC-driven feature operation is INHIBITED (their bits stay
    // 0x00) — an invalid configuration must not run a half-broken mapping on
    // a safety device. CONFIG-FAULT SCOPE: this verdict covers the RC binding
    // table ONLY; it is a broken FEATURE configuration, not a board-wide
    // configuration problem — future feature groups (digital inputs,
    // independent indications) validate their own configuration and are not
    // inhibited by an unrelated RC-map error.
    bool switch_binding_configuration_valid_ = true;
    // Last desired relay target computed by service() — updated every
    // safety evaluation cycle for the coherent snapshot.
    uint8_t last_desired_mask_ = SAFE_RELAY_STATE;
};

} // namespace karr
