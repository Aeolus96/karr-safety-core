#pragma once

// ============================================================================
// Shared RC receiver configuration — the channel table both the receiver
// state machine (rc_receiver) and telemetry/binding consumers read.
// Declared here, defined once in rc_receiver.cpp.
// ============================================================================

#include <cstdint>

namespace karr
{// Channels in the tested i6S/iA6B i-BUS stream.
constexpr uint8_t RC_CHANNEL_COUNT = 10;

// E-stop channel (ch6). TWO roles, both on this channel:
//   1. No-signal check: raw below NO_SIGNAL_RAW_VALUE = receiver unbound.
//   2. WIRELESS E-STOP SWITCH: LOW position (1000) ENGAGES the e-stop,
//      HIGH position (2000) RELEASES it. The RC controller boots with the
//      switch up (1000), so the board always starts in the e-stop state
//      and the operator must deliberately switch it off. RF loss and the
//      failsafe signature are the second (automatic) e-stop layer.
//      Never bind this channel to a feature — enforced at startup.
constexpr uint8_t ESTOP_CHANNEL_INDEX = 5; // ch6 (1-based) = index 5

// --- Channel configuration table ---------------------------------------------
// >>> EDIT THIS TABLE to match your transmitter / mode setup <<<
// (Definition lives in rc_receiver.cpp; docs/diagrams/gen_rc_map.py parses it
// from there.)
// Transmitter: Turnigy TGY-i6S, MODE 1 (right stick = elevator/aileron, left stick =
// throttle/rudder):
//   ch1-ch4  : joysticks (analog 1000..2000)
//   ch5, ch6 : 2-position switches
//   ch7, ch8 : 3-position switches (1000-1500-2000)
//   ch9, ch10: 2-position toggle switches
//
// SWITCH POLARITY CONVENTION (user requirement): switches are ACTIVE when
// LOW (1000) and inactive when HIGH (2000). The failsafe value (1500)
// therefore reads as INACTIVE/neutral for every feature:
//   AnalogAxis          1000..2000 -> -1.0 .. +1.0   (sticks, center = 0)
//   TwoPositionSwitch   LOW -> 1.0 (True), MID/HIGH -> 0.0 (False)
//   ThreePositionSwitch LOW -> 1.0, MID -> 0.0, HIGH -> -1.0
// Buttons mirror this: button = 1 only in the LOW (True) position.
// `inverted` mirrors the published axis value.
//
// ch6 is the wireless e-stop switch (see ESTOP_CHANNEL_INDEX): LOW engages
// the e-stop, HIGH releases it. It is SAFETY-RESERVED — never bind it to
// a feature (RelayFeatureController's startup validator enforces this).
enum class ChannelKind
{
    AnalogAxis,
    TwoPositionSwitch,
    ThreePositionSwitch,
};

struct ReceiverChannelConfiguration
{
    const char *human_name; // shows up in logs / future metadata
    ChannelKind kind;
    bool inverted;
};

extern const ReceiverChannelConfiguration RC_CHANNEL_CONFIGURATION[RC_CHANNEL_COUNT];

// --- Value mapping helpers (pure functions over the config table) -----------
// Definitions live in rc_receiver.cpp.
float mapChannelValueToJoyAxis(uint16_t raw_value, const ReceiverChannelConfiguration &configuration);

// Switch ACTIVATION: TRUE position = LOW zone (1000) for both 2- and
// 3-position switches, per the board's switch polarity convention.
// Analog axes are never "activated".
bool channelIsActivated(uint16_t raw_value, const ReceiverChannelConfiguration &configuration);
} // namespace karr
