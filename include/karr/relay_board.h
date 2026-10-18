#pragma once

// ============================================================================
// RelayBoard — hardware facade for the Waveshare ESP32-S3-POE-ETH-8DI-8RO-C
// board: the 8 relays (TCA9554 I2C expander), the 8 digital inputs, the
// status RGB LED and the buzzer. Everything hardware-specific lives behind
// this class; the feature layer never touches Wire, pins, or registers
// directly. Depends only on the Arduino core + Wire — no ROS, no receiver.
// ============================================================================

#include <cstdint>

namespace karr
{

// --- Relay actuation behavior (shared with feature/telemetry consumers) ---
constexpr uint8_t RELAY_COUNT = 8;
// CRITICAL DECISION: this board's fail-safe default is the all-off mask
// 0x00. A set output bit energizes a relay (vendor Set_EXIOS() semantics:
// bit=1 turns the channel ON), so 0x00 commands every relay off. SCOPE:
// 0x00 is an all-off COMMAND mask, not a universal definition of safe
// behavior for every future feature, and a successful write is not
// contact feedback. Verify against your wiring before trusting this in
// the field.
constexpr uint8_t SAFE_RELAY_STATE = 0x00;

class RelayBoard
{
public:
    // Brings up I2C + expander + status outputs and commands all relays
    // off (0x00 write) as early as possible.
    void begin();

    // The single writer of relay hardware.
    // CRITICAL DECISIONS (state invariants):
    //  - relay_initialized_ and relay_state_known_ are SEPARATE concepts.
    //    Initialized = the full safe sequence (output latch + pin direction)
    //    succeeded at least once. State-known = the cached mask matches the
    //    last confirmed write. A successful output write alone does NOT
    //    establish initialization (pin direction may still be unset).
    //  - While uninitialized, only SAFE requests are accepted and they run
    //    the full init sequence; nonzero requests are refused outright.
    //  - Ordinary target changes do NOT bypass fault retry limits; only a
    //    NEW transition to SAFE gets one immediate attempt (subsequent
    //    failed attempts stay rate-limited).
    //
    // RECOVERY ASSUMPTION (documented, deliberate): relay_initialized_
    // means the safe sequence succeeded at least once and is NOT cleared by
    // ordinary write failures (those only clear relay_state_known_). An
    // INDEPENDENT expander reset (power glitch on the peripheral) is
    // outside this recovery model — its direction configuration could no
    // longer match the assumption, and a later output-register write alone
    // would not re-establish direction. If that failure mode matters, the
    // follow-up is a defined communication-fault condition that triggers
    // full re-initialization (re-running attemptSafeInitialization), not a
    // silent extra state check.
    bool applyRelayStates(uint8_t requested_state_mask);

    // The mask the hardware was LAST SUCCESSFULLY written to. This is NOT a
    // readback — a successful register write is not relay-contact
    // confirmation — and it is only meaningful while isRelayStateKnown().
    uint8_t lastWrittenRelayStates() const { return applied_state_mask_; }
    bool isRelayStateKnown() const { return relay_state_known_; }
    bool isRelayInitialized() const { return relay_initialized_; }
    uint32_t consecutiveWriteFailures() const { return consecutive_write_failures_; }
    // Duration of the last I2C register transaction (micros, measured
    // around Wire.endTransmission inside writeI2cRegister). Control reads
    // this for honest timing diagnostics — it measures the bus, not the
    // surrounding feature evaluation.
    uint32_t lastI2cWriteUs() const { return last_i2c_write_us_; }

    // Digital inputs: bit N = 1 means input N+1 is ACTIVE (pulled low).
    uint8_t readDigitalInputs() const;

    // Status indicators (policy is chosen by the feature layer, plumbing
    // lives here). blink_interval_ms == 0 means solid-on.
    void setStatusLed(uint8_t red, uint8_t green, uint8_t blue, uint16_t blink_interval_ms);
    void requestStatusBeep(uint32_t duration_ms);
    void cancelStatusBeep();
    void serviceStatusIndicators(uint32_t now_ms);

private:
    bool attemptSafeInitialization();
    bool writeI2cRegister(uint8_t device_address, uint8_t register_address, uint8_t value);
    void showStatusRgb(uint8_t red, uint8_t green, uint8_t blue);

    // Relay state — two independent invariants, see the class comment.
    bool relay_initialized_ = false;           // full safe init sequence done
    uint8_t applied_state_mask_ = SAFE_RELAY_STATE;
    bool relay_state_known_ = false;           // cached mask matches hardware
    uint8_t last_attempted_mask_ = SAFE_RELAY_STATE; // drives the retry throttle
    uint32_t cumulative_write_failures_ = 0;   // diagnostics only
    uint32_t consecutive_write_failures_ = 0;  // retry throttle driver
    uint32_t last_write_attempt_ms_ = 0;
    uint32_t last_i2c_write_us_ = 0;           // last transaction duration

    // Indicator state
    struct StatusLedPattern
    {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint16_t blink_interval_ms;
    };
    StatusLedPattern active_led_pattern_ = {0, 0, 0, 0};
    uint32_t led_blink_phase_started_ms_ = 0;
    bool led_in_dark_phase_ = false;
    // Rollover-safe beep timing: compare ELAPSED time against a duration,
    // never an absolute deadline (deadline comparison breaks at millis()
    // wrap, which happens every ~49 days).
    bool buzzer_active_ = false;
    uint32_t buzzer_started_ms_ = 0;
    uint32_t buzzer_duration_ms_ = 0;
};

} // namespace karr
