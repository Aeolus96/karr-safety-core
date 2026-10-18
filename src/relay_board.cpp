#include "karr/relay_board.h"

#include <Arduino.h>
#include <Wire.h> // I2C: hosts the TCA9554 8-bit relay expander

#include "karr/diagnostics.h"

namespace karr
{

// Board-private hardware constants — pins, expander registers, retry
// budgets. RELAY_COUNT / SAFE_RELAY_STATE are shared (see relay_board.h).
namespace
{

// --- I2C bus (hosts the TCA9554 8-bit relay expander) ---
constexpr uint8_t I2C_DATA_PIN = 42;  // SDA
constexpr uint8_t I2C_CLOCK_PIN = 41; // SCL

// --- TCA9554 8-bit I/O expander: the chip that switches the 8 relays ---
// Register transaction sequences follow the vendor driver exactly (proven
// on this board); only error handling was made truthful — the vendor's
// bool-returning functions used `return -1`, which is TRUTHY and would
// report failures as success.
constexpr uint8_t TCA9554_DEVICE_ADDRESS = 0x20;
constexpr uint8_t TCA9554_OUTPUT_REGISTER = 0x01;
constexpr uint8_t TCA9554_CONFIGURATION_REGISTER = 0x03; // 0 = output, 1 = input

// --- Digital inputs DIN1..DIN8 (active-low wiring, internal pull-ups) ---
constexpr uint8_t DIGITAL_INPUT_PINS[8] = {4, 5, 6, 7, 8, 9, 10, 11};

// --- Status RGB LED + buzzer (driven from the core-0 control task at
//     fixed 1 ms / 20 ms cadence; isolated from micro-ROS blocking on
//     core 1) ---
constexpr uint8_t STATUS_RGB_LED_PIN = 38;
constexpr uint8_t STATUS_BUZZER_PIN = 46;
constexpr uint32_t BUZZER_PWM_FREQUENCY_HZ = 1000;   // uint32_t on purpose:
constexpr uint8_t BUZZER_PWM_RESOLUTION_BITS = 8;    // uint8_t truncates 1000
constexpr uint8_t BUZZER_DUTY_CYCLE = 200;           // 0..255, vendor-tested loudness

// --- Relay actuation behavior (board-private) ---
constexpr uint32_t RELAY_WRITE_RETRY_PERIOD_MS = 200; // throttle I2C retries
                                                      // after a failed write
constexpr uint8_t RELAY_INIT_RETRY_COUNT = 3;         // boot-time init attempts
constexpr uint32_t RELAY_INIT_RETRY_PERIOD_MS = 50;   // between attempts

} // namespace

void RelayBoard::begin()
{
    Wire.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);

    // Reset lifecycle invariants: NOTHING is trusted until the full safe
    // initialization sequence succeeds.
    relay_initialized_ = false;
    relay_state_known_ = false;
    applied_state_mask_ = SAFE_RELAY_STATE;
    last_attempted_mask_ = SAFE_RELAY_STATE;
    consecutive_write_failures_ = 0;
    last_write_attempt_ms_ = 0;

    // Digital inputs: internal pull-ups, active-low wiring.
    for (uint8_t index = 0; index < 8; ++index)
    {
        pinMode(DIGITAL_INPUT_PINS[index], INPUT_PULLUP);
    }

    // Status outputs.
    pinMode(STATUS_RGB_LED_PIN, OUTPUT);
    ledcAttach(STATUS_BUZZER_PIN, BUZZER_PWM_FREQUENCY_HZ, BUZZER_PWM_RESOLUTION_BITS);
    ledcWrite(STATUS_BUZZER_PIN, 0);
    active_led_pattern_ = {0, 0, 255, 0}; // solid blue while booting
    showStatusRgb(0, 0, 255);

    // Safe initialization with bounded retries. If it never succeeds,
    // applyRelayStates() keeps retrying the full sequence every
    // RELAY_WRITE_RETRY_PERIOD_MS and refuses all nonzero requests.
    bool initialized = false;
    for (uint8_t attempt = 0; attempt < RELAY_INIT_RETRY_COUNT; ++attempt)
    {
        if (attemptSafeInitialization())
        {
            initialized = true;
            break;
        }
        Serial.printf("[RELAY] init attempt %u failed\n", attempt + 1);
        delay(RELAY_INIT_RETRY_PERIOD_MS); // boot-time only, bounded
    }
    if (initialized)
    {
        Serial.println("[RELAY] initialization completed — safe-output writes succeeded");
    }
    else
    {
        Serial.println("[RELAY] INIT FAILED — energizing inhibited until recovery");
    }
}

// ---------------------------------------------------------------------------
// attemptSafeInitialization — the full safe bring-up sequence. A successful
// output-register write does NOT establish pin direction, so BOTH steps must
// succeed, IN ORDER, before initialization counts.
// CRITICAL DECISION: if the latch write fails we do NOT proceed to configure
// output direction — configuring direction while the output state is unknown
// could energize relays mid-sequence.
// ---------------------------------------------------------------------------
bool RelayBoard::attemptSafeInitialization()
{
    if (!writeI2cRegister(TCA9554_DEVICE_ADDRESS, TCA9554_OUTPUT_REGISTER, SAFE_RELAY_STATE))
    {
        ++cumulative_write_failures_;
        ++consecutive_write_failures_;
        return false;
    }
    if (!writeI2cRegister(TCA9554_DEVICE_ADDRESS, TCA9554_CONFIGURATION_REGISTER, 0x00))
    {
        ++cumulative_write_failures_;
        ++consecutive_write_failures_;
        return false;
    }
    relay_initialized_ = true;
    applied_state_mask_ = SAFE_RELAY_STATE;
    last_attempted_mask_ = SAFE_RELAY_STATE;
    relay_state_known_ = true;
    consecutive_write_failures_ = 0;
    return true;
}

bool RelayBoard::writeI2cRegister(uint8_t device_address, uint8_t register_address, uint8_t value)
{
    Wire.beginTransmission(device_address);
    Wire.write(register_address);
    Wire.write(value);
    const uint32_t txn_start_us = micros();
    bool ok = (Wire.endTransmission(true) == 0);
    last_i2c_write_us_ = micros() - txn_start_us;
    if (!ok)
    {
        enqueueLog(LogCategory::I2c, 1, device_address);
        return false;
    }
    return true;
}

bool RelayBoard::applyRelayStates(uint8_t requested_state_mask)
{
    uint32_t now_ms = millis();

    // Init / retry decision map (the branches below implement exactly this):
    //
    //   uninitialized -- request != 0x00 -----------------> REFUSE (false)
    //                \- request == 0x00 -> attemptSafeInitialization
    //                       (latch THEN direction, order enforced;
    //                        retries throttled >= 200 ms after a failure)
    //   initialized ---- target == last written -----------> no-op (true)
    //                \- new target -> writeI2cRegister (a NEW transition
    //                       to 0x00 gets one immediate attempt; any other
    //                       target is throttled >= 200 ms while failing)
    //                       success -> last write status known (true)
    //                       failure -> last write status UNKNOWN,
    //                                  throttled retry (false)
    //
    // --- Uninitialized: only SAFE requests are allowed, and they run the
    // full init sequence (latch -> direction). Nonzero requests are
    // REFUSED: an expander whose pin direction was never successfully
    // configured must never be asked to energize anything.
    if (!relay_initialized_)
    {
        if (requested_state_mask != SAFE_RELAY_STATE)
        {
            return false;
        }
        if (consecutive_write_failures_ > 0 &&
            (now_ms - last_write_attempt_ms_) < RELAY_WRITE_RETRY_PERIOD_MS)
        {
            return false; // throttle init retries
        }
        last_write_attempt_ms_ = now_ms;
        last_attempted_mask_ = SAFE_RELAY_STATE;
        if (attemptSafeInitialization())
        {
            enqueueLog(LogCategory::RelayBoard, 3, 0);
            return true;
        }
        return false; // attemptSafeInitialization() counted the failures
    }

    // --- Initialized: normal actuation ---
    if (relay_state_known_ && requested_state_mask == applied_state_mask_)
    {
        return true;
    }

    // Rate limiting: ordinary target changes do NOT bypass fault retry
    // limits. Only a NEW transition to SAFE gets one immediate attempt —
    // safe transitions must never be delayed by past failures; subsequent
    // failed attempts of the same target remain rate-limited.
    bool new_safe_transition =
        (requested_state_mask == SAFE_RELAY_STATE) &&
        (requested_state_mask != last_attempted_mask_);
    if (!new_safe_transition && consecutive_write_failures_ > 0 &&
        (now_ms - last_write_attempt_ms_) < RELAY_WRITE_RETRY_PERIOD_MS)
    {
        return false;
    }

    last_attempted_mask_ = requested_state_mask;
    last_write_attempt_ms_ = now_ms;
    if (writeI2cRegister(TCA9554_DEVICE_ADDRESS, TCA9554_OUTPUT_REGISTER, requested_state_mask))
    {
        applied_state_mask_ = requested_state_mask;
        consecutive_write_failures_ = 0;
        relay_state_known_ = true;
        return true;
    }

    ++cumulative_write_failures_;   // diagnostics only
    ++consecutive_write_failures_;  // drives the retry throttle
    relay_state_known_ = false;     // cached mask no longer trustworthy
    enqueueLog(LogCategory::RelayBoard, 1, requested_state_mask);
    return false;
}

uint8_t RelayBoard::readDigitalInputs() const
{
    uint8_t states = 0x00;
    for (uint8_t index = 0; index < 8; ++index)
    {
        if (digitalRead(DIGITAL_INPUT_PINS[index]) == LOW)
        {
            states |= (1u << index); // active-low input pulled low = active
        }
    }
    return states;
}

void RelayBoard::showStatusRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // Argument order is the vendor-tested order for this board's LED (their
    // code passed green into the first data slot — likely to match the LED's
    // channel order). rgbLedWrite() is the non-deprecated rename of the
    // vendor's neopixelWrite() with an identical signature. Do not "fix"
    // the order without checking colors on real hardware.
    rgbLedWrite(STATUS_RGB_LED_PIN, green, red, blue);
}

void RelayBoard::setStatusLed(uint8_t red, uint8_t green, uint8_t blue, uint16_t blink_interval_ms)
{
    if (red == active_led_pattern_.red &&
        green == active_led_pattern_.green &&
        blue == active_led_pattern_.blue &&
        blink_interval_ms == active_led_pattern_.blink_interval_ms)
    {
        return; // unchanged — do not restart the blink phase
    }
    active_led_pattern_ = {red, green, blue, blink_interval_ms};
    led_blink_phase_started_ms_ = millis();
    led_in_dark_phase_ = false;
    showStatusRgb(red, green, blue);
}

// CRITICAL DECISION: beeps are one-shot requests. Sustained conditions
// cannot produce repeated beeps because callers request a beep only on a
// state TRANSITION — no audio spam during a long fault, which matters when
// the device sits next to the operator.
void RelayBoard::requestStatusBeep(uint32_t duration_ms)
{
    buzzer_active_ = true;
    buzzer_started_ms_ = millis();
    buzzer_duration_ms_ = duration_ms;
}

// Cancel a beep that no longer reflects reality: the cutoff was real (relays
// already went safe) but the slower debounced state never confirmed it —
// e.g. a switch bouncing through mid-zone, or a glitch that clears before
// FAILSAFE_CONFIRM_MS. The silence itself says "transient, watch it".
void RelayBoard::cancelStatusBeep()
{
    buzzer_active_ = false;
    ledcWrite(STATUS_BUZZER_PIN, 0);
}

void RelayBoard::serviceStatusIndicators(uint32_t now_ms)
{
    // --- RGB blink handling ---
    if (active_led_pattern_.blink_interval_ms == 0)
    {
        if (led_in_dark_phase_)
        {
            showStatusRgb(active_led_pattern_.red, active_led_pattern_.green,
                          active_led_pattern_.blue);
            led_in_dark_phase_ = false;
        }
    }
    else if (now_ms - led_blink_phase_started_ms_ >= active_led_pattern_.blink_interval_ms)
    {
        led_blink_phase_started_ms_ = now_ms;
        led_in_dark_phase_ = !led_in_dark_phase_;
        if (led_in_dark_phase_)
        {
            showStatusRgb(0, 0, 0);
        }
        else
        {
            showStatusRgb(active_led_pattern_.red, active_led_pattern_.green,
                          active_led_pattern_.blue);
        }
    }

    // --- Buzzer one-shot handling (elapsed-time compare: millis()-wrap safe) ---
    if (buzzer_active_)
    {
        if (now_ms - buzzer_started_ms_ >= buzzer_duration_ms_)
        {
            buzzer_active_ = false;
            ledcWrite(STATUS_BUZZER_PIN, 0);
        }
        else
        {
            ledcWrite(STATUS_BUZZER_PIN, BUZZER_DUTY_CYCLE);
        }
    }
    else
    {
        ledcWrite(STATUS_BUZZER_PIN, 0);
    }
}

} // namespace karr
