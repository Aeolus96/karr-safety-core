// ============================================================================
// K.A.R.R. (Keystone Autonomous Resource Rover) — safety & relay controller
// Firmware project: ESP32-RELAY-FW (e-stop + lighting relay baseline)
// ============================================================================
//
// WHAT THIS FIRMWARE DOES (baseline):
//   1. Reads a Turnigy iA6B RC receiver bound to a TGY-i6S transmitter
//      (i-BUS protocol, 10 channels).
//   2. Publishes the channels as a single sensor_msgs/Joy message over
//      micro-ROS (Ethernet / W5500 only — no WiFi, no Bluetooth, no MQTT).
//   3. Runs relay FEATURES locally: RC switches are mapped to relay
//      functions inside the firmware. Relays are NEVER commanded from ROS —
//      only observed. All control decisions stay in this file by design.
//   4. Publishes a 1 Hz heartbeat plus link / relay-state / digital-input /
//      e-stop diagnostics so device health is always visible from ROS.
//   5. Enforces local safety in TWO scopes (see BLOCK 4): BOARD-level —
//      the relay expander must complete its safe initialization before
//      any feature may energize; FEATURE-level — today's RC-driven
//      features go inactive on lost RC frames, the tested transmitter's
//      failsafe signature (1500 on all channels — "impossible" for a
//      2-position switch), the wireless e-stop, and by default a lost
//      micro-ROS agent (FORCE_SAFE_ON_AGENT_LOSS). "Safe" commands are
//      all-off (0x00) WRITES, not confirmed contact states. The firmware
//      never halts and recovers on its own.
//
// DESIGN RULES:
//   - Dual execution context (ESP32 FreeRTOS):
//     * Core 0 — Control task (pinned, priority configMAX_PRIORITIES-2):
//       samples i-BUS RC every 1 ms (IBusBM NOTIMER requirement), evaluates
//       safety features at 50 Hz (20 ms), writes relays, updates indicators.
//       Never blocks on micro-ROS.
//     * Core 1 — Arduino loop(): micro-ROS service + telemetry publishing.
//       May block (ping, entity lifecycle, reliable publish) but cannot
//       delay the control task.
//   - Shared state is synchronized, never lock-free: FreeRTOS queues with
//     a zero-wait producer (control never blocks on ROS; ROS may block and
//     then reads the newest snapshot) plus one atomic flag with
//     acquire/release ordering. Control task owns all hardware (UART1,
//     I2C); ROS task only reads snapshots for telemetry.
//   - Largest documented blocking operations are now confined to the ROS
//     context: agent ping <= 100 ms, one-shot time sync <= 100 ms, entity
//     create/destroy sessions 100/200 ms, reliable-publish budget 100 ms.
//     The control path runs a nominal 1 ms / 20 ms cadence; worst-case
//     service gap is NOT yet measured on hardware — the timing counters
//     report observed maxima, not a certified bound.
//   - RelayBoard is the single hardware facade. RelayFeatureController is
//     the only class allowed to write relay states, and it does so behind
//     board-level actuation constraints + per-feature dependency checks
//     (see block 4). Everything else only observes (see block 5).
//   - No micro-ROS subscriber exists: the executor/subscription pattern is
//     intentionally absent so no external topic can actuate hardware.
//   - Vendor (Waveshare) register-level code is kept where proven working
//     (TCA9554 / I2C transactions); vendor application logic (WiFi/MQTT/
//     RS485 examples, DIN->relay auto-copy, blocking network test code)
//     was deliberately not copied. See NOTES.md for the keep/drop table.
//
// NAMING CONVENTIONS (so every reader and every AI model knows at a glance
// what kind of identifier it is looking at):
//   - UPPER_SNAKE_CASE  : compile-time constants (obvious by shape —
//                         no cryptic 'k' prefix; the old Google-style
//                         kConstantName was dropped for readability)
//   - camelCase         : methods, function-local variables
//   - snake_case with trailing underscore : class members
//   - CamelCase         : classes, structs, enumerators (enum class)
//
// FILE STRUCTURE — each block below is a self-contained feature with its
// own constants attached. When the single-file test firmware is split into
// real modules later, each block moves to its own .h/.cpp pair unchanged:
//
//   BLOCK 1: RelayBoard               -> relay_board.h / relay_board.cpp
//            (hardware facade: relays, digital inputs, status LED, buzzer)
//   BLOCK 2: RcReceiver               -> rc_receiver.h / rc_receiver.cpp
//            (i-BUS receiver, channel table, failsafe state machine)
//   BLOCK 3: MicroRosBridge           -> micro_ros_bridge.h / .cpp
//            (agent link + publishers; publish-only by design)
//   BLOCK 4: RelayFeatureController   -> relay_feature_controller.h / .cpp
//            (ALL relay control logic: switch->feature bindings +
//             per-feature dependency model + board constraints)
//   BLOCK 5: main                     -> main.cpp (wiring + loop only)
// ============================================================================

#include <Arduino.h>

// --- Hardware buses ---
#include <Wire.h> // I2C: hosts the TCA9554 8-bit relay expander
#include <SPI.h>  // SPI: shared by the W5500 Ethernet controller
#include <ETH.h>  // link/IP status queries only; ETH.begin() itself is
                  // performed by the micro-ROS ethernet transport

// --- micro-ROS (Jazzy distribution, Ethernet transport) ---
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/init_options.h> // explicit ROS domain via init options
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h> // one-shot agent time sync on connect
#include <rmw_microros/timing.h>    // explicit entity create/destroy budgets
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>

// --- i-BUS RC receiver (LOCAL copy in lib/IBusBM, see platformio.ini) ---
#include <IBusBM.h>

// --- FreeRTOS (ESP32 Arduino core) — separates RC/safety from blocking ROS ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

// ============================================================================
// BLOCK 1: RelayBoard
// Hardware facade for the Waveshare ESP32-S3-POE-ETH-8DI-8RO-C board: the 8 relays
// (TCA9554 I2C expander), the 8 digital inputs, the status RGB LED and the
// buzzer. Everything hardware-specific lives behind this class; the feature
// layer (block 4) never touches Wire, pins, or registers directly.
// Split-ready: depends only on the Arduino core + Wire.
// ============================================================================

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

// --- Relay actuation behavior ---
constexpr uint8_t RELAY_COUNT = 8;
// CRITICAL DECISION: this board's fail-safe default is the all-off mask
// 0x00. A set output bit energizes a relay (vendor Set_EXIOS() semantics:
// bit=1 turns the channel ON), so 0x00 commands every relay off. SCOPE:
// 0x00 is an all-off COMMAND mask, not a universal definition of safe
// behavior for every future feature, and a successful write is not
// contact feedback. Verify against your wiring before trusting this in
// the field.
constexpr uint8_t SAFE_RELAY_STATE = 0x00;
constexpr uint32_t RELAY_WRITE_RETRY_PERIOD_MS = 200; // throttle I2C retries
                                                      // after a failed write
constexpr uint8_t RELAY_INIT_RETRY_COUNT = 3;         // boot-time init attempts
constexpr uint32_t RELAY_INIT_RETRY_PERIOD_MS = 50;   // between attempts

// --- Queued log event channel: control task enqueues with zero wait ---
// Control-task reachable code (RC service, feature evaluation, I2C writes)
// enqueues events here. loop() on core 1 drains the queue (bounded per
// iteration — see drainLogQueue) and writes Serial.
// Zero-wait send: if the queue fills, events are dropped and the counter
// increments. Logging never blocks the control path. The dropped counter
// is an atomic: written by control, snapshotted by loop() for diagnostics.
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

static QueueHandle_t g_log_queue = NULL;
static std::atomic<uint32_t> g_log_dropped_count{0};

static inline void enqueueLog(LogCategory category, uint8_t code, uint32_t value)
{
    if (!g_log_queue)
    {
        return;
    }
    LogEvent event{millis(), static_cast<uint8_t>(category), code, value};
    if (xQueueSend(g_log_queue, &event, 0) != pdTRUE)
    {
        g_log_dropped_count.fetch_add(1, std::memory_order_relaxed);
    }
}

// Bounded drain: at most DRAIN_MAX_EVENTS per loop() iteration so a
// sustained event burst cannot indefinitely postpone bridge servicing.
// Returns the number of events drained.
static uint32_t drainLogQueue()
{
    if (!g_log_queue)
    {
        return 0;
    }
    constexpr uint8_t DRAIN_MAX_EVENTS = 16;
    uint32_t drained = 0;
    LogEvent event;
    while (drained < DRAIN_MAX_EVENTS && xQueueReceive(g_log_queue, &event, 0) == pdTRUE)
    {
        const char *cat_text = "?";
        switch (static_cast<LogCategory>(event.category))
        {
        case LogCategory::RcReceiver:
            cat_text = "RC";
            break;
        case LogCategory::RelayBoard:
            cat_text = "RELAY";
            break;
        case LogCategory::FeatureController:
            cat_text = "SAFETY";
            break;
        case LogCategory::I2c:
            cat_text = "I2C";
            break;
        default:
            break;
        }
        Serial.printf("[LOG %lu %s c=%u v=%lu]\n",
                      (unsigned long)event.timestamp_ms, cat_text,
                      event.code, (unsigned long)event.value);
        ++drained;
    }
    return drained;
}

// --- Coherent telemetry snapshot: control task fills once per safety eval ---
// --- cycle (20 ms); loop() (core 1) copies it for publishing / status.      ---
// --- This replaces live cross-core reads of RcReceiver / RelayBoard.        ---
struct ControlSnapshot
{
    uint32_t captured_ms;
    uint32_t frame_sequence;
    uint32_t last_valid_frame_ms;
    uint16_t channels[16];     // RC_CHANNEL_COUNT is 10; IBusBM supports 14
    uint16_t signature_mask;   // 16-bit: bit N = channel N in narrow window
    uint8_t receiver_state;    // ReceiverOperatingState
    bool receiver_local_stop;  // authoritative stop, computed in control
    uint8_t receiver_stop_reasons; // RECV_STOP_* bits (receiver scope only)
    uint8_t desired_mask;
    uint8_t last_written_mask;
    uint8_t digital_inputs;
    bool relay_state_known;
};

static QueueHandle_t g_snapshot_queue = NULL;

static inline void publishSnapshot(const ControlSnapshot &snap)
{
    if (!g_snapshot_queue)
    {
        return;
    }
    // Length-one latest-value: overwrite replaces the queued item even when
    // full, so the consumer always gets the newest snapshot. Zero-wait for
    // the producer; ROS may block — its next consumed snapshot is current.
    xQueueOverwrite(g_snapshot_queue, &snap);
}

static inline bool receiveSnapshot(ControlSnapshot &snap)
{
    if (!g_snapshot_queue)
    {
        return false;
    }
    return xQueueReceive(g_snapshot_queue, &snap, 0) == pdTRUE;
}

// --- Shared state between control task (core 0) and ROS task (loop/core 1) ---
// Synchronized FreeRTOS queues with a zero-wait producer: the control task
// never blocks on the ROS task; the ROS task may block, and its next
// consumed snapshot is then the newest. The control task is the sole
// writer; the ROS task is the sole reader.
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

// ---------------------------------------------------------------------------
// RelayBoard implementation
// ---------------------------------------------------------------------------

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

// ============================================================================
// BLOCK 2: RcReceiver
// Turnigy iA6B receiver (TGY-i6S transmitter) over i-BUS + the
// receiver-facing safety state
// machine. Contains NO ROS types and NO board hardware access — split-ready
// into rc_receiver.h/.cpp with only the IBusBM library as a dependency.
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

// --- Receiver link + failsafe detection ---
// CRITICAL DECISION — receive pin: the tested prototype called
// Serial1.begin(..., RX=21) and then IBusBM.begin(Serial1, NOTIMER, rxPin=1),
// which re-begins UART1 internally — so the pin that actually ran was GPIO1.
// We reproduce the effective tested behavior; change to 21 if your wire
// lands there instead.
constexpr int RC_RECEIVE_PIN = 1;
constexpr uint8_t RC_CHANNEL_COUNT = 10; // channels in the tested i6S/iA6B i-BUS stream
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
// Receiver-local stop reason bits (snapshot + logs; receiver scope only —
// board/agent/config reasons stay distinguishable, see service()).
constexpr uint8_t RECV_STOP_SIGNAL_LOSS = 1u;
constexpr uint8_t RECV_STOP_FAILSAFE = 2u;
constexpr uint8_t RECV_STOP_CH6 = 4u;
constexpr uint8_t RECV_STOP_INVALID_INPUT = 8u;

// E-stop channel (ch6). TWO roles, both on this channel:
//   1. No-signal check: raw below NO_SIGNAL_RAW_VALUE = receiver unbound.
//   2. WIRELESS E-STOP SWITCH: LOW position (1000) ENGAGES the e-stop,
//      HIGH position (2000) RELEASES it. The RC controller boots with the
//      switch up (1000), so the board always starts in the e-stop state
//      and the operator must deliberately switch it off. RF loss and the
//      failsafe signature are the second (automatic) e-stop layer.
//      Never bind this channel to a feature — enforced at startup.
constexpr uint8_t ESTOP_CHANNEL_INDEX = 5; // ch6 (1-based) = index 5

// Switch ZONE boundaries (mapping/activation only — NOT failsafe detection).
//   LOW zone  : raw <  SWITCH_LOW_THRESHOLD   (position 1000)
//   MID zone  : SWITCH_LOW_THRESHOLD .. SWITCH_HIGH_THRESHOLD (reads INACTIVE)
//   HIGH zone : raw >  SWITCH_HIGH_THRESHOLD  (position 2000)
// The narrow FAILSAFE_CENTER +/- FAILSAFE_TOLERANCE window is the only
// signature evidence; ordinary MID samples are intermediate input.

// --- Channel configuration table -------------------------------------------
// >>> EDIT THIS TABLE to match your transmitter / mode setup <<<
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
// a feature (validateSwitchToRelayBindings() enforces this).
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

constexpr ReceiverChannelConfiguration RC_CHANNEL_CONFIGURATION[RC_CHANNEL_COUNT] = {
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

// --- Value mapping helpers (pure functions over the config table) -----------
float mapChannelValueToJoyAxis(uint16_t raw_value, const ReceiverChannelConfiguration &configuration);

// Switch ACTIVATION: TRUE position = LOW zone (1000) for both 2- and
// 3-position switches, per the board's switch polarity convention.
// Analog axes are never "activated".
bool channelIsActivated(uint16_t raw_value, const ReceiverChannelConfiguration &configuration);

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
    // Decode one raw sample into an endpoint category.
    static ChannelDecode decodeRaw(uint16_t raw_value)
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
    static bool inSignatureWindow(uint16_t raw_value)
    {
        return (raw_value >= FAILSAFE_CENTER - FAILSAFE_TOLERANCE) &&
               (raw_value <= FAILSAFE_CENTER + FAILSAFE_TOLERANCE);
    }

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

// ---------------------------------------------------------------------------
// RcReceiver implementation
// ---------------------------------------------------------------------------

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
                    // Intermediate (sweep through the MID zone, or resting
                    // in MID) or Implausible: retain the previous effective
                    // value through a brief sweep so switch bounce cannot
                    // chatter the relay, then resolve a SUSTAINED rest to
                    // canonical MID (1500 -> INACTIVE for every feature).
                    // Deliberately NOT the 2-pos semantics: there, grace
                    // expiry latches invalid input and stops; here MID is
                    // legitimate, so expiry resolves the VALUE instead.
                    // Timer starts on ENTRY and never restarts — same
                    // episode rules as the 2-pos path. The episode arrays
                    // are safe to share with the 2-pos path: channel kinds
                    // are disjoint per index, and both reset paths (long
                    // gap, signal loss) already clear them for every index.
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

// ============================================================================
// BLOCK 3: MicroRosBridge
// Agent connection management + telemetry publishers. PUBLISH-ONLY by
// design: there is deliberately no subscription and no executor, so no
// external ROS topic can ever actuate the relays. Split-ready into
// micro_ros_bridge.h/.cpp; it only touches the RelayBoard / RcReceiver
// public APIs (top integration layer).
// ============================================================================

// --- Network (static IP; agent must match your LAN) ---
const IPAddress BOARD_STATIC_IP(192, 168, 0, 24);
const IPAddress LOCAL_GATEWAY(192, 168, 0, 1);
const IPAddress LOCAL_SUBNET_MASK(255, 255, 255, 0);
const IPAddress MICRO_ROS_AGENT_IP(192, 168, 0, 4);
constexpr uint16_t MICRO_ROS_AGENT_PORT = 8888;
constexpr char BOARD_HOSTNAME[] = "estop_relay_board";
// ROS domain, applied explicitly at support init through rcl init options
// (rclc_support_init_with_options + rcl_init_options_set_domain_id).
constexpr size_t ROS_DOMAIN_ID = 0;

// --- W5500 Ethernet SPI pins (also set as build flags in platformio.ini,
//     which the transport reads from macros; repeated for SPI.begin()) ---
constexpr uint8_t ETHERNET_SPI_CLOCK_PIN = 15;
constexpr uint8_t ETHERNET_SPI_MISO_PIN = 14;
constexpr uint8_t ETHERNET_SPI_MOSI_PIN = 13;
constexpr uint8_t ETHERNET_CHIP_SELECT_PIN = 16;

// --- Topics (short chains; names carry the meaning) ---
constexpr char RECEIVER_JOY_TOPIC[] = "/rc/joy";          // OUT sensor_msgs/Joy
constexpr char RECEIVER_LINK_TOPIC[] = "/rc/link";        // OUT std_msgs/Bool
constexpr char BOARD_HEARTBEAT_TOPIC[] = "/relay_board/heartbeat"; // OUT UInt32 (s)
constexpr char BOARD_RELAY_STATE_TOPIC[] = "/relay_board/relay_state"; // OUT UInt8MultiArray
constexpr char BOARD_INPUTS_TOPIC[] = "/relay_board/inputs";     // OUT UInt8
constexpr char BOARD_ESTOP_TOPIC[] = "/relay_board/estop";       // OUT Bool (engaged)

// --- Cadences / agent ping budgets ---
constexpr uint32_t JOY_PUBLISH_PERIOD_MS = 20;        // 50 Hz
constexpr uint32_t HEALTH_PUBLISH_PERIOD_MS = 1000;   // 1 Hz
constexpr uint32_t AGENT_PING_PERIOD_CONNECTED_MS = 1000;
constexpr uint32_t AGENT_PING_PERIOD_WAITING_MS = 500;
constexpr uint32_t AGENT_PING_TIMEOUT_MS = 100;       // worst-case block
constexpr uint8_t AGENT_PING_ATTEMPTS = 1;
constexpr uint64_t TIME_SYNC_TIMEOUT_MS = 100; // one-shot sync budget (ms)
// Explicit micro-ROS session budgets (bounded blocking, per design rules).
constexpr int ENTITY_CREATION_SESSION_TIMEOUT_MS = 100; // agent handshake per entity
constexpr int ENTITY_DESTROY_SESSION_TIMEOUT_MS = 200; // teardown handshake
constexpr int RELIABLE_PUBLISHER_SESSION_TIMEOUT_MS = 100; // relay_state ack budget
constexpr uint32_t RELAY_STATE_RETRY_PERIOD_MS = 500;    // failed publish retry gap
constexpr uint32_t CLEANUP_RETRY_PERIOD_MS = 500;        // teardown retry cadence

class MicroRosBridge
{
public:
    // Sets up the SPI bus and the micro-ROS ethernet transport (which also
    // configures the W5500 with the static IP). Attaches static message
    // storage. Entity creation happens later, in service().
    void begin();

    // Explicit handle zero-initialization. Static globals are zeroed by
    // the C runtime, but the library is designed around explicit
    // zero-initialized handles (rcl_get_zero_initialized_*) — doing it in
    // the constructor keeps the class correct even if it is ever
    // instantiated differently (stack, array, re-created at runtime).
    MicroRosBridge();

    // Throttled agent ping + connection state machine + entity lifecycle.
    void service(uint32_t now_ms);

    bool isAgentConnected() const;

    // Human-readable agent state for the serial status line
    // (WAITING / AVAILABLE / CONNECTED / CLEANUP).
    const char *agentStateText() const;

    // Telemetry publishers (all no-ops while the agent is not connected).
    // Consume a coherent ControlSnapshot filled by the control task — no
    // live cross-core reads of RcReceiver / RelayBoard.
    void publishReceiverTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms);
    void publishBoardTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms);

private:
    bool initializeEntities();
    bool destroyEntities();
    void resetEntityHandles();
    // Finalizes one child publisher if its flag is set (no-op otherwise):
    // clears the flag on success, keeps it (preserving the parent) on
    // failure. Collapses the 6x fini/clear/log pattern; the call order in
    // destroyEntities stays explicit so the child-before-parent teardown
    // boundary is never hidden in a loop.
    void finalizePublisher(rcl_publisher_t &publisher, bool &created_flag, const char *name);
    bool publishTelemetryMessage(rcl_publisher_t *publisher, const void *message);

    // micro-ROS core
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;

    // publishers (no subscriber — see block header)
    rcl_publisher_t joy_publisher_;
    rcl_publisher_t link_publisher_;
    rcl_publisher_t heartbeat_publisher_;
    rcl_publisher_t relay_state_publisher_;
    rcl_publisher_t inputs_publisher_;
    rcl_publisher_t estop_publisher_;

    // lifecycle flags: the agent can drop at ANY time, so teardown must
    // only touch entities that were actually created.
    struct EntityFlags
    {
        bool support = false;
        bool node = false;
        bool joy_publisher = false;
        bool link_publisher = false;
        bool heartbeat_publisher = false;
        bool relay_state_publisher = false;
        bool inputs_publisher = false;
        bool estop_publisher = false;

        // Any child publisher still created? Used as the teardown gate: the
        // node is finalized only once every child has been released.
        bool anyPublishers() const
        {
            return joy_publisher || link_publisher || heartbeat_publisher ||
                   relay_state_publisher || inputs_publisher || estop_publisher;
        }

        // Any resource at all? Used as the "nothing to do" fast path.
        bool anyResources() const
        {
            return support || node || anyPublishers();
        }
    };
    EntityFlags entities_;

    // agent connection state machine
    enum class AgentState
    {
        WaitingForAgent, // pinging until the agent answers
        AgentAvailable,  // answered — next cycle creates entities
        AgentConnected,  // entities live, publishers active
        CleanupPending,  // agent lost OR entity init failed — entities must
                         // be torn down (possibly over several cycles)
                         // before the next connect attempt
    };
    AgentState agent_state_ = AgentState::WaitingForAgent;
    AgentState previous_agent_state_ = AgentState::WaitingForAgent;
    uint32_t last_ping_ms_ = 0;
    uint32_t last_cleanup_attempt_ms_ = 0; // teardown dispatch schedule
    // Persistent support-abandonment count: repeated abandonments (each a
    // possible local-allocation leak) stay visible across reconnect cycles.
    // Monotonic — never reset — so a reconnect-and-leak cycle cannot hide.
    uint32_t support_abandonment_count_ = 0;
    bool time_sync_pending_ = false; // set on connect; executed on a later
                                     // ping, never in the entity-creation call
    bool time_synced_ = false;       // set only after a SUCCESSFUL sync;
                                     // cleared on loss/teardown — Joy stamps
                                     // are populated only while this is true

    // publish cadence / change tracking
    uint32_t last_joy_publish_ms_ = 0;
    uint32_t last_health_publish_ms_ = 0;
    // /relay_board/relay_state (reliable QoS) publication tracking:
    //  - last_attempted_relay_state_ + relay_attempt_exists_ : the target of
    //    the last attempt (SUCCESS NOT REQUIRED — a failed attempt still sets
    //    the baseline, so only a NEW target bypasses the retry gap)
    //  - relay_state_publish_pending_ : last attempt FAILED (retry gap applies)
    //  - relay_state_publish_forced_ : fresh-connection snapshot request
    // ONE eligibility decision consumes these (see publishBoardTelemetry).
    uint8_t last_attempted_relay_state_ = SAFE_RELAY_STATE;
    bool relay_attempt_exists_ = false;
    bool relay_state_publish_pending_ = false;
    bool relay_state_publish_forced_ = false;
    uint32_t last_relay_state_publish_attempt_ms_ = 0;
    uint32_t publish_failure_count_ = 0;

    // messages + static storage for every sequence-bearing message.
    // In-class value-initialization ({}): nested fields such as
    // frame_id/layout are zeroed independently of storage duration, so the
    // class is also correct if ever instantiated on the stack.
    sensor_msgs__msg__Joy joy_message_ = {};
    std_msgs__msg__Bool link_message_ = {};
    std_msgs__msg__UInt32 heartbeat_message_ = {};
    std_msgs__msg__UInt8MultiArray relay_state_message_ = {};
    std_msgs__msg__UInt8 inputs_message_ = {};
    std_msgs__msg__Bool estop_message_ = {};
    float joy_axes_storage_[RC_CHANNEL_COUNT] = {};
    int32_t joy_buttons_storage_[RC_CHANNEL_COUNT] = {};
    uint8_t relay_state_storage_[RELAY_COUNT] = {};
};

// ---------------------------------------------------------------------------
// MicroRosBridge implementation
// ---------------------------------------------------------------------------

MicroRosBridge::MicroRosBridge()
{
    allocator_ = rcl_get_default_allocator();
    resetEntityHandles(); // single zero-init source — safe pre-publication:
                          // nothing is live yet, so no flag check is needed
}

void MicroRosBridge::begin()
{
    SPI.begin(ETHERNET_SPI_CLOCK_PIN, ETHERNET_SPI_MISO_PIN,
              ETHERNET_SPI_MOSI_PIN, ETHERNET_CHIP_SELECT_PIN);

    // CRITICAL DECISION: set_microros_ethernet_transports() performs
    // ETH.begin() itself using the ETH_PHY_* build flags from platformio.ini,
    // configures the static IP, and points the XRCE-DDS session at the
    // agent. The SPI bus is pre-initialized with the same pins so the
    // transport and any later ETH status queries share one bus setup.
    set_microros_ethernet_transports(
        BOARD_STATIC_IP, LOCAL_GATEWAY, LOCAL_SUBNET_MASK,
        MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, BOARD_HOSTNAME);
    Serial.printf("[ETH] static IP %s, agent %s:%u\n",
                  BOARD_STATIC_IP.toString().c_str(),
                  MICRO_ROS_AGENT_IP.toString().c_str(),
                  MICRO_ROS_AGENT_PORT);

    // Attach static storage to every sequence-bearing message exactly once.
    // From here on, publishing only ever updates `size` — micro-ROS must
    // not reallocate sequence buffers during steady-state operation.
    joy_message_.axes.data = joy_axes_storage_;
    joy_message_.axes.capacity = RC_CHANNEL_COUNT;
    joy_message_.axes.size = 0;
    joy_message_.buttons.data = joy_buttons_storage_;
    joy_message_.buttons.capacity = RC_CHANNEL_COUNT;
    joy_message_.buttons.size = 0;

    relay_state_message_.data.data = relay_state_storage_;
    relay_state_message_.data.capacity = RELAY_COUNT;
    relay_state_message_.data.size = 0;
}

bool MicroRosBridge::isAgentConnected() const
{
    return agent_state_ == AgentState::AgentConnected;
}

const char *MicroRosBridge::agentStateText() const
{
    switch (agent_state_)
    {
    case AgentState::WaitingForAgent:
        return "WAITING";
    case AgentState::AgentAvailable:
        return "AVAILABLE";
    case AgentState::AgentConnected:
        return "CONNECTED";
    case AgentState::CleanupPending:
        return "CLEANUP";
    }
    return "?";
}

bool MicroRosBridge::publishTelemetryMessage(rcl_publisher_t *publisher, const void *message)
{
    // CRITICAL DECISION: publish results are propagated, not discarded.
    // Publication failures are counted. Best-effort telemetry avoids
    // acknowledgement waits. Transport execution time and reliable
    // relay-state publication remain part of the loop timing budget.
    if (rcl_publish(publisher, message, nullptr) != RCL_RET_OK)
    {
        ++publish_failure_count_;
        return false;
    }
    return true;
}

bool MicroRosBridge::initializeEntities()
{
    allocator_ = rcl_get_default_allocator();

    // Domain comes from ROS_DOMAIN_ID through rcl init options (the only
    // domain path: rclc_support_init takes no domain parameter). Same
    // sequence as the micro_ros_platformio ethernet_pubsub example. init
    // options are ALWAYS finalized before return — rclc_support_init copies
    // what it needs, so holding them open across entity creation would leak.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator_) != RCL_RET_OK)
    {
        Serial.println("[ROS] init options init failed");
        return false;
    }
    // Domain configuration is explicit. Init options are local resources —
    // report finalization failure without losing track of initialized
    // support: support_ stays valid even if the fini below fails.
    rcl_ret_t support_result = RCL_RET_ERROR;
    if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK)
    {
        Serial.println("[ROS] domain id config failed");
    }
    else
    {
        support_result = rclc_support_init_with_options(&support_, 0, nullptr, &init_options, &allocator_);
        if (support_result != RCL_RET_OK)
        {
            Serial.println("[ROS] support init failed");
        }
    }
    if (rcl_init_options_fini(&init_options) != RCL_RET_OK)
    {
        // Local options leak only — support, when initialized, is unaffected.
        Serial.println("[ROS] WARNING: init options fini failed (support state kept)");
    }
    if (support_result != RCL_RET_OK)
    {
        return false;
    }
    entities_.support = true;

    // Explicit creation budget while the context is valid: bounds how long
    // each entity-creation handshake can block when the agent is slow.
    // REQUIRED timing configuration is part of successful initialization —
    // do not report the requested budget as active if its setup failed.
    rmw_ret_t creation_timeout_result =
        rmw_uros_set_context_entity_creation_session_timeout(
            rcl_context_get_rmw_context(&support_.context),
            ENTITY_CREATION_SESSION_TIMEOUT_MS);
    if (creation_timeout_result != RMW_RET_OK)
    {
        Serial.printf("[ROS] REQUIRED entity-creation timeout config failed: %d\n",
                      creation_timeout_result);
        return false;
    }

    if (rclc_node_init_default(&node_, "estop_relay_controller", "", &support_) != RCL_RET_OK)
    {
        Serial.println("[ROS] node init failed");
        return false;
    }
    entities_.node = true;

    // Telemetry publishers use BEST-EFFORT QoS: stale data is worthless and
    // a reliable publisher with a full history can block in rcl_publish.
    if (rclc_publisher_init_best_effort(&joy_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
                                        RECEIVER_JOY_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] joy publisher init failed");
        return false;
    }
    entities_.joy_publisher = true;

    if (rclc_publisher_init_best_effort(&link_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                        RECEIVER_LINK_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] link publisher init failed");
        return false;
    }
    entities_.link_publisher = true;

    if (rclc_publisher_init_best_effort(&heartbeat_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
                                        BOARD_HEARTBEAT_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] heartbeat publisher init failed");
        return false;
    }
    entities_.heartbeat_publisher = true;

    // Relay state uses DEFAULT (reliable) QoS: consumers may care about
    // every transition of a safety-relevant output. The publisher's
    // acknowledgement budget is REQUIRED timing configuration — without it
    // a reliable publish could block unboundedly, so init fails if it
    // cannot be configured.
    if (rclc_publisher_init_default(&relay_state_publisher_, &node_,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                                    BOARD_RELAY_STATE_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] relay state publisher init failed");
        return false;
    }
    entities_.relay_state_publisher = true;

    rmw_ret_t publisher_timeout_result = rmw_uros_set_publisher_session_timeout(
        rcl_publisher_get_rmw_handle(&relay_state_publisher_),
        RELIABLE_PUBLISHER_SESSION_TIMEOUT_MS);
    if (publisher_timeout_result != RMW_RET_OK)
    {
        Serial.printf("[ROS] REQUIRED relay-state publisher timeout config failed: %d\n",
                      publisher_timeout_result);
        return false;
    }

    if (rclc_publisher_init_best_effort(&inputs_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                                        BOARD_INPUTS_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] inputs publisher init failed");
        return false;
    }
    entities_.inputs_publisher = true;

    if (rclc_publisher_init_best_effort(&estop_publisher_, &node_,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                        BOARD_ESTOP_TOPIC) != RCL_RET_OK)
    {
        Serial.println("[ROS] estop publisher init failed");
        return false;
    }
    entities_.estop_publisher = true;

    Serial.println("[ROS] entities initialized — node 'estop_relay_controller' up");
    return true;
}

// ---------------------------------------------------------------------------
// resetEntityHandles — explicit re-zeroing of every entity handle. Called
// ONLY when no flag is set (nothing live) or immediately after a terminal
// support-fini abandon (see destroyEntities): overwriting potentially-live
// handles would leak agent resources without crashing, which is NOT
// recovery — the flags decide when this is safe.
// ---------------------------------------------------------------------------
void MicroRosBridge::resetEntityHandles()
{
    // rclc_support_t has no public zero-init accessor; value-initialize it.
    support_ = rclc_support_t{};
    node_ = rcl_get_zero_initialized_node();
    joy_publisher_ = rcl_get_zero_initialized_publisher();
    link_publisher_ = rcl_get_zero_initialized_publisher();
    heartbeat_publisher_ = rcl_get_zero_initialized_publisher();
    relay_state_publisher_ = rcl_get_zero_initialized_publisher();
    inputs_publisher_ = rcl_get_zero_initialized_publisher();
    estop_publisher_ = rcl_get_zero_initialized_publisher();
}

// ---------------------------------------------------------------------------
// finalizePublisher — one child publisher's fini/clear/log step, guarded by
// its flag (no-op when already released). See the destroyEntities failure
// policy below for why the flag is kept on failure.
// ---------------------------------------------------------------------------
void MicroRosBridge::finalizePublisher(rcl_publisher_t &publisher, bool &created_flag, const char *name)
{
    if (!created_flag)
    {
        return;
    }
    rcl_ret_t fini_result = rcl_publisher_fini(&publisher, &node_);
    if (fini_result == RCL_RET_OK)
    {
        created_flag = false;
    }
    else
    {
        Serial.printf("[ROS] %s publisher fini failed: %d\n", name, fini_result);
    }
}

// ---------------------------------------------------------------------------
// destroyEntities — safe teardown, possibly spanning multiple calls.
//
// CRITICAL ORDERING: destroy CHILDREN BEFORE THEIR OWNERS (publishers ->
// node -> support). rcl_publisher_fini()/rcl_node_fini() require their
// parent handles to still be valid, and the context must stay alive until
// all entity cleanup completes. The destroy-timeout is configured FIRST,
// while the context is still valid.
//
// Failure policy (NO attempt counter — stage-specific, per the finalizer
// semantics verified in the installed micro_ros_platformio source):
//  - publisher/node fini: success clears the flag, failure KEEPS it and the
//    parent is preserved; the next call retries. rcl_publisher_fini /
//    rcl_node_fini free and NULL their impl EVEN ON FAILURE, and a repeat
//    call on the NULLed handle returns RCL_RET_OK — so a failed child
//    normally self-heals in exactly one extra cycle (never a wedge; the
//    NULLed impl is re-init-safe, which is also the standard micro-ROS
//    reconnection pattern).
//  - support fini: SINGLE attempt, then terminal. rclc_support_fini (as
//    shipped in this micro_ros_platformio revision) runs its three
//    sub-finalizers unconditionally in one call, and a repeat after a
//    failure returns an error for the already-shut-down context
//    (rcl_shutdown re-entry returns ALREADY_SHUTDOWN/INVALID) — so
//    retrying cannot clear the error. On failure the handles are
//    abandoned immediately: the state machine still proceeds, and the log
//    states plainly that this is NOT a clean teardown (the agent-side
//    session may linger until the agent restarts; any local allocation
//    the partial cleanup did not free stays until reboot — forced
//    abandonment is honesty, not recovery). Repeated abandonments are
//    counted in support_abandonment_count_ and stay visible in the serial
//    log. Abandonment is NOT indefinitely leak-free recovery: no automatic
//    reboot — a board reboot must not become the default answer to a
//    ROS-only failure. If hardware testing ever shows the abandonment
//    count climbing, the planned escalation is SUSPENDING ROS
//    reconnection in a visible degraded state while local input sampling
//    and non-ROS features continue. That decision needs hardware
//    failure-rate data first.
// Returns false while any entity remains live so the caller (the
// CleanupPending state) retries — never blindly overwrite live resources
// and call that recovery.
// ---------------------------------------------------------------------------
bool MicroRosBridge::destroyEntities()
{
    if (!entities_.anyResources())
    {
        // Nothing live (e.g. init failed at the very first step). Ensure a
        // clean slate for the next init attempt — safe because no flag is set.
        resetEntityHandles();
        time_sync_pending_ = false;
        time_synced_ = false;
        return true;
    }

    // Configure the destroy budget FIRST, while the context is still valid.
    if (entities_.support)
    {
        rmw_ret_t timing_result = rmw_uros_set_context_entity_destroy_session_timeout(
            rcl_context_get_rmw_context(&support_.context),
            ENTITY_DESTROY_SESSION_TIMEOUT_MS);
        if (timing_result != RMW_RET_OK)
        {
            // A mis-configured budget undermines the teardown-time contract —
            // say so rather than discarding it as [[maybe_unused]].
            Serial.printf("[ROS] destroy timeout config failed: %d\n", timing_result);
        }
    }

    // Teardown failures are recorded, not silently ignored: each fini that
    // succeeds clears its flag; each failure keeps it (retry next call).
    // Explicit per-publisher calls (NOT a loop/framework): the child-before-
    // parent ordering is a safety boundary and must stay visible here.
    // Each call is a no-op unless its flag is set (guard inside the helper).
    finalizePublisher(joy_publisher_, entities_.joy_publisher, "joy");
    finalizePublisher(link_publisher_, entities_.link_publisher, "link");
    finalizePublisher(heartbeat_publisher_, entities_.heartbeat_publisher, "heartbeat");
    finalizePublisher(relay_state_publisher_, entities_.relay_state_publisher, "relay state");
    finalizePublisher(inputs_publisher_, entities_.inputs_publisher, "inputs");
    finalizePublisher(estop_publisher_, entities_.estop_publisher, "estop");

    // A failed child cleanup prevents owner cleanup: preserve the node
    // until EVERY publisher has been released. Destroying the node while a
    // publisher flag is still set would make the next cleanup call run
    // rcl_publisher_fini() against an already-finalized node.
    if (entities_.anyPublishers())
    {
        Serial.println("[ROS] teardown incomplete — node preserved, retry next cycle");
        return false;
    }

    // Only now attempt node cleanup.
    if (entities_.node)
    {
        rcl_ret_t fini_result = rcl_node_fini(&node_);
        if (fini_result == RCL_RET_OK) entities_.node = false;
        else Serial.printf("[ROS] node fini failed: %d\n", fini_result);
    }

    // Do not finalize support while the node remains.
    if (entities_.node)
    {
        Serial.println("[ROS] teardown incomplete — support preserved, retry next cycle");
        return false;
    }

    if (entities_.support)
    {
        rcl_ret_t fini_result = rclc_support_fini(&support_);
        if (fini_result == RCL_RET_OK)
        {
            entities_.support = false;
        }
        else
        {
            // TERMINAL (see the failure policy above): no retry — for this
            // library revision re-entry cannot clear the error. Abandon the
            // handles for a clean-slate re-init and say exactly what this
            // is NOT: NOT successful cleanup.
            Serial.printf("[ROS] support fini failed (%d) — abandoning handles "
                          "(agent-side entities may linger until the agent restarts; "
                          "any unfreed local allocation stays until reboot)\n",
                          fini_result);
            resetEntityHandles();
            entities_ = EntityFlags{};
            time_sync_pending_ = false;
            time_synced_ = false;
            ++support_abandonment_count_;
            Serial.printf("[ROS] entities abandoned after failed support fini "
                          "(total abandonments: %lu)\n",
                          (unsigned long)support_abandonment_count_);
            return true;
        }
    }

    resetEntityHandles(); // clean slate for the next init
    time_sync_pending_ = false;
    time_synced_ = false;
    Serial.println("[ROS] entities destroyed");
    return true;
}

void MicroRosBridge::service(uint32_t now_ms)
{
    // CRITICAL DECISION (operation ordering):
    //  1. CLEANUP dispatch runs FIRST, on its own schedule, with NO
    //     discovery ping — teardown does not need agent discovery.
    //  2. Detecting agent loss NEVER performs teardown in the same call:
    //     the call returns right after marking CleanupPending so the
    //     feature layer re-evaluates its dependencies FIRST (with
    //     FORCE_SAFE_ON_AGENT_LOSS=true that commands every current
    //     RC-driven feature to 0x00) BEFORE any bounded destroy work
    //     (200 ms budget, several entities) runs.
    //  3. A failed entity creation also defers teardown to a later call
    //     (CleanupPending) — creation failure and destruction never stack
    //     in one invocation.
    //  4. The one-shot time sync runs only on a LATER call than the one
    //     that created the entities, and only while still connected —
    //     never on a destroyed context.
    //
    // Lifecycle map (loop() keeps sampling RC and evaluating features in
    // EVERY state — this state machine only owns the ROS side):
    //
    //   WaitingForAgent --ping ok--> AgentAvailable --created--> AgentConnected
    //         ^                          |                          |
    //         |                          | creation failed          | ping fails:
    //         |                          v                          v  LOSS DETECTED
    //         +---- destroyEntities ---+-> CleanupPending <---------+ ONLY
    //              (500 ms cadence,        (teardown deferred one call:
    //               no discovery ping)      features re-evaluated first)
    if (agent_state_ == AgentState::CleanupPending)
    {
        if (now_ms - last_cleanup_attempt_ms_ >= CLEANUP_RETRY_PERIOD_MS)
        {
            last_cleanup_attempt_ms_ = now_ms;
            if (destroyEntities())
            {
                agent_state_ = AgentState::WaitingForAgent;
                Serial.println("[ROS] waiting for micro-ROS agent...");
                previous_agent_state_ = agent_state_;
            }
            // else: stay CleanupPending and retry on the next schedule tick
        }
        return; // no discovery ping while cleaning up
    }

    uint32_t ping_period = isAgentConnected() ? AGENT_PING_PERIOD_CONNECTED_MS
                                              : AGENT_PING_PERIOD_WAITING_MS;
    if (now_ms - last_ping_ms_ < ping_period)
    {
        return; // not due yet
    }
    last_ping_ms_ = now_ms;

    // Entry state: prevents entity creation and time sync from stacking
    // in the same call (sync must happen on a LATER call than creation).
    const bool connected_at_entry = isAgentConnected();

    bool agent_reachable =
        (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK);

    switch (agent_state_)
    {
    case AgentState::WaitingForAgent:
        if (agent_reachable)
        {
            agent_state_ = AgentState::AgentAvailable;
        }
        break;

    case AgentState::AgentAvailable:
        if (agent_reachable && initializeEntities())
        {
            agent_state_ = AgentState::AgentConnected;
        }
        else
        {
            // Creation failed: defer teardown to the cleanup dispatch —
            // never stack failed creation and destruction in one call.
            agent_state_ = AgentState::CleanupPending;
        }
        break;

    case AgentState::AgentConnected:
        if (!agent_reachable)
        {
            // LOSS DETECTED ONLY — teardown is deferred so ROS-dependent
            // features go inactive FIRST (the loop re-evaluates feature
            // permissions immediately after this call returns; with
            // FORCE_SAFE_ON_AGENT_LOSS=true that is every current feature).
            agent_state_ = AgentState::CleanupPending;
        }
        break;

    case AgentState::CleanupPending:
        // Handled by the dispatch above; unreachable here.
        break;
    }

    // Transition handling
    if (agent_state_ != previous_agent_state_)
    {
        switch (agent_state_)
        {
        case AgentState::WaitingForAgent:
            Serial.println("[ROS] waiting for micro-ROS agent...");
            break;
        case AgentState::AgentConnected:
            Serial.println("[ROS] agent connected");
            time_sync_pending_ = true;            // executed on a LATER call
            relay_state_publish_forced_ = true;   // force a fresh snapshot
            break;
        case AgentState::CleanupPending:
            Serial.println("[ROS] AGENT LOST / init failed — teardown pending; ROS-dependent features to 0x00 (FORCE_SAFE_ON_AGENT_LOSS)");
            break;
        default:
            break;
        }
        previous_agent_state_ = agent_state_;
    }

    // One-shot time sync: BOTH entry and current state must be connected.
    // Entry state prevents entity creation and sync from stacking in one
    // call; current state prevents syncing after loss detection/teardown.
    if (connected_at_entry && isAgentConnected() && time_sync_pending_)
    {
        time_sync_pending_ = false;
        if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) == RMW_RET_OK)
        {
            time_synced_ = true;
            Serial.println("[ROS] device clock synced with agent");
        }
        else
        {
            time_synced_ = false;
            Serial.println("[ROS] time sync unavailable (OK — Joy stamps stay zero)");
        }
    }
}

void MicroRosBridge::publishReceiverTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms)
{
    if (!isAgentConnected())
    {
        return;
    }
    if (now_ms - last_joy_publish_ms_ < JOY_PUBLISH_PERIOD_MS)
    {
        return;
    }
    last_joy_publish_ms_ = now_ms;

    // Receiver health from the coherent snapshot: SignalActive means fresh
    // frames and no failsafe signature. While unhealthy we still publish
    // — but zeroed — so consumers can tell "board alive, RC data neutral"
    // apart from "board dead" using the heartbeat + /rc/link pair.
    bool receiver_healthy = (snapshot.receiver_state ==
                             static_cast<uint8_t>(ReceiverOperatingState::SignalActive));
    uint8_t channel_count = RC_CHANNEL_COUNT;
    for (uint8_t index = 0; index < channel_count; ++index)
    {
        uint16_t raw_value = snapshot.channels[index];
        const ReceiverChannelConfiguration &configuration = RC_CHANNEL_CONFIGURATION[index];
        joy_axes_storage_[index] =
            receiver_healthy ? mapChannelValueToJoyAxis(raw_value, configuration) : 0.0f;
        joy_buttons_storage_[index] =
            (receiver_healthy && channelIsActivated(raw_value, configuration)) ? 1 : 0;
    }
    joy_message_.axes.size = channel_count;
    joy_message_.buttons.size = channel_count;

    // Timestamp: zero means "unsynchronized" — populated ONLY after a
    // successful agent time sync (time_synced_) with a positive epoch.
    // Safety timeouts never use this clock.
    joy_message_.header.stamp = {};
    if (time_synced_)
    {
        int64_t epoch_nanoseconds = rmw_uros_epoch_nanos();
        if (epoch_nanoseconds > 0)
        {
            joy_message_.header.stamp.sec = static_cast<int32_t>(epoch_nanoseconds / 1000000000LL);
            joy_message_.header.stamp.nanosec =
                static_cast<uint32_t>(epoch_nanoseconds % 1000000000LL);
        }
    }

    publishTelemetryMessage(&joy_publisher_, &joy_message_);
}

void MicroRosBridge::publishBoardTelemetry(const ControlSnapshot &snapshot, uint32_t now_ms)
{
    if (!isAgentConnected())
    {
        return;
    }

    // Relay state (reliable QoS) has ONE authoritative eligibility decision
    // — the periodic health tick must NOT bypass the retry gap:
    //   fresh-connection snapshot   -> attempt immediately
    //   new target vs last ATTEMPTED -> attempt immediately
    //   same target as a failed attempt -> enforce RELAY_STATE_RETRY_PERIOD_MS
    //   periodic refresh            -> only when no pending failure
    // NOTE: latest-state SNAPSHOTS, not a guaranteed event history — a
    // failed transition can be superseded by newer state even with
    // reliable QoS.
    uint8_t relay_states = snapshot.last_written_mask;
    bool periodic_due = (now_ms - last_health_publish_ms_) >= HEALTH_PUBLISH_PERIOD_MS;
    bool relay_new_target =
        !relay_attempt_exists_ || (relay_states != last_attempted_relay_state_);
    bool relay_retry_allowed = !relay_state_publish_pending_ ||
        (now_ms - last_relay_state_publish_attempt_ms_) >= RELAY_STATE_RETRY_PERIOD_MS;

    bool relay_should_attempt;
    if (relay_state_publish_forced_ || relay_new_target)
    {
        relay_should_attempt = true; // fresh snapshot / new target: immediate
    }
    else if (relay_state_publish_pending_)
    {
        relay_should_attempt = relay_retry_allowed; // same failed target: gap
    }
    else
    {
        relay_should_attempt = periodic_due; // periodic refresh
    }

    if (!periodic_due && !relay_should_attempt)
    {
        return;
    }

    if (periodic_due)
    {
        last_health_publish_ms_ = now_ms;

        heartbeat_message_.data = now_ms / 1000; // uptime seconds: a stalled
        publishTelemetryMessage(&heartbeat_publisher_, &heartbeat_message_); // counter = wedged device

        // SEMANTICS: /rc/link = frame-state liveness (fresh frames, no
        // CONFIRMED failsafe signature) — NOT permission to operate; the
        // decoded CH6 stop and persistent invalid input can still stop
        // locally.
        link_message_.data = (snapshot.receiver_state ==
                              static_cast<uint8_t>(ReceiverOperatingState::SignalActive));
        publishTelemetryMessage(&link_publisher_, &link_message_);

        inputs_message_.data = snapshot.digital_inputs;
        publishTelemetryMessage(&inputs_publisher_, &inputs_message_);

        // SEMANTICS: /relay_board/estop = authoritative receiver-local stop,
        // computed once in control and copied here — never rebuilt from raw
        // fields. Receiver scope only; board/agent/config reasons are
        // reported separately (LED, status line, reason-code logs).
        estop_message_.data = snapshot.receiver_local_stop;
        publishTelemetryMessage(&estop_publisher_, &estop_message_);
    }

    if (relay_should_attempt)
    {
        for (uint8_t index = 0; index < RELAY_COUNT; ++index)
        {
            relay_state_storage_[index] = (relay_states >> index) & 0x01;
        }
        relay_state_message_.data.size = RELAY_COUNT;
        last_attempted_relay_state_ = relay_states;
        relay_attempt_exists_ = true;
        relay_state_publish_forced_ = false;
        last_relay_state_publish_attempt_ms_ = now_ms;
        if (publishTelemetryMessage(&relay_state_publisher_, &relay_state_message_))
        {
            relay_state_publish_pending_ = false;
        }
        else
        {
            relay_state_publish_pending_ = true; // retried at the retry cadence
        }
    }
}

// ============================================================================
// BLOCK 4: RelayFeatureController
// ALL relay control logic lives here. RC switches are mapped to relay
// FEATURES; nothing from ROS can command a relay — features receive inputs
// (RC channels today, later digital inputs / other sensors) and manage the
// relays through the RelayBoard. Split-ready into
// relay_feature_controller.h/.cpp.
//
// LAYERING NOTE (answer to "should features connect to a RelayBoard
// instance?"): YES — the board is the dependency sink. The feature layer
// holds RelayBoard& and RcReceiver& references, so dependencies flow
// RelayBoard <- RcReceiver + Bridge <- Controller <- main. The graph stays
// acyclic, every block is independently testable and splittable, and the
// board itself knows nothing about ROS or features.
//
// DEPENDENCY MODEL (board-wide restrictions vs feature dependencies):
//   BOARD-LEVEL actuation constraint: relay hardware completed its safe
//     initialization — applies to EVERY feature; RelayBoard additionally
//     refuses nonzero writes while uninitialized, so this cannot be
//     bypassed by feature code.
//   FEATURE-GROUP dependencies: each group declares its own (RC health,
//     digital-input validity, ROS availability, a defined stop condition)
//     and its own behavior when a dependency is unavailable. Today there
//     is exactly ONE group — the RC-driven bindings below — and it keeps
//     full receiver-local inhibition; nothing else is implemented yet
//     (binding names are type placeholders, see the table).
//   Composition: each group evaluates ITS dependencies, ORs the bits it
//     requests into the shared mask; board constraints are applied
//     centrally AFTER combination; this class stays the single writer.
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
//   The OR-combination below is a SHARING rule (any owning feature may
//   request ON), NOT an arbitration policy for conflicting priorities —
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

// Persisted result of the startup validation OF THE RC BINDING TABLE.
// While false, RC-driven feature operation is INHIBITED (their bits stay
// 0x00) — an invalid configuration must not run a half-broken mapping on
// a safety device. CONFIG-FAULT SCOPE: this verdict covers the RC binding
// table ONLY; it is a broken FEATURE configuration, not a board-wide
// configuration problem — future feature groups (digital inputs,
// independent indications) validate their own configuration and are not
// inhibited by an unrelated RC-map error.
bool switch_binding_configuration_valid = true;

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

class RelayFeatureController
{
public:
    RelayFeatureController(RelayBoard &board, RcReceiver &receiver)
        : relay_board_(board), receiver_(receiver) {}

    // Evaluates each feature group's own dependencies, combines the
    // resulting relay requests, applies board-level constraints and
    // performs the single relay write. agent_connected is injected by
    // main() so this class stays decoupled from the micro-ROS layer.
    // (No clock input: these decisions are purely combinational on the
    // latest receiver/board state.)
    void service(bool agent_connected);
    // Last desired (target) relay mask computed by service() — for the
    // coherent telemetry snapshot. Differs from lastWrittenRelayStates()
    // when an I2C write failed.
    uint8_t lastDesiredMask() const { return last_desired_mask_; }

private:
    // BOARD-LEVEL actuation constraint — applies to every feature, no
    // exceptions (see the block header's dependency model).
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
    // constraints — evaluated in exactly the order of the block-header
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
    // Last desired relay target computed by service() — updated every
    // safety evaluation cycle for the coherent snapshot.
    uint8_t last_desired_mask_ = SAFE_RELAY_STATE;
};

// ---------------------------------------------------------------------------
// RelayFeatureController implementation
// ---------------------------------------------------------------------------

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
    if (!switch_binding_configuration_valid)
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
    //    it requests into the shared mask (see block-header diagram).
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
    // --- Per-scope dependency evaluation (see block-header diagram) ---
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
            if (!switch_binding_configuration_valid) reason_code |= 8u;
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
    else if (!switch_binding_configuration_valid)
    {
        relay_board_.setStatusLed(180, 0, 255, 1000); // purple, slow blink: invalid RC binding config
    }
    else
    {
        relay_board_.setStatusLed(0, 255, 0, 0); // green, solid: no policy condition detected (not contact verification)
    }
}

// ---------------------------------------------------------------------------
// validateSwitchToRelayBindings — startup sanity check of the RC BINDING
// TABLE (the table is meant to be hand-edited; catch mistakes early).
// Per-binding verdicts come from the single checkBinding() rule set (logs
// the FIRST failure per binding — still invalid either way); the
// double-bound scan stays here because it is a TABLE-level property
// (warn only — intentional sharing may be legitimate; see RELAY OWNERSHIP
// in the block header: OR-combination is a sharing rule, not arbitration).
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

// ============================================================================
// BLOCK 5: main — wiring and the cooperative loop (bounded-blocking ops,
// see DESIGN RULES above)
// ============================================================================

// System objects (construction order is harmless: only references are held).
RelayBoard relay_board;
RcReceiver rc_receiver;
MicroRosBridge micro_ros_bridge;
RelayFeatureController relay_feature_controller(relay_board, rc_receiver);

// --- Shared state between control task (core 0) and ROS task (loop/core 1) ---
// Synchronized handoff: the atomic agent flag is written by loop() and read
// by control with acquire/release ordering. Snapshots flow the other way
// via the overwrite queue above.
std::atomic<bool> g_agent_connected{false};
static TaskHandle_t g_control_task_handle = NULL; // set by setup(), read for diagnostics

// Control task: owns RC + safety + relays. Runs independently of micro-ROS.
// Pinned to core 0 so RC sampling and relay writes are never delayed by
// blocking ROS operations (ping, entity lifecycle, reliable publish).
static void controlTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    constexpr TickType_t kTickPeriodMs = pdMS_TO_TICKS(1); // 1 ms base cadence
    constexpr uint32_t kTimingLogPeriodMs = 1000; // log timing stats once/s

    uint32_t last_safety_ms = 0;
    uint32_t last_timing_log_ms = 0;
    uint32_t last_service_start_us = 0;
    uint32_t max_service_gap_us = 0;   // start-to-start, micros clock
    uint32_t max_feature_eval_us = 0;  // work duration, sleep excluded
    uint32_t max_i2c_write_us = 0;     // actual bus transaction (getter)
    uint32_t max_lateness_us = 0;      // wake lateness vs intended schedule
    uint32_t service_count = 0;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, kTickPeriodMs);
        // Work starts HERE — after the deliberate sleep. Everything below
        // measures execution, never sleep.
        const uint32_t work_start_us = micros();
        // Scheduling lateness: how far past the intended wake tick we ran.
        // xLastWakeTime was already advanced by vTaskDelayUntil; expected
        // wake = xLastWakeTime ticks -> us. (Tick = 1 ms on this build.)
        const uint32_t expected_wake_us = (uint32_t)xLastWakeTime * 1000u;
        // Unsigned subtraction handles micros() wraparound; a huge value
        // means we woke early (delayed tick update) — clamp to 0.
        uint32_t lateness_us = work_start_us - expected_wake_us;
        if (lateness_us < 1000000u && lateness_us > max_lateness_us)
        {
            max_lateness_us = lateness_us;
        }

        uint32_t now_ms = millis();

        // Receiver service start-to-start gap on the micros clock.
        if (last_service_start_us != 0)
        {
            const uint32_t gap_us = work_start_us - last_service_start_us;
            if (gap_us > max_service_gap_us) max_service_gap_us = gap_us;
        }
        last_service_start_us = work_start_us;

        // Pump i-BUS every 1 ms (required by IBUSBM_NOTIMER — upstream says
        // loop() must be called at least every 1 ms).
        rc_receiver.service(now_ms);

        // Full safety evaluation at 50 Hz (20 ms) — well under the 100 ms budget.
        // NOTE: "immediate" CH6 LOW stop means this evaluation, which runs
        // at most 20 ms after the receiver decodes the endpoint.
        if (now_ms - last_safety_ms >= 20)
        {
            const uint32_t feature_start_us = micros();
            last_safety_ms = now_ms;

            bool agent_conn = g_agent_connected.load(std::memory_order_acquire);
            relay_feature_controller.service(agent_conn);

            const uint32_t feature_eval_us = micros() - feature_start_us;
            if (feature_eval_us > max_feature_eval_us) max_feature_eval_us = feature_eval_us;

            // Honest I2C duration: the bus transaction itself, measured
            // inside writeI2cRegister — not the surrounding evaluation.
            const uint32_t i2c_write_us = relay_board.lastI2cWriteUs();
            if (i2c_write_us > max_i2c_write_us) max_i2c_write_us = i2c_write_us;

            // FRESH timestamp — captured AFTER feature evaluation so beep
            // start time (set inside requestStatusBeep()) is never in the
            // future relative to now_ms. This fixes the unsigned-underflow
            // bug that silently cancelled every new beep.
            relay_board.serviceStatusIndicators(millis());

            // Publish a coherent telemetry snapshot for the ROS path — one
            // consistent view of receiver + board state, no live reads.
            // Includes the authoritative receiver-local stop + reasons.
            ControlSnapshot snap{};
            snap.captured_ms = now_ms;
            snap.frame_sequence = rc_receiver.lastFrameCounter();
            snap.last_valid_frame_ms = rc_receiver.lastValidFrameMs();
            for (uint8_t index = 0; index < RC_CHANNEL_COUNT; ++index)
            {
                snap.channels[index] = rc_receiver.rawChannel(index);
            }
            snap.signature_mask = rc_receiver.signatureMask();
            snap.receiver_state = static_cast<uint8_t>(rc_receiver.state());
            snap.receiver_local_stop = rc_receiver.receiverLocalStop();
            snap.receiver_stop_reasons = rc_receiver.receiverStopReasons();
            snap.desired_mask = relay_feature_controller.lastDesiredMask();
            snap.last_written_mask = relay_board.lastWrittenRelayStates();
            snap.digital_inputs = relay_board.readDigitalInputs();
            snap.relay_state_known = relay_board.isRelayStateKnown();
            publishSnapshot(snap);

            ++service_count;
        }

        // Periodic timing log (drained by loop() on core 1). All values are
        // measured maxima over the window — reported as observed, never as
        // a certified bound. No arbitrary compliance threshold: a "missed
        // deadline" is not defined here; consumers judge the raw maxima.
        if (now_ms - last_timing_log_ms >= kTimingLogPeriodMs)
        {
            last_timing_log_ms = now_ms;
            enqueueLog(LogCategory::FeatureController, 5, max_service_gap_us);
            enqueueLog(LogCategory::FeatureController, 6, max_feature_eval_us);
            enqueueLog(LogCategory::FeatureController, 7, max_i2c_write_us);
            enqueueLog(LogCategory::FeatureController, 8, max_lateness_us);
            enqueueLog(LogCategory::FeatureController, 9, service_count);
            // Stack high-water mark (words remaining) — watch for overflow.
            enqueueLog(LogCategory::FeatureController, 10,
                       uxTaskGetStackHighWaterMark(NULL));
            max_service_gap_us = 0;
            max_feature_eval_us = 0;
            max_i2c_write_us = 0;
            max_lateness_us = 0;
            service_count = 0;
        }
    }
}

// One compact status line every 5 s (the old firmware printed per-loop —
// drowned real events and added serial overhead to the control loop).
constexpr uint32_t STATUS_PRINT_PERIOD_MS = 5000;

void printPeriodicStatus(const ControlSnapshot &snapshot, uint32_t now_ms)
{
    static uint32_t last_status_print_ms = 0;
    if (now_ms - last_status_print_ms < STATUS_PRINT_PERIOD_MS)
    {
        return;
    }
    last_status_print_ms = now_ms;

    const char *agent_text = micro_ros_bridge.agentStateText();
    // Receiver text from the AUTHORITATIVE stop + reasons (computed in
    // control), never rebuilt from raw fields here.
    const char *receiver_text =
        (snapshot.receiver_state !=
         static_cast<uint8_t>(ReceiverOperatingState::SignalActive))
            ? (snapshot.receiver_state ==
                       static_cast<uint8_t>(ReceiverOperatingState::FailsafeEngaged)
                   ? "ESTOP-FS"
                   : "LOST")
            : (snapshot.receiver_local_stop ? "STOP" : "ACTIVE");

    const char *config_suffix = switch_binding_configuration_valid ? "" : " CFG-INVALID";
    uint32_t snap_age_ms = (now_ms >= snapshot.captured_ms)
                               ? (now_ms - snapshot.captured_ms)
                               : 0;
    static uint32_t snap_age_max_ms = 0;
    if (snap_age_ms > snap_age_max_ms) snap_age_max_ms = snap_age_ms;
    // Reset the window maximum each print so it reports the worst age seen
    // in the last 5 s, not since boot.
    uint32_t report_age_max_ms = snap_age_max_ms;
    snap_age_max_ms = 0;
    uint32_t log_dropped = g_log_dropped_count.load(std::memory_order_relaxed);
    Serial.printf("[STATUS] up=%lus eth=%s ip=%s agent=%s rc=%s relay=0x%02X%s%s snap_age=%lums snap_age_max=%lums log_dropped=%lu\n",
                  (unsigned long)(now_ms / 1000),
                  ETH.linkUp() ? "up" : "down",
                  ETH.localIP().toString().c_str(),
                  agent_text,
                  receiver_text,
                  snapshot.last_written_mask,
                  snapshot.relay_state_known ? "" : " (last write UNKNOWN)",
                  config_suffix,
                  (unsigned long)snap_age_ms,
                  (unsigned long)report_age_max_ms,
                  (unsigned long)log_dropped);
}

void setup()
{
    Serial.begin(115200);

    // 0) Relays FIRST — before ANY debug delay: the safe output state must
    //    be established as early as possible (e.g., MCU reset while the
    //    expander is still powered with previously energized outputs).
    relay_board.begin();

    // Boot-only pause so the USB-CDC serial monitor shows the first lines.
    // This is the ONLY delay() outside RelayBoard::begin()'s bounded init
    // retries, and it runs before any subsystem that matters is live — the
    // runtime loop below uses only bounded blocking ops (see DESIGN RULES).
    delay(500);

    Serial.println();
    Serial.println("=================================================");
    Serial.println(" K.A.R.R. safety controller — e-stop / lighting (ESP32-RELAY-FW)");
    Serial.println("=================================================");

    // 1) Local inputs + receiver
    rc_receiver.begin(RC_RECEIVE_PIN);

    // 2) Configuration sanity check (hand-edited tables). The result is
    //    PERSISTED: while invalid, RC-driven feature operation stays
    //    inhibited — scoped to this table, not a board-wide fault.
    switch_binding_configuration_valid = validateSwitchToRelayBindings();
    if (switch_binding_configuration_valid)
    {
        Serial.println("[CFG] switch->relay bindings valid");
    }
    else
    {
        Serial.println("[CFG] WARNING: invalid bindings — RC-driven features INHIBITED until fixed");
    }

    // 3) Ethernet + micro-ROS transport (agent connection managed in loop).
    //    Bypassed entirely when BYPASS_MICROROS is defined.
#ifndef BYPASS_MICROROS
    micro_ros_bridge.begin();
#else
    Serial.println("[BOOT] micro-ROS bypassed — running control-only");
#endif

    Serial.println("[BOOT] initialization complete — entering dual-task loop");
    Serial.println("[BOOT] core 0: RC + safety (control task)");
    Serial.println("[BOOT] core 1: micro-ROS (this loop)");
    Serial.println("[BOOT] start the agent: micro_ros_agent udp4 --port 8888");

    // Create the log event queue before the control task starts so control-task
    // code can enqueue without blocking on Serial. The queue lives on core 1
    // (loop drains it, bounded per iteration); the control task only ever
    // enqueues with zero wait.
    g_log_queue = xQueueCreate(16, sizeof(LogEvent));

    // Coherent telemetry snapshot queue (length one, overwrite latest).
    // Control task publishes; loop() on core 1 consumes.
    g_snapshot_queue = xQueueCreate(1, sizeof(ControlSnapshot));

    if (!g_log_queue || !g_snapshot_queue)
    {
        // Init failure: safe boot output (0x00) was already commanded, but
        // normal operation must NOT start — that would run without logging
        // or telemetry. Report (contact state NOT verified) and halt here.
        Serial.println("[BOOT] QUEUE CREATION FAILED — halting (relays at boot safe state, contact NOT verified)");
        for (;;)
        {
            delay(1000);
        }
    }

    // Validate the 1 ms tick period BEFORE creating the task.
    if (pdMS_TO_TICKS(1) == 0)
    {
        Serial.println("[BOOT] TICK TOO COARSE (pdMS_TO_TICKS(1)==0) — halting");
        for (;;)
        {
            delay(1000);
        }
    }

    // 4) Start the control task on core 0 — RC + safety run here,
    //    independent of micro-ROS blocking on core 1.
    BaseType_t create_result = xTaskCreatePinnedToCore(
        controlTask, "ControlTask", 8192, NULL, configMAX_PRIORITIES - 2,
        &g_control_task_handle, 0);
    if (create_result != pdPASS || !g_control_task_handle)
    {
        // Creation failed: safe boot output (0x00) was already commanded,
        // but normal operation must NOT start without the control task.
        Serial.println("[BOOT] CONTROL TASK CREATION FAILED — halting (relays at boot safe state, contact NOT verified)");
        for (;;)
        {
            delay(1000);
        }
    }
    Serial.printf("[BOOT] control task pinned to core 0, priority %u/%u\n",
                  (unsigned)configMAX_PRIORITIES - 2,
                  (unsigned)configMAX_PRIORITIES - 1);
}

void loop()
{
    // -----------------------------------------------------------------------
    // ROS PATH ONLY — runs on core 1. May block (ping, entity lifecycle,
    // reliable publish) but CANNOT delay the control task on core 0, which
    // samples RC, evaluates features, writes relays, and updates indicators
    // at a fixed 1 ms / 20 ms cadence regardless of what happens here.
    // -----------------------------------------------------------------------
    // Drain the control-task log event queue first: all Serial writes from
    // the control path happen here, never inside the control task.
    drainLogQueue();

#ifndef BYPASS_MICROROS
    micro_ros_bridge.service(millis());

    // Agent availability published IMMEDIATELY after bridge servicing —
    // before any potentially blocking telemetry — so control sees the
    // latest agent state with minimum delay.
    g_agent_connected.store(micro_ros_bridge.isAgentConnected(),
                            std::memory_order_release);
#endif

    // Consume the coherent snapshot for telemetry + status. No live
    // cross-core reads of RcReceiver / RelayBoard. When micro-ROS is
    // bypassed, g_agent_connected stays false and RC-driven features are
    // inhibited (FORCE_SAFE_ON_AGENT_LOSS).
    ControlSnapshot snap{};
    if (receiveSnapshot(snap))
    {
#ifndef BYPASS_MICROROS
        micro_ros_bridge.publishReceiverTelemetry(snap, millis());
        micro_ros_bridge.publishBoardTelemetry(snap, millis());
#endif
        printPeriodicStatus(snap, millis());
    }
}
