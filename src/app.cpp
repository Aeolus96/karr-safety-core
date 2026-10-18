#include "karr/app.h"

#include <Arduino.h>
#include <ETH.h> // link/IP status queries only; ETH.begin() itself is
                 // performed by the micro-ROS ethernet transport

// --- FreeRTOS (ESP32 Arduino core) — separates RC/safety from blocking ROS ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "karr/diagnostics.h"
#include "karr/micro_ros_bridge.h"
#include "karr/rc_receiver.h"
#include "karr/relay_board.h"
#include "karr/relay_feature_controller.h"
#include "karr/state_exchange.h"

namespace karr
{
namespace
{

// System objects (construction order is harmless: only references are held).
RelayBoard relay_board;
RcReceiver rc_receiver;
MicroRosBridge micro_ros_bridge;
RelayFeatureController relay_feature_controller(relay_board, rc_receiver);

static TaskHandle_t g_control_task_handle = NULL; // set by setup(), read for diagnostics

// Control task: owns RC + safety + relays. Runs independently of micro-ROS.
// Pinned to core 0 so RC sampling and relay writes are never delayed by
// blocking ROS operations (ping, entity lifecycle, reliable publish).
void controlTask(void *parameter)
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

            bool agent_conn = loadAgentConnected();
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

    const char *config_suffix =
        relay_feature_controller.isSwitchBindingConfigurationValid() ? "" : " CFG-INVALID";
    uint32_t snap_age_ms = (now_ms >= snapshot.captured_ms)
                               ? (now_ms - snapshot.captured_ms)
                               : 0;
    static uint32_t snap_age_max_ms = 0;
    if (snap_age_ms > snap_age_max_ms) snap_age_max_ms = snap_age_ms;
    // Reset the window maximum each print so it reports the worst age seen
    // in the last 5 s, not since boot.
    uint32_t report_age_max_ms = snap_age_max_ms;
    snap_age_max_ms = 0;
    uint32_t log_dropped = logDroppedCount();
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

} // namespace
void app::setup()
{
    Serial.begin(115200);

    // 0) Relays FIRST — before ANY debug delay: the safe output state must
    //    be established as early as possible (e.g., MCU reset while the
    //    expander is still powered with previously energized outputs).
    relay_board.begin();

    // Boot-only pause so the USB-CDC serial monitor shows the first lines.
    // This is the ONLY delay() outside RelayBoard::begin()'s bounded init
    // retries, and it runs before any subsystem that matters is live — the
    // runtime loop below uses only bounded blocking ops (see design rules).
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
    const bool bindings_valid = relay_feature_controller.validateStartupConfiguration();
    if (bindings_valid)
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
    const bool log_queue_ready = initializeLogQueue();

    // Coherent telemetry snapshot queue (length one, overwrite latest).
    // Control task publishes; loop() on core 1 consumes.
    const bool snapshot_queue_ready = initializeSnapshotQueue();

    if (!log_queue_ready || !snapshot_queue_ready)
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

void app::loop()
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
    storeAgentConnected(micro_ros_bridge.isAgentConnected());
#endif

    // Consume the coherent snapshot for telemetry + status. No live
    // cross-core reads of RcReceiver / RelayBoard. When micro-ROS is
    // bypassed, agent availability stays false and RC-driven features are
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

} // namespace karr
