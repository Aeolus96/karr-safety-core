# K.A.R.R. Safety Core (ESP32-RELAY-FW) — Notes

Baseline firmware for the **K.A.R.R.** (Keystone Autonomous Resource Rover)
safety & relay controller — the Waveshare **ESP32-S3-POE-ETH-8DI-8RO-C**
board: micro-ROS over **Ethernet only**, i-BUS RC input (Turnigy TGY-i6S
transmitter + iA6B receiver), 8 relay outputs, 1 Hz heartbeat/diagnostics. Cooperative application loop with
BOUNDED blocking ops (see Timing below); no additional application tasks.
All code in `src/main.cpp`, organized as **5 feature
blocks** (see file header) ready to split into `.h/.cpp` pairs:
`RelayBoard` <- `RcReceiver` + `MicroRosBridge` <- `RelayFeatureController` <- main.

**Naming**: `UPPER_SNAKE_CASE` = compile-time constants (obvious by shape —
the old `kConstantName` prefix was dropped), `camelCase` = methods/vars,
`snake_case_` = class members, `CamelCase` = classes/enumerators. Constants
live with their feature block, not at the top of the file.

## Bring-up

1. Flash: `pio run -t upload` (USB-CDC, 115200 monitor).
2. On the ROS PC (Jazzy): `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
3. Check: `ros2 topic echo /rc/joy`, `ros2 topic hz /rc/joy`,
   `ros2 topic echo /relay_board/heartbeat`
4. Permission model (relays commanded to 0x00 = all-off WRITE request,
   not confirmed contacts — see "Dependency model" below):
   - BOARD-level: relay hardware initialized.
   - RC-driven features (today's only group): RC frames fresh
     (<=200 ms — the code accepts exactly 200 ms), NO confirmed
     receiver-local stop (signal loss, confirmed failsafe signature,
     decoded ch6 stop, persistent invalid input), agent connected
     (`FORCE_SAFE_ON_AGENT_LOSS=true`), binding table valid.
   LED: red SOLID = relay bus fault · red blink = agent unavailable (ROS
   connectivity) · yellow blink = receiver-local inhibition (RC
   features) · purple blink = invalid RC binding config · green = no
   policy condition detected (not contact verification).
5. Config validity: confirm NO purple blink and no `CFG-INVALID` in the
   serial status line — invalid switch->relay bindings inhibit the
   RC-driven feature group (today: all features) even while the board
   constraint itself is satisfied.

## Config knobs (each inside its block)

- `RC_CHANNEL_CONFIGURATION[]` (BLOCK 2) — channel type table, currently
  **Mode 1**: ch1-ch4 joysticks, ch5/ch6 2-pos, ch7/ch8 3-pos, ch9/ch10 2-pos
- `RC_RECEIVE_PIN` — **GPIO1** is the *effective* pin of the tested
  prototype (`IBus.begin` re-begins UART1 with rxPin=1, overriding the
  prototype's own `Serial1.begin(21)`)
- `SWITCH_TO_RELAY_BINDINGS[]` (BLOCK 4) — placeholder switch→relay map;
  **the feature names are TYPE PLACEHOLDERS** (front_lighting, beacon,
  work_light, strobe, auxiliary_1) — they are NOT the exact features we
  will build later; only the ch6 wireless e-stop is a defined safety
  function today. All relay control logic goes here. **ROS can never
  command relays** (no subscription exists by design)
- `FORCE_SAFE_ON_AGENT_LOSS` — default true (fail de-energized). SCOPE:
  it gates ROS-dependent features — today that is every feature; it must
  NOT gate future independent features (each group declares its own ROS
  dependency instead)
- Network (BLOCK 3): board `192.168.0.24`, agent `192.168.0.4:8888`, domain 0

## Topic contract (publish-only)

| Topic | Type | Rate |
|---|---|---|
| `/rc/joy` | sensor_msgs/Joy | 50 Hz (zeroed on frame loss/failsafe — `isSignalActive()`, not full inhibition) |
| `/rc/link` | std_msgs/Bool | 1 Hz (state-machine active = liveness, not permission) |
| `/relay_board/heartbeat` | std_msgs/UInt32 (uptime s) | 1 Hz |
| `/relay_board/relay_state` | std_msgs/UInt8MultiArray (8) | on change + 1 Hz |
| `/relay_board/inputs` | std_msgs/UInt8 | 1 Hz |
| `/relay_board/estop` | std_msgs/Bool | 1 Hz |

**Topic scopes**: `/rc/link` = receiver state-machine liveness, not
permission to operate. `/relay_board/estop` = **receiver-local
inhibition** — NOT a board-wide relay-off indication and NOT a
permission-to-operate signal; with independent features it may be TRUE
while an independent safety indication stays active (scope, not
contradiction). `/relay_board/relay_state` = last mask SUCCESSFULLY
written — latest-state snapshots, never a guaranteed transition history.

## Safety semantics (read this before interpreting telemetry)

**Dependency model — board constraints vs feature dependencies:**

Three distinct concepts (matches `RelayFeatureController`, diagram in
BLOCK 4 of the code):

1. **Board-level actuation constraint** — relay hardware completed its
   safe initialization. Applies to EVERY feature; `applyRelayStates()`
   independently refuses nonzero writes while uninitialized.
2. **Feature-group dependencies** — each group declares its own (RC
   health, digital-input validity, ROS availability, a defined stop
   condition) and its own behavior when a dependency is unavailable.
   Today there is exactly ONE group: the RC-driven bindings (type
   placeholder names).
3. **Combination** — each permitted group ORs its requested bits into one
   mask; board constraints are applied centrally AFTER combination; the
   controller remains the single relay-output writer.

Feature CATEGORIES (categories with defined roles — not implemented
behaviors):

| Feature category | RC required? | ROS required? | Unavailable-input behavior |
|---|---|---|---|
| RC-operated lighting (current bindings) | Yes | Yes (`FORCE_SAFE_ON_AGENT_LOSS`) | inactive — bits to 0x00 |
| Digital-input-operated lighting | No, unless specified | No | its own digital-input policy |
| Independent safety indication | Not inherently | No | continues its defined indication |
| Future ROS-reliant functionality | Only if used | Yes | stops / enters its defined fallback |

Composition flow (same diagram in the BLOCK 4 header):

    RC state ──────────┐
    Digital inputs ────┼──> per-group dependency evaluation
    ROS availability ──┘            │
                                    v
                      combine feature output requests (OR)
                                    │
                                    v
                      apply board-level constraints (relay init)
                                    │
                                    v
                      single relay-output writer

- **Relay ownership**: one owner per relay unless sharing is intentional.
  Requests combine by bitwise OR — a SHARING rule (any owning feature may
  request ON), NOT an arbitration policy for conflicting priorities; the
  startup validator warns (never fails) on duplicate assignments so
  sharing stays visible.
- **Config-fault scope**: an invalid RC binding table inhibits the RC
  group ONLY — a broken feature configuration, not a board-wide
  configuration problem. Future groups validate their own configuration.
- **Stop-condition boundary**: RC loss, wireless e-stop engagement and a
  future physical stop input need an EXPLICIT output scope per feature
  when those features are designed. An independent warning light may
  remain active during a stop while an actuator must not — neither "stop
  everything" nor "ignore stops for independent features" may be applied
  silently. Today ALL outputs are RC-driven and inherit full
  receiver-local inhibition; that policy stays until each output's policy
  is deliberately changed.

- **RC-group permission** (today's only feature group) requires ALL of:
  RC frames fresh (<=200 ms — the code accepts exactly 200 ms), NO
  confirmed receiver stop (see e-stop layers below), the agent connected
  (`FORCE_SAFE_ON_AGENT_LOSS=true`), and a valid RC binding table. Board
  level: relay hardware initialized. One predicate,
  `RcReceiver::isLocallyInhibited()`, covers signal loss + confirmed
  failsafe + decoded CH6 stop + persistent invalid input — the permission
  check, `/relay_board/estop` and the LED policy all consult it so they can
  never disagree about receiver-local state. Brief intermediate input or an
  unconfirmed signature NEVER inhibits by itself.
- **Relay initialization invariant** (separate from cached state):
  `relay_initialized_` = the full safe sequence (output latch write, THEN
  direction write — in that order, aborting if the latch fails) succeeded
  at least once. While uninitialized, `applyRelayStates()` REFUSES nonzero
  requests and retries the full sequence every 200 ms. A successful output
  write alone never establishes initialization.

  Decision map (matches `applyRelayStates()`):

      uninitialized -- request != 0x00 -------------> REFUSE
                     \- request == 0x00 -> full init sequence (latch THEN
                            direction), throttled >= 200 ms after failure
      initialized ---- target == last written --------> no-op
                     \- new target -> I2C write (a NEW 0x00 transition
                            gets one immediate attempt; other targets are
                            throttled while failing) -> success = last
                            write status known; failure = last write
                            status UNKNOWN, retry throttled
- **Write retry policy**: ordinary target changes do NOT bypass fault retry
  limits; only a NEW transition to SAFE gets one immediate attempt;
  subsequent failed attempts of the same target stay rate-limited (200 ms).
- **E-stop layers** (any one inhibits RC-driven features). "Immediate" CH6
  LOW means the next nominal 20 ms feature evaluation, not necessarily the
  same 1 ms iteration that decoded it:
  1. Wireless e-stop switch ch6, DECODED with intermediate grace: valid LOW
     engages at once; valid HIGH releases at once; brief intermediate input
     preserves the previous decoded stop state for at most
     INTERMEDIATE_GRACE_MS (200 ms provisional, separately named policy);
     implausible values and "no data yet" (0) read ENGAGED without grace;
     no prior valid state reads ENGAGED. Only a deliberate valid HIGH
     releases. The board ALWAYS starts inhibited (boot is never
     an "e-stopped" event — no engagement is required): a valid HIGH is
     accepted immediately, with no rearm transition required. Persistently
     intermediate CH6 (grace expired) engages receiver-local stop.
  2. Receiver failsafe signature: a 2-position channel (ch5/ch6/ch9/ch10)
     inside the NARROW 1490-1510 window in fresh frames starts per-channel
     confirmation (timer starts on ENTRY, never restarts on wobble); 200 ms
     of continuous fresh-frame window evidence confirms that channel
     (FAILSAFE_CONFIRM_MS, provisional). The all-channel-1500 failsafe is
     the TESTED transmitter configuration's observed behavior — it parks a
     2-pos switch inside the window, where a healthy switch never rests
     (1000/2000 only); this is not a universal i-BUS failsafe property and
     the tolerance is NOT protocol-defined — verify transmitter-off,
     startup, and reconnection on the bench. Confirmation is per-channel
     (no shared timer); leaving the window resets that channel's
     confirmation but does NOT grant permission. Clearing one channel never
     clears another channel, CH6 stop, invalid input, or frame loss.
     Recovery requires fresh VALID replacement evidence (an endpoint) for
     the affected channel. Ordinary brief transitions never cut outputs;
     each feature request retains its channel's last valid endpoint while
     intermediate (bounded by INTERMEDIATE_GRACE_MS), then latches a
     distinct InvalidSwitchInput receiver-local stop.
  3. Frame loss: no i-BUS frames decoded for >200 ms (RF loss, receiver
     power, or a UART wiring fault — indistinguishable at this layer), or
     ch6 below 980 (receiver unbound).
- `/relay_board/estop` = `isLocallyInhibited()` — TRUE whenever the
  receiver-local chain is not fully healthy. Receiver-local inhibition
  only: not a board-wide relay-off indication, never a
  permission-to-operate claim.
- **LED priority** (first match wins — a single selected LED cannot show
  every simultaneous condition; the serial status line and topics carry
  the detail):

      1. relay bus fault?          --yes--> RED solid
         |
         no
         v
      2. agent unavailable?        --yes--> RED blink
         |
         no
         v
      3. receiver-local            --yes--> YELLOW blink
         inhibition?
         |
         no
         v
      4. invalid RC binding        --yes--> PURPLE blink
         config?
         |
         no
         v
      5. GREEN solid

  1 = uninitialized / last write status unknown / consecutive I2C
  failures — writes, not contact feedback. 2 = ROS CONNECTIVITY only: it
  does NOT claim local features stopped. 3 = RC-dependent features
  inhibited — not every output (with independent features,
  `/relay_board/estop` TRUE may coexist with an active safety indication
  — scope, not conflict). 4 = the RC group's config scope. 5 = no
  condition covered by this policy detected — NOT physical verification
  of relay operation, and NOT merely "frames are available".
- `/relay_board/relay_state` snapshot semantics: the last mask
  SUCCESSFULLY written to the expander is published (no readback — a
  register write is not contact confirmation). Reliable QoS, 100 ms ack
  budget, latest-state SNAPSHOTS only (a failed transition can be
  superseded by newer state) — see "Relay-state publication" below for
  the eligibility rules.
- **Switch polarity convention**: switches are TRUE/active when LOW (1000):
  2-pos LOW=1.0/HIGH=0.0; 3-pos LOW=1.0/MID=0.0/HIGH=-1.0. The failsafe
  value (1500) reads inactive for every feature. Joy buttons = 1 only in
  the LOW (True) position.
- **Agent loss ordering**: loss is DETECTED and the state marked
  CleanupPending in one service call (entities stay alive); the loop's
  post-network pass then re-evaluates feature permissions — with
  `FORCE_SAFE_ON_AGENT_LOSS=true` every current RC-driven feature is
  commanded to 0x00 (a write request) — BEFORE the next service call
  performs teardown (publishers -> node -> support) on its own 500 ms
  retry schedule with NO discovery ping. Local RC sampling and feature
  evaluation continue in EVERY connection state; there is no busy-wait
  for the agent and no endless retry loop inside a single service call.
  Individual timeout budgets bound each step, but init/teardown stack
  several steps — whole-call duration needs on-hardware measurement
  (incremental lifecycle work across loop iterations is the fallback if
  measured delays are excessive).

  Lifecycle map (matches `MicroRosBridge::service()`; loop() samples RC
  and evaluates features in EVERY state):

      WaitingForAgent --ping ok--> AgentAvailable --created--> AgentConnected
            ^                          |                          |
            |                          | creation failed          | ping fails:
            |                          v                          v  LOSS DETECTED
            +---- destroyEntities ---+-> CleanupPending <---------+ ONLY
                 (500 ms cadence,        (teardown deferred one call:
                  no discovery ping)      features re-evaluated first)

  **Teardown ordering is strict**: a failed publisher fini keeps its flag
  AND blocks node finalization; a failed node fini blocks support
  finalization — parents are preserved until every child is released.
  Failure-stage policies and their revision-specific reasoning live in
  the **Maintenance appendix** — re-verify after any micro_ros_platformio
  update. Repeated cleanup abandonment is NOT indefinitely leak-free
  recovery: abandonments are counted (`support_abandonment_count_`,
  monotonic) and logged; if testing ever shows the count climbing, the
  planned escalation is SUSPENDING ROS reconnection in a visible degraded
  state while local features continue — an automatic whole-board reboot
  is NOT the default answer to a ROS-only failure.
- **Relay-state publication** (reliable QoS, 100 ms ack budget): ONE
  eligibility decision — fresh-connection snapshot or a new target vs the
  last ATTEMPTED mask publishes immediately; the same failed target
  enforces a 500 ms retry gap (the 1 Hz health tick cannot bypass it);
  periodic refresh only when nothing is pending. Latest-state snapshots,
  not a guaranteed event history. Heartbeat/link/inputs/estop keep their
  independent 1 Hz cadence.
- **Required timing config**: entity-creation and relay-publisher session
  timeouts are part of successful initialization — if either setter fails,
  init fails and is retried (the bridge never runs without its requested
  budgets). The destroy timeout is best-effort (logged) because teardown
  must always proceed.
- **Time sync**: runs only on a LATER service call than the one that
  created the entities (`connected_at_entry && isAgentConnected()`), never
  on a destroyed context; cleared on teardown. Joy stamps are populated
  only while `time_synced_`.
- **Configuration health (RC-group scope)**: `validateSwitchToRelayBindings()`
  runs at boot; while invalid, `switch_binding_configuration_valid=false`
  inhibits the RC-driven feature group (today: all features) — not just
  logged — and it is surfaced three ways: purple LED blink, `CFG-INVALID`
  in the serial status line, and an explicit
  `[SAFETY] RC features INHIBITED ... rc_config=INVALID` log. A broken RC
  map is NOT a board-wide configuration fault: future independently
  configured groups are unaffected. `/relay_board/estop` intentionally
  stays receiver-local only.
- **Peripheral-reset assumption** (documented): `relay_initialized_` is
  not cleared by ordinary write failures. An INDEPENDENT expander reset is
  outside the recovery model; the follow-up if needed is a defined
  communication-fault trigger for full re-initialization.
- **Rearm**: the baseline has NO global rearm latch and no required
  switch-transition sequence — when a feature's dependencies recover, it
  follows its current valid inputs according to that feature's recovery
  policy (a switch left TRUE re-energizes as soon as permission returns).
  Future hazardous outputs may require a per-feature rearm rule in
  BLOCK 4 after a separate review; that possibility must not introduce a
  rearm subsystem into the baseline.
- **Timing**: cooperative loop with BOUNDED blocking: agent ping <=100 ms,
  one-shot time sync <=100 ms (later cycle), entity create/destroy sessions
  100/200 ms, reliable-publish budget 100 ms, boot-time relay init retries.
  I2C / best-effort publishes / logging are individually bounded but not
  measured — worst-case loop gaps need on-hardware measurement.
  Timing is reliable SERVICE, not exact cadence: joy nominally 50 Hz
  (≈40–45 Hz with variation is acceptable for the described downstream
  use), health ≈1 Hz, and the RC-loss timeout must trigger reliably when
  expired — modest response overshoot (~210–250 ms) is acceptable; missing
  or repeatedly postponing the timeout is not. Prioritize avoiding long
  stacked operations over exact publication rates; validate RC loss
  during network work (reconnect/teardown), not just on an idle
  connection — buffered UART bytes after a long service gap can refresh
  frame freshness (documented limitation in `RcReceiver::service`).
- micro_ros.meta headroom: 6 publishers (limit 8), no subscription/executor.
- **Later feature (noted, not implemented)**: the physical big-red-button
  e-stop chain wired to a GPIO input — a third stop layer, designed with
  the DIN inputs, whose OUTPUT SCOPE must be defined explicitly per
  feature (see the stop-condition boundary above), not assumed to be
  either "all off" or "ignored by independent features".


## Failsafe detection (confirmation-gated, per-channel)

    Decoded frames fresh AND ch6 not below unbound threshold?
        |
        +-- No --> SignalLost ---> receiver-local stop (signal loss)
        |
        +-- Yes --> Any 2-pos channel with CONFIRMED signature?
                        |
                        +-- Yes --> FailsafeEngaged -> stop (failsafe)
                        |
                        +-- No --> SignalActive (liveness); stop still
                                   applies for decoded CH6 stop or
                                   persistent invalid input.

    Brief intermediate / unconfirmed signature NEVER stops by itself.

Behavior table (bench patch — provisional timings, not a validated safety
configuration):

| Input condition | Behavior |
|---|---|
| Ordinary 2-pos switch reaches a valid endpoint | Its own relay request updates at the next 20 ms evaluation |
| 3-pos switch (ch7/ch8) reaches a valid endpoint | Its own relay request updates at the next 20 ms evaluation |
| 3-pos switch rests in the MID zone (or reads implausible) past 200 ms grace | Effective value resolves to canonical MID (inactive) — never failsafe evidence, never an invalid-input stop |
| Ordinary switch briefly enters the narrow signature window | Start per-channel confirmation; retain last valid endpoint; no global cutoff |
| Signature present in valid new frames for 200 ms | Confirmed failsafe for that channel; safe mask (0x00) |
| CH6 reaches valid LOW | Engage stop at the next control evaluation (no 200 ms wait) |
| CH6 reaches valid HIGH | Release switch-stop, subject to other permissions |
| CH6 briefly intermediate | Preserve previous decoded stop state, at most 200 ms grace |
| No valid frames for the receiver timeout | Stop independently of signature confirmation |
| Implausible or persistently undecodable CH6 | Stop without grace; never retain "released" |
| Ordinary 2-pos channel persistently intermediate past grace | Distinct InvalidSwitchInput receiver-local stop |
| CH6 at 1500 with no previous endpoint | Remains stopped (boots stopped) |
| CH6 alternating 1500/1400 | Grace starts on first entry, never restarts — expires once |
| Signature moves to non-signature intermediate | Signature confirmation resets; intermediate episode continues; NOT permission |
| One channel clears | Other channels' flags, CH6 stop, invalid input, frame loss all unaffected |

The tested transmitter configuration outputs 1500 on all channels during
failsafe (observed behavior of THIS i6S/iA6B setup — not a universal i-BUS
failsafe property). Only the NARROW 1490-1510 window is signature evidence;
ordinary mid-zone samples are intermediate input with retained endpoints.
Confirmation is per-channel across ch5/ch6/ch9/ch10 (ch7/ch8 excluded — a
legitimate middle position is not failsafe evidence there). ch7/ch8 feed
only their effective feature value: a valid endpoint updates the bound
relay at the next 20 ms evaluation, a brief sweep retains the previous
value, and a sustained MID-zone rest resolves to inactive after the same
200 ms grace. Confirm on the bench that the transmitter-off signature
actually appears on all four; do
NOT require all four simultaneously to declare failsafe. IBusBM keeps
delivering receiver frames after RF loss, so the frame counter alone cannot
replace failsafe-value detection. A frame arriving after a long service gap
resets all per-channel evidence before its own values are processed — old
evidence is never treated as uninterrupted confirmation. Frame-counter
stall >200 ms = loss; the buffered-UART freshness limitation is documented
in `RcReceiver::service` — validate RC loss during network work, not only
on an idle connection. Provisional 200 ms timings are a policy choice, not
a demonstrated safe stopping-time requirement: actual RF-loss response is
receiver detection/output delay + firmware confirmation + control
scheduling — measure transmitter-loss to safe-mask write on the bench.

## IBusBM: registry vs local — DIFFERENCE FOUND

Diffed the local copy (from the test project) against upstream:

- **vs registry v1.1.4** (`bmellink/IBusBM@^1.1.4`): the ONLY difference is
  the ESP32 timer init inside `begin()` (lines 114-121):
  - local (kept): `timerBegin(1000000)` + `timerAttachInterrupt(timer, &onTimer)`
    + `timerAlarm(timer, 1000, true, 0)` → **arduino-esp32 3.x API**
  - v1.1.4: `timerBegin(timerid, F_CPU/1000000L, true)` + `timerAlarmWrite`
    + `timerAlarmEnable` → **pre-3.0 API, FAILS TO COMPILE on core 3.x**
    (verified: `platformio run` with the registry version errors out on
    `timerBegin(uint32_t)` / `timerAlarmWrite` not declared)
- vs master: master is further ahead (same 2.x-style timer code + STM32
  HardwareTimer support); the i-BUS protocol parsing code itself is
  byte-identical everywhere.

**Conclusion**: the "small fix" someone applied to the local copy is the
core-3.x timer API update. We keep the **local copy** in `lib/IBusBM`
(deterministic + compiles). The timer path is never executed anyway —
we use `IBUSBM_NOTIMER` and poll `loop()` ourselves — but it must compile.
If upstream ever publishes a 3.x-compatible release, we can switch to the
registry and delete `lib/IBusBM`.

## diagnostics_msgs evaluation (planned — NOT implemented)

Baseline decision: keep the simple scalar topics; translate them into
standard diagnostics on the ROS PC if needed. Before any firmware change,
run this small isolated proof of concept against the INSTALLED
micro_ros_platformio revision and the Jazzy agent:

1. Confirm generated `diagnostic_msgs` type support LINKS (the headers
   are present in the installed libmicroros; the build must also link the
   microxrce-dds typesupport for `DiagnosticArray`).
2. Publish ONE `DiagnosticStatus` with a small FIXED number of key/value
   entries from a statically allocated message (fixed-capacity strings
   and sequences — no recurring allocation, no unbounded string growth).
3. Confirm correct reception and deserialization on the ROS PC.
4. Exercise disconnect/reconnect cycles.
5. Check message storage, serialized size, transport limits and loop
   impact. If all pass, a fixed-capacity nominal 1 Hz diagnostics
   publisher is a reasonable option.

**Compatibility caveat**: the micro-ROS diagnostics framework README
explicitly states a limitation with publishing standard diagnostic
messages (offering simplified messages + a host-side bridge instead). Do
not assume that describes the current firmware/agent combination — and do
not ignore it either. Fallbacks: host-side translation of the existing
scalar topics, or the dedicated micro-ROS diagnostics stack (extra
packages + bridge).

**Rule**: diagnostics OBSERVE control behavior, never determine it — a
diagnostics failure must not disable relay features. Candidate status
groups: relay initialization/write failures · RC freshness, confirmed
failsafe, wireless stop, invalid input · ROS connection, publication
failures, cleanup
abandonments · configuration validity · feature-specific
inhibition/fallback.

## Validation scenarios (demonstrate the dependency model)

| Scenario | Expected result |
|---|---|
| Boot without ROS agent | local features run once their own prerequisites are satisfied |
| Agent loss / restart | local outputs remain governed by local inputs; ROS reconnects separately |
| RC loss | RC-dependent features enter their defined inhibited state (bits to 0x00) |
| RC loss + independent safety indication (future) | indication continues per its own policy |
| RC recovery with active switch | current baseline feature resumes — no global rearm gesture |
| Brief 2-position intermediate sample | endpoint retained — NO cutoff; grace expiry (>200 ms) → InvalidSwitchInput stop; no premature FailsafeEngaged announcement |
| Signature confirmed on one channel | that channel stops; clearing it never clears other channels, CH6 stop, invalid input, or frame loss |
| RC loss during reconnect/teardown | timeout still triggers without indefinite delay |
| Relay write failure | truthful fault reporting + existing rate-limited recovery |
| Invalid RC binding | no invalid actuation; unrelated feature groups unaffected (config scope) |
| Diagnostics failure (future) | control behavior remains independent |

## Vendor file verdict (for later cleanup of the test projects — NOT done yet)

| File | Verdict | Reason |
|---|---|---|
| I2C_Driver / WS_TCA9554PWR | **USED** (inlined into RelayBoard) | proven register sequences; bool error returns fixed |
| WS_Relay (core) | **USED** (as RelayBoard) | dropped BT/WIFI/MQTT/RS485 command parser, bug at line 239 (Flag[0] vs [CHx-1]) |
| WS_GPIO (RGB/Buzzer) | **USED** (rewritten, no tasks) | vendor had 3 tasks + shared flags for simple patterns |
| WS_DIN | **READ-ONLY now** | DIN→relay auto-copy every 20 ms, no debounce — unsafe here |
| WS_ETH | **pattern used, file dropped** | only ETH.begin via micro-ROS transport; NTP/TCP/UDP test code removed (blocking, ran in event callback) |
| lib/IBusBM | **USED (local copy)** | 3.x timer API fix; see diff section above |
| WS_CAN | **LATER** (flagged) | header/impl signature mismatch (won't link), blocking 1 s alert read; needs rework when CAN comes |
| WS_RTC / WS_PCF85063 | **LATER / likely drop** | micro-ROS time-syncs on connect (`rmw_uros_sync_session`, one-shot, already in code) |
| WS_WIFI / WS_Bluetooth / WS_MQTT / WS_RS485 / WS_Serial / WS_SD / WS_Information.h | **DROP** | not needed; drags WiFi/BLE/MQTT creds, SD, Modbus, NTP libs |
| ArduinoJson / PubSubClient / NTPClient deps | **DROP** | only used by dropped files |
| bandwidth-test code in MICROROS main.cpp | **DROP** | 10× /rc/ch* pubs, ECHO sub, `while(1)` error halt, per-loop prints |

## Roadmap

1. Phase 1 (done): baseline — eth + agent SM + RC state + Joy + dependency model (scoped permissions) + heartbeat + one-shot time sync
2. Phase 2: validate Joy content on hardware; fix channel table + RX pin if needed; run the validation-scenario table above (incl. RC loss during reconnect/teardown, not just idle)
3. Phase 3: feature mapping (real features replacing the placeholder `SWITCH_TO_RELAY_BINDINGS` names, DIN usage); VALIDATE the existing RC-loss watchdog (`RECEIVER_FRAME_TIMEOUT_MS`) under load — no new watchdog needed
4. Phase 4: diagnostics depth (I2C failures, agent RTT, reset cause); run the `diagnostic_msgs` PoC above before any firmware change
5. Phase 5: CAN (rewrite WS_CAN non-blocking on GPIO17/18), RTC decision
6. Late cleanup: remove flagged vendor files from the two test projects, split main.cpp blocks into headers

## Maintenance appendix — revision-specific cleanup reasoning

**Finalizer semantics** (verified against the installed rcl/rclc source;
re-verify in `build/mcu/src/{rclc/rclc,rcl/rcl}` after ANY
micro_ros_platformio update — this policy assumes the current finalizer
behavior):

- `rcl_publisher_fini` / `rcl_node_fini` free and NULL their impl EVEN ON
  FAILURE; a repeat call returns OK: failed-child retries self-heal in
  one extra cycle, handles are re-init-safe, and NO attempt counter is
  needed for those stages.
- `rclc_support_fini` is the opposite — as shipped in this
  micro_ros_platformio revision it runs its three sub-finalizers
  unconditionally in ONE call, and a repeat after failure returns an error
  for the already-shut-down context (`rcl_shutdown` re-entry returns
  ALREADY_SHUTDOWN / INVALID), so retrying cannot clear it. Support fini
  therefore gets a SINGLE attempt, then the handles are abandoned
  immediately with a loud log that this is NOT clean teardown (agent-side
  entities may linger until the agent restarts; any unfreed local
  allocation stays until reboot). Abandonments increment the persistent,
  monotonic `support_abandonment_count_` (logged) so reconnect-and-leak
  cycles stay visible.

