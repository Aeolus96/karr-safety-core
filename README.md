# K.A.R.R. Safety Core

Firmware for the **safety & relay controller** of **K.A.R.R.** (*Keystone
Autonomous Resource Rover*).

The board reads the RC receiver locally, runs e-stop and lighting relay
features **on-device**, and publishes telemetry to ROS 2 over Ethernet via
micro-ROS. Relays are **never commanded from ROS** — the link is
publish-only by design, so no external topic can actuate hardware.

> Design decisions, safety semantics, and the full rationale live in
> **[NOTES.md](NOTES.md)** — this README is the at-a-glance view.

## Hardware

| Part | Role |
| --- | --- |
| Waveshare **ESP32-S3-POE-ETH-8DI-8RO-C** | Target board — ESP32-S3, 8 relay outputs (TCA9554 I2C expander), 8 digital inputs, W5500 Ethernet, status RGB LED + buzzer |
| Turnigy **TGY-i6S** | RC transmitter (MODE 1) |
| Turnigy **iA6B** | RC receiver — i-BUS serial link into the ESP32; **ch6 = wireless e-stop** (safety-reserved) |
| ROS 2 PC (**Jazzy**) | Runs the micro-ROS agent; static LAN (board `192.168.0.24`, agent `192.168.0.4:8888`) |
| GL.iNet **Opal** travel router | Test-bench LAN (gateway `192.168.0.1`) — board + laptop both wired |

## What it does

1. Decodes the 10-channel i-BUS stream and publishes it as
   [`sensor_msgs/Joy`](https://github.com/ros2/common_interfaces) on
   `/rc/joy` (~50 Hz).
2. Maps RC switches to relay features **locally** (ch6 wireless e-stop is
   the defined safety function; lighting bindings are type placeholders
   for now).
3. Publishes 1 Hz health telemetry: `/rc/link`, `/relay_board/heartbeat`,
   `/relay_board/relay_state`, `/relay_board/inputs`,
   `/relay_board/estop`.
4. Enforces local safety: RC loss, the tested failsafe signature, the
   wireless e-stop, and (by default) agent loss all drive RC-dependent
   outputs to the all-off state. Recovery is automatic — no rearm gesture.

## Network layout (test bench)

The firmware uses **static IPs** in
[`src/micro_ros_bridge.cpp`](src/micro_ros_bridge.cpp) — no DHCP for the
board, so the router's "randomized MAC" note does not matter here. Just
reserve/remember the addresses below; the one thing that must match is the
agent IP the board dials.

| Device | IP | Notes |
| --- | --- | --- |
| Router (gateway) | `192.168.0.1` | GL.iNet Opal; laptop + board both wired to it |
| Relay board | `192.168.0.24` | Hardcoded `BOARD_STATIC_IP`; pingable even with no agent |
| micro-ROS agent | `192.168.0.4:8888` | Hardcoded `MICRO_ROS_AGENT_IP/PORT` — the laptop (or whoever runs the agent) **must own this IP** (static lease or alias) |
| Laptop (today) | `192.168.0.219` via DHCP | Fine as-is until Stage 2, then it needs `.4` |

## Software dependencies

- [PlatformIO](https://platformio.org/) CLI (build/flash/monitor)
- `arduino` framework on the espressif32 platform (ESP32-S3, 16 MB flash)
- [`micro_ros_platformio`](https://github.com/micro-ROS/micro_ros_platformio)
  — **Jazzy** distribution, **Ethernet** transport
- `IBusBM` (i-BUS) — vendored in [`lib/IBusBM/`](lib/IBusBM/) because the
  registry version doesn't compile against arduino-esp32 3.x (see NOTES.md)
- Docker — runs the micro-ROS agent + ROS 2 CLI (see
  [`docs/test-bench/notes.md`](docs/test-bench/notes.md))

## Quick start

```bash
pio run                    # build
pio run -t upload          # flash over USB-CDC
pio device monitor         # serial log @ 115200 baud
```

Test bench (router, static IPs, agent, staged checklist) lives in
[`docs/test-bench/notes.md`](docs/test-bench/notes.md).

## Test checklist

Stages 0–5, one at a time — detail in
[`docs/test-bench/notes.md`](docs/test-bench/notes.md).

| Stage | Scope | Status |
| --- | --- | --- |
| 0 — Docs & diagrams | `d2 validate`, `gen_rc_map.py --check` | ✅ done |
| 1 — USB-only | build, flash, boot log, status line | ✅ done 2026-10-17 (GUI flash verified via CLI serial) |
| 2 — Ethernet + agent | Docker agent, topics, kill/recover | ✅ done 2026-10-17 (50 Hz joy, kill→WAITING→reconnect, no rearm) |
| 3 — RC link | ACTIVE / LOST / failsafe / ch6 stop / intermediate grace | ✅ done 2026-10-17 (estop, failsafe, no-TX boot, TX drop verified) |
| 4 — Relays + inputs | breadboard: switch→relay, DIN, buzzer/LED | ⬜ needs HW (2-pos switch→relay ✔ 2026-10-17; re-test ch7/ch8 3-pos after effective-value fix) |
| 5 — Regression | minimal sequence for future contributors | ⬜ after 1–3 green |

⚠️ **Anomaly 2026-10-17:** board runs a stale `microROS ESP32 Test Suite`
image (`Fatal error - halting!`) — reflash with this repo before Stage 1.

## Onboarding diagrams (D2)

Diagrams for onboarding in [`docs/diagrams/`](docs/diagrams/notes.md):
system architecture, firmware blocks, safety permission, cooperative loop,
and a generated RC channel map
(`python3 docs/diagrams/gen_rc_map.py --check`).

## Repository layout

| Module | Ownership |
| --- | --- |
| [`src/main.cpp`](src/main.cpp), [`src/app.cpp`](src/app.cpp), [`include/karr/app.h`](include/karr/app.h) | Arduino entry points, subsystem wiring, boot order, control task, ROS-loop orchestration, and periodic status |
| [`src/relay_board.cpp`](src/relay_board.cpp), [`include/karr/relay_board.h`](include/karr/relay_board.h) | TCA9554 relay writes/retries, digital inputs, LED, buzzer, and board timing |
| [`include/karr/rc_config.h`](include/karr/rc_config.h), [`src/rc_receiver.cpp`](src/rc_receiver.cpp), [`include/karr/rc_receiver.h`](include/karr/rc_receiver.h) | Channel definitions and i-BUS decoding, receiver state, stop reasons, confirmation, and recovery |
| [`src/relay_feature_controller.cpp`](src/relay_feature_controller.cpp), [`include/karr/relay_feature_controller.h`](include/karr/relay_feature_controller.h) | Switch-to-relay bindings, validation, feature permission, output mask, and sole runtime relay writer |
| [`include/karr/control_snapshot.h`](include/karr/control_snapshot.h), [`src/state_exchange.cpp`](src/state_exchange.cpp), [`include/karr/state_exchange.h`](include/karr/state_exchange.h) | Value-only telemetry snapshot, length-one queue, and agent-availability atomic handoff |
| [`src/diagnostics.cpp`](src/diagnostics.cpp), [`include/karr/diagnostics.h`](include/karr/diagnostics.h) | Bounded queued logging and dropped-event count |
| [`src/micro_ros_bridge.cpp`](src/micro_ros_bridge.cpp), [`include/karr/micro_ros_bridge.h`](include/karr/micro_ros_bridge.h) | Ethernet/micro-ROS setup, agent lifecycle, ROS entities, topics, QoS, publication, and time sync |

Dependencies flow inward through the application wiring: the feature
controller uses the receiver and board; the bridge consumes copied
snapshots; the state-exchange and diagnostics modules provide bounded
cross-task handoffs. `app.cpp` is the composition root. There are no live
receiver/board reads in the ROS bridge.

**Where to edit:** receiver channels in [`include/karr/rc_config.h`](include/karr/rc_config.h);
bindings in [`src/relay_feature_controller.cpp`](src/relay_feature_controller.cpp);
hardware pins/register constants in [`src/relay_board.cpp`](src/relay_board.cpp);
network, topic, QoS, and ROS timing settings in [`src/micro_ros_bridge.cpp`](src/micro_ros_bridge.cpp);
task setup/cadence and boot sequencing in [`src/app.cpp`](src/app.cpp).
The channel-map generator reads the RC table from `src/rc_receiver.cpp`
and bindings from `src/relay_feature_controller.cpp`.

## Status

Baseline stage: Ethernet + micro-ROS telemetry + local RC safety logic are
working on hardware. Relay feature names in
`SWITCH_TO_RELAY_BINDINGS[]` are **placeholders** — only the ch6 wireless
e-stop is a defined safety function today. See the Roadmap in
[NOTES.md](NOTES.md) for what's next.

## License

Apache 2.0 — see [LICENSE](LICENSE).
