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

The firmware uses **static IPs** (`src/main.cpp` BLOCK 3) — no DHCP for the
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

```
platformio.ini       PIO environment, build flags, micro-ROS config
micro_ros.meta       micro-ROS entity limits (publishers/subscriptions)
src/main.cpp         All firmware — organized as 5 split-ready feature blocks
lib/IBusBM/          Vendored i-BUS library (arduino-esp32 3.x fix)
partitions_*.csv     16 MB flash partition table
NOTES.md             Design decisions, safety semantics, validation plans
docs/diagrams/       D2 onboarding diagrams + notes.md + gen_rc_map.py
AGENTS.md            Agent working preferences (git safety, docs, platform rules)
```

## Status

Baseline stage: Ethernet + micro-ROS telemetry + local RC safety logic are
working on hardware. Relay feature names in
`SWITCH_TO_RELAY_BINDINGS[]` are **placeholders** — only the ch6 wireless
e-stop is a defined safety function today. See the Roadmap in
[NOTES.md](NOTES.md) for what's next.

## License

Apache 2.0 — see [LICENSE](LICENSE).
