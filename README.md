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
|---|---|
| Waveshare **ESP32-S3-POE-ETH-8DI-8RO-C** | Target board — ESP32-S3, 8 relay outputs (TCA9554 I2C expander), 8 digital inputs, W5500 Ethernet, status RGB LED + buzzer |
| Turnigy **TGY-i6S** | RC transmitter (MODE 1) |
| Turnigy **iA6B** | RC receiver — i-BUS serial link into the ESP32; **ch6 = wireless e-stop** (safety-reserved) |
| ROS 2 PC (**Jazzy**) | Runs the micro-ROS agent; static LAN (board `192.168.0.24`, agent `192.168.0.4:8888`) |

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

## Software dependencies

- [PlatformIO](https://platformio.org/) CLI (build/flash/monitor)
- `arduino` framework on the espressif32 platform (ESP32-S3, 16 MB flash)
- [`micro_ros_platformio`](https://github.com/micro-ROS/micro_ros_platformio)
  — **Jazzy** distribution, **Ethernet** transport
- `IBusBM` (i-BUS) — vendored in [`lib/IBusBM/`](lib/IBusBM/) because the
  registry version doesn't compile against arduino-esp32 3.x (see NOTES.md)
- ROS 2 Jazzy + `micro_ros_agent` on the host PC

## Quick start

```bash
pio run                    # build
pio run -t upload          # flash over USB-CDC
pio device monitor         # serial log @ 115200 baud
```

On the ROS PC:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic echo /rc/joy
ros2 topic echo /relay_board/heartbeat
```

## Repository layout

```
platformio.ini       PIO environment, build flags, micro-ROS config
micro_ros.meta       micro-ROS entity limits (publishers/subscriptions)
src/main.cpp         All firmware — organized as 5 split-ready feature blocks
lib/IBusBM/          Vendored i-BUS library (arduino-esp32 3.x fix)
partitions_*.csv     16 MB flash partition table
NOTES.md             Design decisions, safety semantics, validation plans
```

## Status

Baseline stage: Ethernet + micro-ROS telemetry + local RC safety logic are
working on hardware. Relay feature names in
`SWITCH_TO_RELAY_BINDINGS[]` are **placeholders** — only the ch6 wireless
e-stop is a defined safety function today. See the Roadmap in
[NOTES.md](NOTES.md) for what's next.

## License

Apache 2.0 — see [LICENSE](LICENSE).
