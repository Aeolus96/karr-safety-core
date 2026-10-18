# Test bench (bench notes live here, not in README)

## Network layout

The firmware uses **static IPs** (`src/micro_ros_bridge.cpp`) — no DHCP
for the board.

| Device | IP | Notes |
|---|---|---|
| Router (gateway) | `192.168.0.1` | GL.iNet Opal; laptop + board both wired to it |
| Relay board | `192.168.0.24` | Hardcoded `BOARD_STATIC_IP`; pingable even with no agent |
| micro-ROS agent | `192.168.0.4:8888` | Hardcoded `MICRO_ROS_AGENT_IP/PORT` — the laptop (or whoever runs the agent) **must own this IP** (static lease or alias) |
| Laptop (2026-10-17) | `192.168.0.219` via DHCP | Fine as-is until agent testing, then it needs `.4` |

Randomized MAC: the router note is a **client privacy feature** (phones /
laptops randomize the MAC *they* present). The ESP32 presents its burned-in
W5500 MAC and doesn't use DHCP at all — ignore it.

## micro-ROS via Docker (macOS + Ubuntu 24.04)

Pinned image `microros/micro-ros-agent:jazzy` for both agent and ROS CLI —
same commands on both OSes, no host ROS install.

Why not conda/robostack: the `ros2` env has `rclpy` but **no**
`micro_ros_agent` package exists in robostack-jazzy. Docker is the only
reliable agent source on macOS. Conda stays for general tinkering only.

```bash
# 1. Laptop must own .4 (board dials it). macOS:
sudo ifconfig en9 alias 192.168.0.4 netmask 255.255.255.0
# Ubuntu: sudo ip addr add 192.168.0.4/24 dev <iface>
ping -c2 192.168.0.24    # board reachable?

# 2. Agent:
docker run --rm -p 8888:8888/udp microros/micro-ros-agent:jazzy udp4 --port 8888

# 3. ROS CLI — note --entrypoint override (image defaults to the agent binary):
docker run --rm --entrypoint bash microros/micro-ros-agent:jazzy \
  -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /rc/joy"
```

macOS note: Docker Desktop can't do host-network multicast discovery
reliably, but UDP 8888 port-mapping works because the board dials a fixed
IP:port (no discovery needed board→agent). Ubuntu can also use
`--network host`.

## USB serial without an interactive terminal

`pio device monitor` needs a TTY. For scripts/agents use pyserial directly:

```bash
~/.platformio/penv/bin/python -c "
import serial, time
s = serial.Serial('<PORT>', 115200, timeout=1)
s.dtr = False; time.sleep(0.5); s.dtr = True  # reset, catch boot log
s.reset_input_buffer(); t = time.time()
while time.time() - t < 30: print(s.readline().decode(errors='replace').rstrip())
"
```

Platform: keep `~/.platformio/penv/bin` on PATH (`pio --version` → 6.x).
Never hardcode `/dev/` ports — let `pio` auto-detect USB.

## Staged bring-up detail (the checklist table lives in README)

Stage 1 expects (from `src/app.cpp` and the board/controller modules):
- `[RELAY] initialization completed` (else `INIT FAILED` → red SOLID LED,
  relays inhibited — check I2C/TCA9554)
- `[CFG] switch->relay bindings valid` (else purple blink)
- `[STATUS]` line every 5 s: `eth=up ip=192.168.0.24 agent=… rc=… relay=0x…`
- No agent + no RC: red blink (agent-unavailable wins priority),
  `agent=WAITING rc=LOST relay=0x00`

**Anomaly 2026-10-17:** board currently runs a stale `microROS ESP32 Test
Suite` image (`Fatal error - halting!`) — must be reflashed with this repo
before any further stage.

Stage 2 expects: `[ROS] agent connected`, `agent=CONNECTED`, topics
`/rc/joy` (~50 Hz), `/rc/link`, `/relay_board/{heartbeat,relay_state,
inputs,estop}` (~1 Hz). Kill agent → red blink, `CLEANUP`/`WAITING`,
relays `0x00`; restart → recovery, no rearm.

Stage 3 expects: TX ON → `rc=ACTIVE`, `/rc/link` true; TX OFF → `rc=LOST`
≤200 ms, Joy zeroed, yellow blink; all-1500 failsafe CONFIRMED (≥200 ms)
→ `rc=ESTOP-FS`; ch6 LOW → `rc=STOP` (reason bit 4) +
`/relay_board/estop` true; brief 2-pos mid-range bounce (e.g. ch5 through
1250..1750) must NOT cut that relay; sustained intermediate ≥200 ms →
`rc=STOP` (reason bit 5, RECV_STOP_INVALID_INPUT). LED priority:
red-solid (bus fault) > red-blink (no agent) >
yellow-blink (RC inhibited) > purple-blink (bad config) > green (all
clear — not contact verification).

Stage 4 (breadboard): RC switch → expected bit in `relay=0x…` +
`/relay_board/relay_state` on-change; inhibits drive `0x00` writes; ground
DIN GPIO 4–11 (active-low) → `/relay_board/inputs` bitmask; buzzer beeps
once per transition only.
