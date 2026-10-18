# Diagrams — onboarding visuals (D2)

Text-based diagrams: edit the `.d2`, the `.svg` re-renders. Kept small on
purpose — these are onboarding guidance + logic/data-flow checks, not
auto-synced mirrors of the code (see "Keeping diagrams honest" below).

Open any `.d2` in VS Code (D2 extension) for live preview, or:

```bash
d2 docs/diagrams/01-system-architecture.d2   # renders .svg next to it
d2 --watch docs/diagrams/01-system-architecture.d2  # live reload in browser
for f in docs/diagrams/*.d2; do d2 "$f" "${f%.d2}.svg"; done  # all
```

| Diagram | Source | What it shows |
|---|---|---|
| [System architecture](diagrams/01-system-architecture.svg) | [01-system-architecture.d2](diagrams/01-system-architecture.d2) | TX → RX → ESP32 → relays/inputs/LED, ROS observe-only, publish-only invariant |
| [Firmware modules](diagrams/02-firmware-blocks.svg) | [02-firmware-blocks.d2](diagrams/02-firmware-blocks.d2) | Module ownership, dependency direction, and cross-task handoffs |
| [Safety permission](diagrams/03-safety-permission.svg) | [03-safety-permission.d2](diagrams/03-safety-permission.d2) | Board-level + RC-group checks → single relay write (all-off = safe) |
| [Cooperative loop](diagrams/04-cooperative-loop.svg) | [04-cooperative-loop.d2](diagrams/04-cooperative-loop.d2) | `loop()` service order, incl. re-evaluation after network work |
| [RC channel map](diagrams/05-rc-channel-map.svg) *(generated)* | [gen_rc_map.py](diagrams/gen_rc_map.py) | Channel table + switch→relay bindings, parsed from their source modules |

## Keeping diagrams honest

- **No magic "D2 watches your C++".** D2 watches `.d2` files
  (`d2 --watch`), not source code. The VS Code extension previews
  `.d2` edits live — that's the tight loop.
- **Reminder, not automation:** when a commit is proposed, check whether
  any diagram needs a hand update (see `AGENTS.md`). Nothing runs per-edit.
- **One generated exception:** `gen_rc_map.py` parses
  `RC_CHANNEL_CONFIGURATION[]` from `src/rc_receiver.cpp` and
  `SWITCH_TO_RELAY_BINDINGS[]` from `src/relay_feature_controller.cpp`,
  then emits `05-rc-channel-map.d2`:
  `python3 docs/diagrams/gen_rc_map.py` to regenerate,
  `--check` for CI-style staleness detection.
- **Rule of thumb:** hand-draw architecture/safety/loop (they encode
  *intent*), generate fact diagrams (tables, bindings) from code.
