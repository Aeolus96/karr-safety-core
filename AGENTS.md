# AGENTS.md — persistent working preferences for this repo

> This file acts as local "permanent memory" for AI coding agents working in
> `karr-safety-core`, because agents have no cross-session memory of their own.
> Keep it up to date. It is checked into git so every agent + developer sees it.

## Git safety rule (user requirement, 2026-10-17)

- **NEVER `git commit` + `git push` automatically.**
- You MAY suggest a commit message and ask: "ready to commit + push?"
- Only run `git commit` / `git push` after the user gives explicit permission
  in that same task (a prior blanket permission does not count).
- Read-only git commands (`status`, `log`, `diff`) are always fine.

## Pre-commit reminder (user requirement, 2026-10-17)

- When about to PROPOSE a commit (not on every small edit), remind to check:
  1. Do any D2 diagrams in `docs/diagrams/` need updating for this change?
     (intent diagrams by hand; `05-rc-channel-map.d2` via
     `python3 docs/diagrams/gen_rc_map.py`)
  2. Does the test checklist in `README.md` need a checkbox update?
- Keep it a lightweight human reminder, not a hook or CI gate.

## Docs rule (user requirement, 2026-10-17)

- **One README only:** `README.md` at the repo root.
- Topic-specific notes live as small `notes.md` files alongside the thing
  they describe (e.g. `docs/diagrams/notes.md`), kept short and to the point.
- Never create a second `README.md` anywhere in the repo.

## Platform rule (user requirement, 2026-10-17)

- Keep the project working on **macOS + Ubuntu 24.04** (no Windows).
- No hardcoded `/dev/` paths or OS-specific assumptions in `platformio.ini`
  or scripts — let PlatformIO auto-detect the USB port; handle paths
  portably (e.g. `~/.platformio/penv/bin` on PATH for `pio`).

## Complete objectives without pausing (user requirement, 2026-10-17)

Never stop or hand control back to the user in the middle of a larger objective. If multiple steps remain (e.g., editing files, updating docs, running tests), proceed to the next step immediately using your tools without asking for feedback or pausing to summarize. Only stop when the overarching mission is 100% complete.
