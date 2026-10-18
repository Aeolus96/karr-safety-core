# Engineering workflow for karr-safety-core

Make the smallest correct, testable change. Preserve responsiveness and defined safety behavior. Prefer working simplicity over speculative frameworks.

## Instructions and scope
- Read applicable AGENTS.md instructions and these rules at task start. Preserve AGENTS.md as the shared source of project preferences; do not duplicate it into competing rule files.
- Follow the current user request. If repository instructions conflict with the request, identify the conflict before taking a consequential action; do not silently invent a resolution.
- In review/plan mode, investigate and propose changes without editing. In an authorized implementation task, continue through edits, relevant tests, diff review, and required documentation without asking permission between ordinary steps.
- Pause only for a real blocker, unresolved consequential policy choice, or required authorization. Completing an objective does not authorize expanding it.
- Never commit or push without explicit permission in the current task. Never discard unrelated edits, reset/clean the working tree, rewrite history, flash hardware, actuate outputs, or install/update dependencies without appropriate authorization.

## Discover before editing
- Confirm repository root and working-tree state with read-only Git commands. Inspect existing changes before touching affected files.
- Read relevant README.md/NOTES.md sections and inspect the nearby file layout once. Do not reread entire documents for every edit or assume documentation overrides implementation.
- Trace the affected path: input -> decode/state -> permission -> output -> telemetry. Inspect definitions, callers, initialization, recovery, and tests relevant to the change.
- Verify claims against the actual installed dependencies and build configuration when behavior is version-sensitive. Use official documentation to resolve missing API facts; do not assume a current online example matches the installed version.
- Use only tools actually available in this session. Do not require invented tool names such as search_grep or list_dir.

## Search efficiently
- Prefer scoped ripgrep searches for terminal-based code search. Built-in code search and symbol-navigation tools are equally acceptable when they return usable results.
- Discover filenames first when the location is unknown. Search likely source directories before the whole repository. Prefer fixed-string searches for literal identifiers and regex only when needed.
- Examples:
  - `rg --files -g 'AGENTS.md' -g '*receiver*' -g '*test*'`
  - `rg -n -F 'receiverLocalStop' src lib test`
  - `rg -n -C 12 'signature_confirmed_|decoded_ch6_stop_' src/main.cpp`
  Adapt paths to the repository; missing directories are not evidence that a symbol does not exist.
- Exclude .git, .pio, generated output, binaries, and vendored dependencies from routine searches. Search the specific dependency subtree deliberately when investigating library behavior.
- Read complete relevant functions and bounded contiguous sections, including surrounding branches. Do not repeatedly read a large file in tiny line-by-line slices or dump it all to locate one definition.
- Avoid unscoped recursive grep/find and huge terminal output. This is an efficiency rule, not a blanket ban on ordinary shell utilities. If rg is unavailable, use built-in search or an available scoped fallback; do not install tools without authorization.
- Treat rg exit code 1 as no matches, not a tool failure. If output is truncated, narrow the search; never claim the result is exhaustive.

## Diagnose before changing
- Separate observed behavior, source-proven defects, hypotheses, and unknowns. Do not promote a plausible cause into a confirmed diagnosis.
- For timing issues, identify where time is spent: reception, parsing, permission, hardware write, physical response, or observation. Do not adjust thresholds just to hide a symptom.
- A schedule is not a worst-case bound. Distinguish wakeup lateness, service gaps, work duration, transport waits, and physical relay latency. Instrument the actual operation, not adjacent work.
- A successful register write is not contact feedback. A clean build is not a behavior test. Freshly parsed buffered bytes are not automatically fresh arrival evidence.

## Implement narrowly
- Preserve the approved task structure, single relay writer, bounded logging handoff, and coherent telemetry snapshots unless changing them is explicitly in scope.
- Keep synchronous logging, ROS calls, and unbounded waits off the local control path. Shared mutable state needs a correct synchronization/ownership strategy, not volatile.
- Keep ordinary switch decoding, failsafe confirmation, receiver timeout, and recovery eligibility distinct. Do not silently change thresholds, grace, stop scope, automatic recovery, agent dependency, or rearm behavior.
- Resetting pending timing evidence must not accidentally clear confirmed faults or restore stale permission. Recovery must satisfy the documented fresh-input requirements.
- Check every channel kind and binding when changing shared helpers. Verify mask width, initialization, timer rollover, and boundary comparisons. Do not apply two-position retention semantics to legitimate three-position middles.
- Avoid speculative feature frameworks, unrelated refactors, new tasks, generic retry machinery, or additional safety subsystems without a demonstrated need. Keep dependencies pinned; do not upgrade to fix an unrelated issue.
- Match existing naming and formatting. Comment important invariants and non-obvious reasons, not every statement. Do not expand comments into repeated design essays.

## Verify behavior, not just compilation
- For a reported bug, add the smallest practical regression test; demonstrate failure before the fix when feasible. Test the actual production logic rather than a separately rewritten model.
- For receiver/state changes, exercise time-ordered inputs: valid endpoints, brief intermediate, sustained signature, category wobble, implausible input, missing frames, recovery, and independent channel clearing. Include previously working channel types and outputs.
- Use an injectable clock or supplied timestamps for deterministic timing tests where practical. Test threshold boundaries and retained-state eligibility across loss/recovery, not only happy paths.
- Run relevant tests and the project build. Inspect the final diff and `git diff --check`; do not claim the build is clean if it failed, emitted unreviewed warnings, or was not run.
- Maintain macOS and Ubuntu compatibility. Do not hardcode user directories, serial-device paths, or local environment assumptions.
- Hardware-only claims remain unverified until authorized bench testing is performed. Keep hazardous actuation disabled during tests. Report measured results separately from expected behavior.
- Update only documentation and diagrams affected by the actual behavior change. Follow AGENTS.md for README, D2, and commit reminders. Search for contradictions, not just obsolete words.

## Completion report
- State what changed and why, then give verification results and remaining limitations. Keep it concise; do not repeat the plan or reproduce large diffs.
- Distinguish: static review, deterministic tests, build/link checks, and hardware observations. List commands actually run and their results. Never turn a test checklist into a claim that tests passed.
- For review findings, include severity, function/file location, a concrete triggering sequence, and the smallest corrective direction. Do not pad the review with hypothetical issues unrelated to scope.
- If blocked, explain the exact blocker and completed work. Never fabricate tool access, code edits, clean builds, or bench results.
