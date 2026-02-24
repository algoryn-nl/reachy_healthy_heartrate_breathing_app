# Development Log & TODOs

## Tasks

### Current

(nothing actively in-progress)

### Upcoming — High Priority

- [ ] Add test coverage for openai_realtime.py (emit, receive, idle, tool dispatch)
- [x] Fix antenna blending race condition — false positive; only real issue was `get_status()` reading `_is_listening` off-thread, fixed to use `_shared_is_listening` (2026-02-24)
- [x] Fix session setup exception handling gap (openai_realtime.py:415-417) (2026-02-24)

### Upcoming — Medium Priority

- [x] Reset idle tool call flag on exception path (openai_realtime.py:538-583) (2026-02-24)
- [x] Refactor tool registry to lazy init — no disk I/O or sys.exit at import time (2026-02-24)
- [x] Validate tool loading matches tools.txt — warns on unregistered tool names (2026-02-24)
- [x] Pin Gradio to stable version — `>=5.50.0` replaces dev pin (2026-02-24)
- [x] Add mypy coverage for tests/ (2026-02-24)
- [x] Align requires-python to `>=3.12` matching mypy target and actual runtime (2026-02-24)

### Upcoming — Low Priority

- [x] Add baseline pruning to LightOrchestrator (unbounded per-hour growth) (2026-02-24)
- [x] Use `decimal.Decimal` for cost tracking (openai_realtime.py:43-55) — cost constants and accumulator converted to Decimal; eliminates float precision loss over long sessions (2026-02-24)
- [x] Extract magic numbers in moves.py to named constants (lines 266-282) — BreathingMove params, neutral goto duration, and telemetry log interval extracted as documented module-level constants (2026-02-24)
- [x] Remove dead commented-out code in HeadWobbler (head_wobbler.py:149-157) (2026-02-24)
- [x] Document IdlePolicy state transitions with state diagram — full state diagram, transition table, and parameter reference in `idle_policy.py` module docstring; summary in README.md and CLAUDE.md (2026-02-24)
- [x] Surface unused BIO state field and targets truncation flag (mmWave.py) — `device_state`, `max_target_count`, `targets_truncated` surfaced in scan/measure results; sensor dashboard added to headless UI with `/sensor` REST endpoint (2026-02-24)

### Done

- [x] Graceful degradation when mmWave sensor disconnects mid-session — error tracking in IdlePolicy (record_error, consecutive error counter, backoff suppression), error propagation through ToolDispatcher to sensor_state, dashboard shows "Disconnected" chip + error banner, stale data detection (2026-02-24)
- [x] Improve serial port auto-detection with VID/PID + HELLO probe (2026-02-23)
- [x] Skip NaN targets in emitTargets() firmware (2026-02-23)
- [x] Advertise light sensor status in HELLO feature bits (2026-02-23)
- [x] Surface protocol version mismatch at WARNING level (2026-02-23)
- [x] Decompose `_run_realtime_session()` into TranscriptHandler, ToolDispatcher, AudioRouter (2026-02-23)
- [x] Extract IdlePolicy from openai_realtime handler (2026-02-23)
- [x] Extract LightOrchestrator from openai_realtime handler (2026-02-23)
- [x] Add logging to silent exception handlers (2026-02-23)
- [x] Consolidate duplicated utility functions into env_utils.py (2026-02-23)
- [x] Add mmWave env tuning defaults (2026-02-23)
- [x] Non-blocking tool dispatch with timeout, semaphore, and cancellation (2026-02-23)
- [x] Verify lux extraction paths match mmWave output — confirmed correct, added regression tests (2026-02-24)
- [x] Throttle light baseline saves with dirty flag + 60s interval, flush at shutdown (2026-02-24)
- [x] Fix session setup exception handling gap — try/finally around event loop clears connection + connected_event (2026-02-24)
- [x] Reset idle tool call flag on exception path in emit() (2026-02-24)
- [x] Add receive() connection guard regression test (2026-02-24)

## Notes

### Development Decisions

- handleDecodedPacket() buffer validation already sufficient: exact length check + per-command payload validation (2026-02-23)
- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- IdlePolicy + LightOrchestrator extracted as first phase of handler decomposition (2026-02-23)
- Lux extraction paths in env_utils.py verified against mmWave output: all 4 paths (measure/scan x latest_light/light_summary) match (2026-02-24)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Technical Debt

- `_run_realtime_session()` decomposition complete — 5 handler classes extracted (IdlePolicy, LightOrchestrator, TranscriptHandler, ToolDispatcher, AudioRouter)
- ~~Tool registry initializes at import time with disk I/O and potential `sys.exit(1)`~~ (fixed: lazy init + ToolRegistryError, 2026-02-24)
- Test coverage inversely correlates with module size (`openai_realtime.py` worst)

### Recurring Issues & Solutions

(to be populated as issues arise)
