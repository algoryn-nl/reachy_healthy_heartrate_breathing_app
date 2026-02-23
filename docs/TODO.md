# Development Log & TODOs

## Tasks

### Current

(nothing actively in-progress)

### Upcoming — High Priority

- [ ] Decompose `_run_realtime_session()` (~320 lines) into smaller handlers
- [ ] Add `asyncio.wait_for()` timeout to tool dispatch (openai_realtime.py:567)
- [ ] Add test coverage for openai_realtime.py (emit, receive, idle, tool dispatch)
- [ ] Fix race condition in antenna blending (moves.py:599-633)
- [ ] Surface protocol version mismatch as ERROR, not DEBUG (mmWave.py:162-164)
- [ ] Fix session setup exception handling gap (openai_realtime.py:415-417)

### Upcoming — Medium Priority

- [ ] Reset idle tool call flag on exception path (openai_realtime.py:538-583)
- [ ] Batch light baseline saves with timer-based flush (light_orchestrator.py:156)
- [ ] Fix lux extraction path inconsistency (env_utils.py:161-185)
- [ ] Improve serial port auto-detection with device signature filtering (mmWave.py:118-126)
- [ ] Refactor tool registry to avoid global side effects at import (core_tools.py:286)
- [ ] Validate tool loading matches tools.txt expectations (core_tools.py:143-264)
- [ ] Pin Gradio to stable version (pyproject.toml)
- [ ] Add mypy coverage for tests/ (pyproject.toml)
- [ ] Resolve Python version mismatch: requires-python >=3.10 vs mypy target 3.12

### Upcoming — Low Priority

- [ ] Add baseline pruning to LightOrchestrator (unbounded per-hour growth)
- [ ] Use `decimal.Decimal` for cost tracking (openai_realtime.py:43-55)
- [ ] Extract magic numbers in moves.py to named constants (lines 266-282)
- [ ] Remove dead commented-out code in HeadWobbler (head_wobbler.py:149-157)
- [ ] Document IdlePolicy state transitions with state diagram
- [ ] Surface unused BIO state field and targets truncation flag (mmWave.py)

### Upcoming — Firmware

- [ ] Add buffer length validation in `handleDecodedPacket()` (reachy-sensor.ino:590)
- [ ] Add `isfinite()` checks before coordinate encoding (reachy-sensor.ino:450-456)
- [ ] Signal light sensor init failures to host (reachy-sensor.ino:668)

### Done

- [x] Extract IdlePolicy from openai_realtime handler (2026-02-23)
- [x] Extract LightOrchestrator from openai_realtime handler (2026-02-23)
- [x] Add logging to silent exception handlers (2026-02-23)
- [x] Consolidate duplicated utility functions into env_utils.py (2026-02-23)
- [x] Add mmWave env tuning defaults (2026-02-23)

## Notes

### Development Decisions

- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- IdlePolicy + LightOrchestrator extracted as first phase of handler decomposition (2026-02-23)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Technical Debt

- `openai_realtime.py` `_run_realtime_session()` is ~320 lines — decomposition started but not complete
- Tool registry initializes at import time with disk I/O and potential `sys.exit(1)`
- Test coverage inversely correlates with module size (`openai_realtime.py` worst)

### Recurring Issues & Solutions

(to be populated as issues arise)
