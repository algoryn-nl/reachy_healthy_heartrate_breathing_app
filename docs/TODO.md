# Development Log & TODOs

## Tasks

### Current

(nothing actively in-progress)

### Upcoming — High Priority

- [ ] Add `asyncio.wait_for()` timeout to tool dispatch (openai_realtime.py:567)
- [ ] Add test coverage for openai_realtime.py (emit, receive, idle, tool dispatch)
- [ ] Fix race condition in antenna blending (moves.py:599-633)
- [ ] Fix session setup exception handling gap (openai_realtime.py:415-417)

### Upcoming — Medium Priority

- [ ] Reset idle tool call flag on exception path (openai_realtime.py:538-583)
- [ ] Batch light baseline saves with timer-based flush (light_orchestrator.py:156)
- [ ] Fix lux extraction path inconsistency (env_utils.py:161-185)
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

### Done

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

## Notes

### Development Decisions

- handleDecodedPacket() buffer validation already sufficient: exact length check + per-command payload validation (2026-02-23)
- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- IdlePolicy + LightOrchestrator extracted as first phase of handler decomposition (2026-02-23)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Technical Debt

- `_run_realtime_session()` decomposition complete — 5 handler classes extracted (IdlePolicy, LightOrchestrator, TranscriptHandler, ToolDispatcher, AudioRouter)
- Tool registry initializes at import time with disk I/O and potential `sys.exit(1)`
- Test coverage inversely correlates with module size (`openai_realtime.py` worst)

### Recurring Issues & Solutions

(to be populated as issues arise)
