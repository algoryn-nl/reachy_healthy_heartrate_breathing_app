# Development Log & TODOs

## Tasks

### Current

(nothing actively in-progress)

### Done (recent)

- [x] **PY-REFACTOR-1**: Reframe BH1750 lux sensor from ambient light to proximity/occlusion — rewrite `light_context.py` (new states: `clear_path`, `close_presence`, `sudden_occlusion`, `partial_occlusion`), simplify `light_orchestrator.py` (remove EMA baseline system, baseline persistence, time-of-day logic), update `openai_realtime.py` constructor, remove `flush()`, update instructions.txt and dashboard label, analytics schema v2 with `PRAGMA user_version` migration, remove 10 obsolete env vars, add `HEALTHY_LIGHT_MODERATE_LUX_THRESHOLD` (2026-03-03)

### Upcoming — Critical (before next release)

- [x] **FW-CRIT-1**: Fix vitals hysteresis bypass in `emitBio()` — require `vitalsStreak >= VITALS_CONFIRM` in `br_emit_ok`/`hr_emit_ok` (`reachy-sensor.ino`) (2026-02-28)
- [x] **FW-CRIT-2**: Add expiry to cached vitals (`lastBR`/`lastHR`) — invalidate after 2 s to prevent stale reporting under fallback lock (`reachy-sensor.ino`) (2026-02-28)
- [x] **FW-CRIT-3**: Fix race condition in presence detection — restructure `dist_ok` check to use actual (possibly fallback) distance value (`reachy-sensor.ino`) (2026-02-28)
- [x] **PY-CRIT-1**: Replace `sys.exit()` with exceptions in `config.py:60-64` and `prompts.py:86,88,91` — let `main.py` handle shutdown (2026-02-28)
- [x] **PY-CRIT-2**: Add path traversal validation in `headless_personality.py:66` and `gradio_personality.py:64,188` — check `resolve().is_relative_to()` (2026-02-28)
- [x] **PY-CRIT-3**: Fix falsy-zero threshold bug in `light_context.py` — replace `value or DEFAULT` with direct use of already-resolved variables (2026-02-28)

### Upcoming — High Priority

- [x] **PY-HIGH-1**: Wrap `base64.b64decode()` in try/except in `audio_router.py:41` (2026-02-28)
- [x] **PY-HIGH-2**: Add locking to tool registry globals in `tools/core_tools.py:38-40` — `_REGISTRY_LOCK` with double-checked locking; 2 thread-safety tests (2026-03-01)
- [x] **PY-HIGH-3**: Add independent `asyncio.wait_for()` timeout to auto light_context dispatch in `tool_dispatcher.py:397-411` (2026-02-28)
- [x] **PY-HIGH-4**: Fix HeadWobbler TOCTOU race — consolidated generation check + `_base_ts` init into single lock hold; generation guard inside `_sway_lock` before `sway.feed()`; `reset()` wraps state bump + `sway.reset()` in `_sway_lock` for atomicity; documented lock ordering (`_sway_lock` → `_state_lock`); 4 new thread-safety tests (`audio/head_wobbler.py`, `tests/audio/test_head_wobbler.py`) (2026-03-01)
- [x] **PY-HIGH-5**: Guard `MovementManager.start()` against thread leak — `_lifecycle_lock` serialises `start()`/`stop()`; `is_alive()` guard + 7 tests incl. concurrent-safety (`moves.py`, `tests/test_moves.py`) (2026-03-01)
- [x] **PY-HIGH-6**: Fix type annotation `robot: ReachyMini = None` → `robot: ReachyMini | None = None` (`main.py:39`) (2026-02-28)
- [x] **FW-HIGH-1**: Handle COBS encode overflow — increment `txDropCount` diagnostic counter on silent drop; surfaced via EVT_DIAG (`reachy-sensor.ino`) (2026-02-28)
- [x] **FW-HIGH-2**: Add sensor error telemetry — `mmwaveFailCount`/`mmwaveConsecutiveFails` counters + periodic EVT_DIAG emission (`reachy-sensor.ino`) (2026-02-28)
- [x] **FW-HIGH-3**: Rate-limit state telemetry — 200 ms minimum interval between change-triggered EVT_STATE emissions (`reachy-sensor.ino`) (2026-02-28)

### Upcoming — Medium Priority

- [x] **PY-MED-1**: Remove redundant `GET /personalities/save_raw` endpoint — POST variant already exists (`headless_personality_ui.py`) (2026-03-01)
- [x] **PY-MED-2**: Remove duplicate sample rate assignments in `openai_realtime.py` — single typed assignment with `Literal[24000]` (2026-03-01)
- [x] **PY-MED-3**: Defer `last_activity_time` init to `_run_realtime_session()` in `openai_realtime.py` — `__init__` sets `0.0`, session start sets loop time; no lock needed (single async thread); 2 tests (2026-03-01)
- [x] **PY-MED-4**: Fix float equality comparison for timestamps in `camera_worker.py` — use `abs() < 1e-9` (2026-03-01)
- [x] **PY-MED-5**: Make `move_queue` private (`_move_queue`) in `moves.py` — public API via `clear_move_queue()` unchanged (2026-03-01)
- [x] **PY-MED-6**: Replace API key polling loop with `threading.Event` — `_api_key_event` signaled from `_persist_api_key()`; no more busy-poll (`console.py`) (2026-03-01)
- [x] **PY-MED-7**: Decompose launch() into smaller methods in console.py — extracted _load_instance_env(), _acquire_api_key_from_hf(), _wait_for_api_key(), _run_async_session() (2026-03-03)
- [x] **PY-MED-8**: Migrate analytics from JSONL to SQLite in `light_orchestrator.py` — `_init_analytics_db()` creates WAL-mode DB with flattened `light_events` table; time-based retention via `_prune_analytics()` (configurable via `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS`, default 90 days); lazy DB init on first write; 8 tests (2026-03-01)
- [x] **PY-MED-9**: Platform-aware serial port glob ordering in mmWave `_resolve_serial_port()` — `sys.platform`-conditional patterns: macOS gets `cu.usbmodem`/`tty.usbmodem`, Linux gets `ttyACM`/`ttyUSB` (2026-03-01)
- [x] **PY-MED-10**: Standardize tool error handling — `tool_ok()`/`tool_error()` helpers in `core_tools.py`; all 13 tool files migrated; error dicts always have `"error"` key, success dicts always have `"status"` key (2026-03-04)
- [x] **FW-MED-1**: Add BH1750 periodic re-init on failure — retry `begin()` every 10s when `!lightSensorReady` (`reachy-sensor.ino`) (2026-03-01)
- [x] **FW-MED-2**: Encapsulate firmware globals into 6 structs — `VitalsCache`, `PresenceState`, `EmitTimers`, `DiagCounters`, `HostSettings`, `PrevState` (`reachy-sensor.ino`) (2026-03-01)
- [x] **FW-MED-3**: Document buffer sizing rationale; add `static_assert` for `txPayloadBuf` vs `MAX_TARGETS_WIRE` (`reachy-sensor.ino`) (2026-03-01)
- [x] **FW-MED-4**: Add `CMD_SET_GUARD_RAILS` (0x06) for runtime-configurable BR/HR guard rails — 4x u16 centi-bpm payload, clamped to absolute physiological bounds, 6 tests (2026-03-05)
- [x] **FW-MED-5**: Add hysteresis to `guessPose()` — ±5cm dead zone (50–60cm), holds previous pose in band (`reachy-sensor.ino`) (2026-03-01)

### Upcoming — Low Priority

- [x] **PY-LOW-1**: Validate IdlePolicy constructor parameters (reject negative/zero timings) — 9 `ValueError` checks matching constraint table; 12 tests (2026-03-01)
- [x] **PY-LOW-2**: Log warning in `_safe_load_obj()` instead of silent `{}` return (2026-03-04)
- [x] **PY-LOW-3**: Clean up `sys.modules` on `exec_module()` failure in `_load_module_from_file()` — try/except + pop on error; 2 tests (2026-03-01)
- [x] **PY-LOW-4**: Add error handling + named constants to `sweep_look.py` — `SWEEP_MAX_ANGLE_RAD`, `SWEEP_TRANSITION_S`, `SWEEP_HOLD_S`; try/except returns error dict; docstring documents angle difference from mmWave; 3 tests (2026-03-01)
- [x] **PY-LOW-5**: Consolidate personality REST save endpoints — redundant GET variant removed with PY-MED-1 (2026-03-01)
- [x] **FW-LOW-1**: Replace `0x00` magic literal with `FRAME_DELIMITER` named constant (2026-03-04)
- [x] **FW-LOW-2**: Add firmware unit test harness — reachy_codec.h extracted, ctypes shim, 41 cross-validation tests (2026-03-04)

### Upcoming — Test Coverage Gaps

- [x] Add AudioRouter tests for malformed base64, empty audio, odd-byte-length (5 tests; 2026-02-28)
- [ ] Add TranscriptHandler tests for rapid concurrent `on_partial()` calls
- [ ] Add ToolDispatcher tests for malformed sensor state JSON (light_context timeout covered by PY-HIGH-3)
- [x] Add mmWave tests for serial timeout/disconnection recovery and dropped frames in `_poll_events()` (17 tests; 2026-02-28)
- [x] Add sweep_look failure-path tests (robot operation errors) (3 tests; 2026-03-01)
- [x] Add multi-threaded tests for `head_wobbler.py` — deadlock stress test, atomicity test, sway state invariants, rapid reset cycles (4 tests; 2026-03-01)
- [ ] Add multi-threaded tests for `moves.py`
- [x] Add `light_context.py` test for threshold explicitly set to `0.0` (falsy-zero regression; 4 tests; 2026-02-28)
- [x] Add personality management path traversal security tests (33 tests; 2026-02-28)
- [ ] Replace `asyncio.sleep(0.05)` timing in tests with deterministic mechanisms
- [ ] Convert test factory functions (`_policy()`, `_orchestrator()`) to proper pytest fixtures

### Done

- [x] Full-stack code review (firmware + Python) — 44 issues identified across 5 severity levels; documented in `docs/plans/2026-02-27-code-review.md` (2026-02-27)
- [x] Device state context integration — `build_device_context()` pure function injected into mmWave results via ToolDispatcher; vitals reliability mapping, state transition detection, system prompt guidance; 17 new tests (2026-02-27)
- [x] Multi-person tracking logic — IdlePolicy multi-target awareness (record_multi_target, suggest_scan_only, configurable interval multiplier), ToolDispatcher routing, scan-only idle mode, system prompt hint; 11 new tests (2026-02-26)
- [x] Protocol version handshake rejection — `_handshake_version()` sends CMD_PING at session start, validates first frame's version byte, returns version_mismatch error on failure; 6 new tests (2026-02-26)
- [x] Bio rate boundary condition tests — 22 tests for decode at firmware guard rails (BR 4–30, HR 35–200), null sentinels, wire edge cases, tool acceptance gate (2026-02-26)
- [x] LightOrchestrator file I/O error tests — 21 tests for corrupted/truncated/binary JSON, non-dict repair, permission errors on read/write/flush, analytics write failures, prune edge cases (2026-02-26; baseline I/O tests removed in proximity/occlusion reframe, 2026-03-03 — replaced with analytics schema migration test)
- [x] Add test coverage for openai_realtime.py — 71 total tests (60 new) covering all public methods: receive (mono/stereo/resample), emit (idle logic with mmWave/non-mmWave policy gating), send_idle_signal, full event loop routing, shutdown cleanup, apply_personality, restart_session, helpers, API key persistence; fixed scipy float64 resample bug in receive() (2026-02-24)
- [x] Graceful degradation when mmWave sensor disconnects mid-session — error tracking in IdlePolicy (record_error, consecutive error counter, backoff suppression), error propagation through ToolDispatcher to sensor_state, dashboard shows "Disconnected" chip + error banner, stale data detection (2026-02-24)
- [x] Fix antenna blending race condition — false positive; only real issue was `get_status()` reading `_is_listening` off-thread, fixed to use `_shared_is_listening` (2026-02-24)
- [x] Fix session setup exception handling gap — try/finally around event loop clears connection + connected_event (2026-02-24)
- [x] Reset idle tool call flag on exception path in emit() (2026-02-24)
- [x] Add receive() connection guard regression test (2026-02-24)
- [x] Add baseline pruning to LightOrchestrator (unbounded per-hour growth) (2026-02-24; baseline system removed in proximity/occlusion reframe, 2026-03-03)
- [x] Use `decimal.Decimal` for cost tracking — eliminates float precision loss over long sessions (2026-02-24)
- [x] Extract magic numbers in moves.py to named constants (2026-02-24)
- [x] Remove dead commented-out code in HeadWobbler (2026-02-24)
- [x] Document IdlePolicy state transitions with state diagram (2026-02-24)
- [x] Surface unused BIO state field and targets truncation flag in scan/measure results + sensor dashboard (2026-02-24)
- [x] Refactor tool registry to lazy init — no disk I/O or sys.exit at import time (2026-02-24)
- [x] Validate tool loading matches tools.txt — warns on unregistered tool names (2026-02-24)
- [x] Pin Gradio to stable version — `>=5.50.0` replaces dev pin (2026-02-24)
- [x] Add mypy coverage for tests/ (2026-02-24)
- [x] Align requires-python to `>=3.12` matching mypy target and actual runtime (2026-02-24)
- [x] Verify lux extraction paths match mmWave output — confirmed correct, added regression tests (2026-02-24)
- [x] Throttle light baseline saves with dirty flag + 60s interval, flush at shutdown (2026-02-24; baseline system removed in proximity/occlusion reframe, 2026-03-03)
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

## Notes

### Development Decisions

- handleDecodedPacket() buffer validation already sufficient: exact length check + per-command payload validation (2026-02-23)
- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- IdlePolicy + LightOrchestrator extracted as first phase of handler decomposition (2026-02-23)
- Lux extraction paths in env_utils.py verified against mmWave output: all 4 paths (measure/scan x latest_light/light_summary) match (2026-02-24)
- scipy.signal.resample returns float64 from int16 input; fastrtc's audio_to_int16 only accepts int16/float32 — fixed by casting to float32 after resample in receive() (2026-02-24)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Technical Debt

#### Active (from 2026-02-27 code review)

- ~~**`sys.exit()` in library code** (`config.py`, `prompts.py`): Blocks testability, prevents graceful error recovery.~~ (fixed: `ConfigError` exception + `main.py` catch; 2026-02-28)
- ~~**Path traversal in personality management**: User-supplied profile names not validated against directory escape.~~ (fixed: `resolve().is_relative_to()` containment checks in both headless and Gradio paths + `_write_profile` defense-in-depth; 33 security tests; 2026-02-28)
- ~~**Falsy-zero default pattern**: `value or DEFAULT` treats 0.0/0/""/False as missing. Confirmed bug in `light_context.py`, likely present elsewhere.~~ (fixed: removed redundant `or DEFAULT` fallbacks — thresholds already resolved via `coerce_float()` with defaults; audited codebase, no other instances; 4 regression tests; 2026-02-28)
- **Thread safety gaps**: ~~Tool registry globals unprotected~~ (fixed: `_REGISTRY_LOCK` with double-checked locking, 2026-03-01); camera_worker state transitions unsynchronized. ~~MovementManager start/stop~~ (fixed: `_lifecycle_lock`, 2026-03-01). ~~HeadWobbler generation TOCTOU~~ (fixed: consolidated lock hold + generation guard inside `_sway_lock`, 2026-03-01). Priority: high.
- ~~**Firmware vitals logic bugs**: Hysteresis bypass in emitBio, stale vitals in fallback lock, dist_ok race condition.~~ (fixed: 2026-02-28)
- ~~**Error handling inconsistency**: Three patterns (error dicts, exceptions, sys.exit) coexist across the codebase.~~ (fixed: `tool_ok()`/`tool_error()` helpers standardize all tool returns; `sys.exit()` only in `main.py`; 2026-03-04)
- **Config singleton at import time**: `config = Config()` in `config.py` module body. Import-time failure cascades to all dependents. Priority: medium.

#### Resolved

- ~~`_run_realtime_session()` decomposition complete — 5 handler classes extracted~~ (2026-02-23)
- ~~Tool registry initializes at import time with disk I/O and potential `sys.exit(1)`~~ (fixed: lazy init + ToolRegistryError, 2026-02-24)
- ~~Test coverage inversely correlates with module size (`openai_realtime.py` worst)~~ (fixed: comprehensive test coverage added, 2026-02-24)
- ~~All planned testing gaps closed: bio rate boundaries, LightOrchestrator file I/O errors, openai_realtime.py coverage~~ (2026-02-26)

### Recurring Issues & Solutions

- **`or` vs `is not None`**: When accepting optional numeric parameters, always use `x if x is not None else default`. The `x or default` pattern silently replaces 0/0.0/False/"" with defaults.
- **Thread safety contracts**: Every module with multi-threaded access should document expected callers and synchronization requirements in its module docstring (see `moves.py` as the gold standard).
