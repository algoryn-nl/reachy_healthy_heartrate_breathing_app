# Changelog

Detailed record of all resolved issues and completed roadmap phases.

## Resolved Issues

### Critical — Firmware

- **FW-CRIT-1**: Vitals hysteresis bypass in `emitBio()` — `br_emit_ok`/`hr_emit_ok` now require `vitalsStreak >= VITALS_CONFIRM` (2026-02-28)
- **FW-CRIT-2**: Stale vitals under fallback lock — 2s expiry on `lastBR`/`lastHR` via `VITALS_CACHE_EXPIRY_MS` (2026-02-28)
- **FW-CRIT-3**: Race condition in presence detection — `dist_ok` recomputed after fallback via `isFinitePositive(dist_cm)` (2026-02-28)

### Critical — Python

- **PY-CRIT-1**: `sys.exit()` in library code — replaced with `ConfigError` exception; `main.py` catches at entry point (2026-02-28)
- **PY-CRIT-2**: Path traversal vulnerability — `resolve().is_relative_to()` containment in both headless and Gradio paths + `_write_profile` defense-in-depth; 33 security tests (2026-02-28)
- **PY-CRIT-3**: Falsy-zero threshold bug — removed redundant `or DEFAULT` fallbacks; thresholds already resolved via `coerce_float()`; 4 regression tests (2026-02-28)

### High Priority

- **PY-HIGH-1**: Unhandled `base64.b64decode()` in `audio_router.py` — try/except + odd-byte guard; 5 tests (2026-02-28)
- **PY-HIGH-2**: Tool registry globals have no locking — `_REGISTRY_LOCK` with double-checked locking; 2 thread-safety tests (2026-03-01)
- **PY-HIGH-3**: Auto light_context dispatch has no independent timeout — `asyncio.wait_for()` with configurable timeout; 1 test (2026-02-28)
- **PY-HIGH-4**: HeadWobbler TOCTOU race — consolidated lock hold + generation guard inside `_sway_lock`; documented lock ordering; 4 thread-safety tests (2026-03-01)
- **PY-HIGH-5**: Thread leak in `MovementManager.start()` — `_lifecycle_lock` serialises `start()`/`stop()`; 7 tests (2026-03-01)
- **PY-HIGH-6**: Type annotation error — `robot: ReachyMini = None` → `ReachyMini | None = None` (2026-02-28)
- **FW-HIGH-1**: COBS encode overflow silently drops packets — `txDropCount` incremented on drop, surfaced via EVT_DIAG (2026-02-28)
- **FW-HIGH-2**: No sensor error telemetry — `mmwaveFailCount`/`mmwaveConsecutiveFails` counters + periodic EVT_DIAG emission (2026-02-28)
- **FW-HIGH-3**: No rate-limiting on EVT_STATE emission — 200 ms minimum interval between change-triggered emissions (2026-02-28)

### Medium Priority

- **PY-MED-1**: `GET /personalities/save_raw` has side effects — removed redundant GET variant; POST already exists (2026-03-01)
- **PY-MED-2**: Duplicate sample rate assignments in `openai_realtime.py` — single typed assignment (2026-03-01)
- **PY-MED-3**: `last_activity_time` initialized before event loop — deferred to `_run_realtime_session()`; 2 tests (2026-03-01)
- **PY-MED-4**: Float equality comparison for timestamps in `camera_worker.py` — `abs() < 1e-9` epsilon comparison (2026-03-01)
- **PY-MED-5**: Public `move_queue` bypasses thread-safe command queue — renamed to `_move_queue` (2026-03-01)
- **PY-MED-6**: API key polling loop (0.2s `time.sleep`) in `console.py` — `threading.Event` signaled from `_persist_api_key()` (2026-03-01)
- **PY-MED-7**: `launch()` overloaded (~63 lines) in `console.py` — decomposed into 4 methods (2026-03-03)
- **PY-MED-8**: Unbounded analytics JSONL growth in `light_orchestrator.py` — migrated to SQLite with time-based retention (2026-03-01)
- **PY-MED-9**: mmWave serial glob patterns ordered for macOS, not current platform — `sys.platform`-conditional (2026-03-01)
- **PY-MED-10**: Inconsistent error handling — `tool_ok()`/`tool_error()` helpers standardize all tool returns (2026-03-04)
- **FW-MED-1**: BH1750 light sensor init is one-shot — periodic re-init every 10s on failure (2026-03-01)
- **FW-MED-2**: 19 unstructured global variables — 6 flat structs (2026-03-01)
- **FW-MED-3**: Buffer sizes undocumented — sizing rationale + `static_assert` for MAX_TARGETS_WIRE (2026-03-01)
- **FW-MED-4**: Guard rails are compile-time only — `CMD_SET_GUARD_RAILS` (0x06), 4x u16 centi-bpm, 6 tests (2026-03-05)
- **FW-MED-5**: `guessPose()` has no hysteresis — ±5cm dead zone with previous-pose hold (2026-03-01)

### Low Priority

- **PY-LOW-1**: IdlePolicy constructor doesn't validate parameters — 9 `ValueError` checks; 12 tests (2026-03-01)
- **PY-LOW-2**: `_safe_load_obj()` returns `{}` on parse errors without logging — warns on non-dict parsed values (2026-03-04)
- **PY-LOW-3**: `_load_module_from_file()` pollutes `sys.modules` before `exec_module()` succeeds — try/except cleans on failure; 2 tests (2026-03-01)
- **PY-LOW-4**: `sweep_look.py` has no error handling — named constants, try/except, docstring; 3 tests (2026-03-01)
- **PY-LOW-5**: Three personality save endpoints — redundant GET variant removed (2026-03-01)
- **FW-LOW-1**: `0x00` frame delimiter not a named constant — `FRAME_DELIMITER` (2026-03-04)
- **FW-LOW-2**: No firmware unit tests — 15 pure functions in `reachy_codec.h`, ctypes shim, 41 pytest tests (2026-03-04)

### Other Resolved

- Handler logic covered via IdlePolicy/LightOrchestrator/TranscriptHandler/ToolDispatcher/AudioRouter tests + 71 direct `openai_realtime.py` tests (2026-02-24)
- Race condition in antenna blending: false positive — fixed `get_status()` to use `_shared_is_listening` (2026-02-24)
- Session setup exception returns early but connection object may already be set — try/finally (2026-02-24)
- Idle tool call flag not reset on exception path (2026-02-24)
- Light baseline save throttled with dirty flag + 60s interval (2026-02-24; baseline system removed 2026-03-03)
- Lux extraction paths verified correct (2026-02-24)
- Tool registry runs disk I/O and can `sys.exit(1)` at import time — lazy init + `ToolRegistryError` (2026-02-24)
- `tools.txt` entries silently skipped — post-registration validation (2026-02-24)
- Gradio pinned to dev version — `>=5.50.0` stable floor (2026-02-24)
- mypy only covers `src/` — added `tests/` (2026-02-24)
- `requires-python >=3.10` vs mypy 3.12 mismatch — aligned to `>=3.12` (2026-02-24)
- Unbounded per-hour baseline growth — stale user entries pruned (2026-02-24; removed 2026-03-03)
- Float precision loss in cost accumulation — `decimal.Decimal` (2026-02-24)
- Magic numbers in moves.py — extracted as named constants (2026-02-24)
- Dead commented-out code in HeadWobbler — removed (2026-02-24)
- IdlePolicy state machine transitions undocumented — documented (2026-02-24)
- BIO state field captured but never used — `device_state` surfaced (2026-02-24)
- Targets truncation flag extracted but never surfaced — surfaced in dashboard (2026-02-24)

---

## Completed Roadmap Phases

### Phase 1: Critical Fixes (v0.2.3) — COMPLETE (2026-02-28)

Firmware (3 items): emitBio hysteresis, cached vitals expiry, presence detection race.
Python (3 items): sys.exit → exceptions, path traversal validation, falsy-zero fix.
Tests (3 items): falsy-zero regression, path traversal security, malformed base64.

### Phase 2: Stability & Thread Safety (v0.3.0) — COMPLETE

Audio & Movement (4 items): base64 guard, HeadWobbler TOCTOU, MovementManager lifecycle, threading docs.
Tool System (3 items): registry locking, light_context timeout, error standardization.
Firmware (3 items): COBS overflow counter, error telemetry, EVT_STATE rate-limit.
REST API (2 items): redundant endpoint removal, type annotation fix.
Tests (4 items): multi-threaded tests, malformed sensor state, deterministic async, pytest fixtures.

### Phase 3: Code Quality & Maintainability (v0.3.x) — COMPLETE

Python Refactoring (7 items): duplicate assignments, deferred init, float equality, private queue, event-based signaling, launch decomposition, platform-aware globs.
Firmware Refactoring (5 items): struct encapsulation, buffer sizing, guessPose hysteresis, FRAME_DELIMITER, BH1750 re-init.
Observability (2 items): SQLite analytics migration, IdlePolicy validation.
Low-Priority Cleanup (4 items): safe_load_obj logging, sys.modules cleanup, sweep_look error handling, pytest fixtures.

### Phase 4: Firmware v2 (v0.4.0) — COMPLETE

1. CMD_SET_GUARD_RAILS — runtime-configurable BR/HR ranges (2026-03-05)
2. EVT_DIAG — error counters exposed (2026-02-28)
3. CMD_RESET — soft-reset for sensor recovery (2026-03-05)
4. Firmware unit test harness — 41 pytest tests via ctypes (2026-03-04)
5. Decoupled state machine — poll/tick architecture, 10 Hz fixed tick (2026-03-05)

---

## Completed Future Vision Items

- Handler decomposition: 5 standalone testable classes (2026-02-23)
- Non-blocking tool dispatch with `asyncio.create_task()` + `Semaphore(1)` + `wait_for()` timeout (2026-02-23)
- Protocol version handshake rejection at session start (2026-02-26)
- Graceful degradation when mmWave sensor disconnects mid-session (2026-02-24)
- Integration tests for async streaming loop with mock WebSocket (2026-02-24)
- Boundary condition tests for bio rate filtering (2026-02-26)
- File I/O error tests for LightOrchestrator persistence (2026-02-26; replaced with analytics schema migration test 2026-03-03)
- Multi-person tracking: IdlePolicy multi-target awareness, scan-only mode, system prompt hint (2026-02-26)
- Device state context: `build_device_context()` enriches mmWave results (2026-02-27)
- Sensor dashboard in headless UI (2026-02-24)
- Full-stack code review: 44 issues across firmware and Python (2026-02-27)
- Reframed BH1750 lux sensor from ambient light to proximity/occlusion signal (2026-03-03)
- Gradio sensor dashboard: live vitals card, radar canvas, vitals history graph, WebSocket push (2026-03-05)
- CMD_RESET mmWave radar recovery (2026-03-05)
- Decoupled state machine from sensor polling (2026-03-05)
- Health trend analysis: two-tier aggregation, TrendAnalyzer, dashboard trends panel (2026-03-05)
