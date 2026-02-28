# Code Review — v0.2.2 Full-Stack Audit

**Date**: 2026-02-27
**Scope**: Arduino firmware (`hardware/arduino/reachy-sensor/reachy-sensor.ino`, 838 lines) + Python host code (all `src/` modules, profile tools, tests)
**Rating**: 7.5/10 — solid v0.2 prototype; needs hardening for production

---

## Executive Summary

Strong architectural decisions: handler decomposition into 5 testable classes, callback-based dependency injection, profile-driven tool system, COBS+CRC binary protocol, lazy tool registry, and a 301-test suite. Issues fall into five categories: **critical firmware logic bugs**, **`sys.exit()` in library code**, **path traversal vulnerability**, **thread safety gaps**, and **maintainability debt**.

---

## Firmware Issues

### FW-CRIT-1: Vitals hysteresis bypassed in `emitBio()`

**File**: `reachy-sensor.ino` (state machine section)
**Severity**: Critical

```cpp
bool br_emit_ok = vitals_allowed && br_valid;
bool hr_emit_ok = vitals_allowed && hr_valid;
```

These flags allow individual vital emission even when `vitalsStreak` hasn't reached `VITALS_CONFIRM`. The streak counter exists specifically to prevent premature reporting after transient conditions, but `emitBio()` bypasses it. Should require `vitalsStreak >= VITALS_CONFIRM`.

### FW-CRIT-2: Fallback lock allows stale vitals

**File**: `reachy-sensor.ino` (presence logic)
**Severity**: Critical

When a target is briefly lost, `fallbackTargetLock` keeps vitals flowing for `TARGET_LOSS_GRACE_MS`. But `lastBR`/`lastHR` have no expiry — they could be arbitrarily old. If the target disappears for 999 ms (under grace window), stale vitals from minutes ago could be reported as current. Fix: add expiry to cached vitals (e.g., invalidate if older than 2 s).

### FW-CRIT-3: Race condition in presence detection with stale `dist_ok`

**File**: `reachy-sensor.ino` (presence logic)
**Severity**: Critical

After the sensor fails to return distance, the code falls back to `lastDist` but checks `dist_ok && isFinitePositive(dist_cm)` in presence logic. At that point `dist_ok` still reflects the failed reading, not the fallback. Presence detection is inconsistent with the actual distance used downstream.

### FW-HIGH-1: COBS encode failure silently drops packets

**File**: `reachy-sensor.ino` (COBS encoding)
**Severity**: High

If `cobsEncode()` returns 0 (overflow), the send function returns without error. Host never knows a packet was lost. Should send `EVT_ERR` or at minimum increment a diagnostic counter.

### FW-HIGH-2: No sensor error recovery

**File**: `reachy-sensor.ino` (`loop()`)
**Severity**: High

If `mmWave.update()` returns false, the code simply returns early. No retry, no error counter, no telemetry to host. Python side tracks errors via `IdlePolicy`, but firmware gives no visibility into sensor health.

### FW-HIGH-3: No rate-limiting on state telemetry

**File**: `reachy-sensor.ino` (telemetry emission)
**Severity**: High

Rapid state oscillation (e.g., sensor glitch causing STILL_NEAR ↔ MOVING) floods the host with `EVT_STATE` packets. Only the 1-second periodic fallback provides rate limiting. Should cap state-change emission frequency.

### FW-MED-1: BH1750 init is one-shot, no recovery

**File**: `reachy-sensor.ino` (`setup()`)
**Severity**: Medium

If I2C bus is flaky at startup, light telemetry is permanently lost. No retry mechanism. Should attempt periodic re-init if `lightSensorReady` is false.

### FW-MED-2: Global state explosion (19 statics)

**File**: `reachy-sensor.ino` (lines ~140–176)
**Severity**: Medium

Related state (presence tracking, vitals tracking, timing) is scattered across 19 unstructured globals. Should group into structs for clarity and encapsulation.

### FW-MED-3: Buffer sizes undocumented

**File**: `reachy-sensor.ino` (RX/TX buffers)
**Severity**: Medium

Buffer sizes (384, 512, 640 bytes) have no sizing rationale documented. Max payload is well under limits today, but no assertion or comment protects against overflow if `MAX_TARGETS_WIRE` increases.

### FW-MED-4: Guard rails hardcoded, not runtime-configurable

**File**: `reachy-sensor.ino` (constants section)
**Severity**: Medium

BR 4–30, HR 35–200, near zone 35–150 cm are compile-time only. Different users or environments can't tune without reflashing. Consider a `CMD_SET_GUARD_RAILS` command.

### FW-MED-5: Pose guess has no hysteresis

**File**: `reachy-sensor.ino` (`guessPose()`)
**Severity**: Medium

Simple threshold at 55 cm flips between SITTING and STANDING with no hysteresis. Person walking near the boundary will cause pose flickering.

### FW-LOW-1: `0x00` frame delimiter not a named constant

**File**: `reachy-sensor.ino`
**Severity**: Low

Magic literal `0x00` used across encode/decode. Should be `static const uint8_t FRAME_DELIMITER = 0x00;`.

### FW-LOW-2: No firmware unit tests

**File**: (none)
**Severity**: Low

State machine, COBS codec, and CRC could all be tested on-host with a simple harness. Currently relies entirely on Python-side protocol tests.

---

## Python Issues: Critical / Security

### PY-CRIT-1: `sys.exit()` in library functions

**Files**: `config.py:60-64` (module-level), `prompts.py:86,88,91` (`get_session_instructions()`)
**Severity**: Critical

Hard process termination during import/call. Prevents graceful error handling, makes testing impossible without subprocess isolation, violates separation of concerns. Should raise exceptions (`RuntimeError`, `ConfigError`) and let `main.py` handle shutdown.

### PY-CRIT-2: Path traversal in personality management

**Files**: `headless_personality.py:66`, `gradio_personality.py:64,188`
**Severity**: Critical (security)

Profile directories resolved from user-supplied names without containment checks:

```python
profile_dir = profiles_root / selection  # No validation
```

Crafted name like `../../../etc` escapes the profiles directory. Fix: validate `profile_dir.resolve().is_relative_to(profiles_root.resolve())`.

### PY-CRIT-3: Falsy-zero threshold bug in `light_context.py`

**File**: `light_context.py:143,249` (profile tool)
**Severity**: Critical

Pattern `value or DEFAULT` treats `0.0` as falsy, substituting the default. If any threshold is explicitly set to 0.0, the `or` operator ignores it. Affects all threshold comparisons. Fix: use `x if x is not None else default`.

---

## Python Issues: High Priority

### PY-HIGH-1: Unhandled `base64.b64decode()` in AudioRouter

**File**: `audio_router.py:41`
**Severity**: High

`np.frombuffer(base64.b64decode(delta), dtype=np.int16)` — no try/except. Malformed base64 delta from the WebSocket raises `binascii.Error` and crashes the audio loop.

### PY-HIGH-2: Global mutable state in tool registry without locking

**File**: `tools/core_tools.py:38-40`
**Severity**: High

`ALL_TOOLS`, `ALL_TOOL_SPECS`, `_TOOLS_INITIALIZED` are module-level globals with no synchronization. If tool initialization is triggered from multiple threads, state corruption is possible.

### PY-HIGH-3: Auto light_context dispatch has no independent timeout

**File**: `tool_dispatcher.py:397-411`
**Severity**: High

Light context auto-invocation runs inside a try/except that swallows all exceptions. If `run_from_mmwave()` hangs, it blocks the entire tool result path. Should apply `asyncio.wait_for()` independently.

### PY-HIGH-4: HeadWobbler generation check outside lock

**File**: `audio/head_wobbler.py:78-81`
**Severity**: High

Checks `_generation` before acquiring lock, then uses it inside. Classic TOCTOU race condition. Additionally, `reset()` queue draining (lines 156-165) can violate `Queue` contract if items are enqueued concurrently.

### PY-HIGH-5: Thread leak in `MovementManager.start()`

**File**: `moves.py:766-771`
**Severity**: High

Creates new thread without checking if previous thread is still alive. Calling `start()` multiple times without `stop()` leaks threads.

### PY-HIGH-6: Type annotation error

**File**: `main.py:39`
**Severity**: High

`robot: ReachyMini = None` should be `robot: ReachyMini | None = None`. Contradicts type hint under `mypy --strict`.

---

## Python Issues: Medium Priority

### PY-MED-1: GET with side effects in personality REST API

**File**: `headless_personality_ui.py:191-209`
**Severity**: Medium

`GET /personalities/save_raw` writes profile files. Violates HTTP semantics (GET must be safe and idempotent). Should be POST.

### PY-MED-2: Duplicate sample rate assignments

**File**: `openai_realtime.py:73-80`
**Severity**: Medium

Lines 73-74 set sample rate annotations, then lines 79-80 immediately overwrite them. Redundant and confusing.

### PY-MED-3: `last_activity_time` initialized before event loop

**File**: `openai_realtime.py:85`
**Severity**: Medium

`asyncio.get_event_loop().time()` called in `__init__`. If `__init__` runs before event loop is active, this raises or returns incorrect value. No lock protects concurrent reads/writes from idle policy.

### PY-MED-4: Float equality comparison for timestamps

**File**: `camera_worker.py:175`
**Severity**: Medium

`self.last_face_detected_time == current_time` compares float timestamps with `==`. Unreliable due to floating-point precision.

### PY-MED-5: Public `move_queue` accessible directly

**File**: `moves.py:307`
**Severity**: Medium

`move_queue` is a public `deque` attribute. External code can mutate it directly, bypassing the thread-safe command queue pattern.

### PY-MED-6: API key polling loop blocks with `time.sleep()`

**File**: `console.py:376-385`
**Severity**: Medium

Tight 0.2 s `time.sleep()` polling loop for API key availability. Should use event-based signaling instead.

### PY-MED-7: `launch()` method is overloaded

**File**: `console.py:322-385`
**Severity**: Medium

~63 lines doing env loading, HuggingFace download, settings UI init, key waiting, media start, route mounting, and async loop launch. Should decompose into smaller methods.

### PY-MED-8: Unbounded analytics JSONL growth in LightOrchestrator

**File**: `light_orchestrator.py`
**Severity**: Medium

Analytics JSONL file grows indefinitely. No rotation or size limit.

### PY-MED-9: mmWave glob patterns are macOS-first

**File**: `mmWave.py` (profile tool, `_resolve_serial_port()`)
**Severity**: Medium

Glob fallback tries `/dev/cu.usbmodem*` and `/dev/tty.usbmodem*` before Linux `/dev/ttyACM*`. Should prioritize the current platform.

### PY-MED-10: Inconsistent error handling across tools

**Severity**: Medium (cross-cutting)

Three patterns coexist: return error dict (mmWave), raise exception (`camera.py` raises `RuntimeError`), hard exit (`config.py`, `prompts.py`). Should standardize on error dicts for tool-level errors and exceptions for library-level errors.

---

## Python Issues: Low Priority

### PY-LOW-1: IdlePolicy constructor doesn't validate parameters

**File**: `idle_policy.py`
**Severity**: Low

Negative `sweep_cooldown_s` or zero `probe_interval_s` silently breaks scheduling logic.

### PY-LOW-2: `_safe_load_obj()` silently returns `{}` on parse errors

**File**: `tools/core_tools.py`
**Severity**: Low

Hides malformed tool call arguments.

### PY-LOW-3: Module pollution in `_load_module_from_file()`

**File**: `tools/core_tools.py`
**Severity**: Low

Adds module to `sys.modules` before `exec_module()`. If execution fails, a broken module reference persists.

### PY-LOW-4: sweep_look has no error handling

**File**: `sweep_look.py:29-35` (profile tool)
**Severity**: Low

No try/except around robot operations. Magic number `0.9 * np.pi` hardcoded; inconsistent with mmWave's `max_yaw = 0.8`.

### PY-LOW-5: Three save endpoints for personalities

**File**: `headless_personality_ui.py`
**Severity**: Low

`/save`, `/save_raw` POST, `/save_raw` GET — confusing API surface. Should consolidate.

---

## Test Suite Assessment

### Strengths

- 301 tests with comprehensive coverage across all handler classes, protocol, and configuration
- Excellent isolation via `conftest.py` (skip dotenv, clear env vars)
- Strong error-handling tests (corrupted JSON, permission errors, boundary conditions, version mismatches)
- Good mock strategy with `_EventConn` fake for WebSocket simulation
- `pytest-asyncio` well-integrated for async tests

### Coverage Gaps

| Area | Missing |
|------|---------|
| AudioRouter | Malformed base64 input, empty audio, oversized chunks |
| TranscriptHandler | Rapid concurrent `on_partial()` calls |
| ToolDispatcher | Light_context timeout, malformed JSON in sensor state |
| mmWave | Serial timeout/disconnection recovery, dropped frames in `_poll_events()` |
| sweep_look | Zero failure-path tests |
| Concurrency | No multi-threaded tests for `moves.py` or `head_wobbler.py` |
| light_context | Threshold set to `0.0` (falsy-zero bug) |
| Personality mgmt | Path traversal (security test) |

### Style

- Some tests use `asyncio.sleep(0.05)` for timing — flaky on slow CI
- Helper factory functions (e.g., `_policy()`, `_orchestrator()`) duplicate setup that should be pytest fixtures
- Some fixtures use `SimpleNamespace` where proper mocks or dataclasses would improve type safety

---

## Cross-Cutting Concerns

### CC-1: `or` vs `is not None` for defaults

Multiple files use `value or DEFAULT`, which treats `0`, `0.0`, `""`, `False` as missing. Confirmed as a bug in `light_context.py`. Should use `value if value is not None else DEFAULT` throughout.

### CC-2: Thread safety model underdocumented

`moves.py` has a clear threading contract in its module docstring. `head_wobbler.py` and `camera_worker.py` do not. Expected threading contracts should be explicit for every module with concurrent access.

### CC-3: Error handling inconsistency

Three patterns (error dicts, exceptions, `sys.exit()`) coexist. Standardize: error dicts for tools, exceptions for libraries, never `sys.exit()` outside `main()`.

### CC-4: Config singleton instantiated at import time

`config.py:167` creates `config = Config()` at module level. If `__init__` fails (bad env var, file missing), the entire import chain breaks. Consider lazy initialization or explicit `init()` call.
