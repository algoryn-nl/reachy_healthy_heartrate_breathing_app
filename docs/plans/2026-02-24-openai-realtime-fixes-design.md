# Design: openai_realtime.py Exception Handling Fixes

## Context

Three remaining high/medium priority issues in `openai_realtime.py`:

1. **Session setup exception handling gap** (High) — `self.connection` not cleaned up when `_run_realtime_session()` fails outside `start_up()`
2. **Idle tool call flag not reset on exception** (Medium) — `is_idle_tool_call` stays `True` after `send_idle_signal()` raises
3. **Test coverage for bug paths** (High) — targeted tests for the two fixes above

## Fix 1: Connection cleanup in `_run_realtime_session()`

### Problem

When `_run_realtime_session()` is called from `_restart_session()` via `asyncio.create_task()`, there is no wrapping `finally` to clear `self.connection`. If the event loop crashes after `self.connection = conn` is assigned (line 416), the `async with` closes the underlying WebSocket, but `self.connection` remains pointing to the closed connection object. Other coroutines (`receive()`, `emit()`) may then attempt operations on a dead connection.

The `start_up()` method has its own `finally` block (line 264) that clears `self.connection`, but `_restart_session()` does not use that path.

### Fix

Wrap the event-loop section of `_run_realtime_session()` in `try/finally`:

```python
self.connection = conn
try:
    self._connected_event.set()
    async for event in self.connection:
        ...
finally:
    self.connection = None
    self._connected_event.clear()
```

The double-clear from `start_up()`'s `finally` is harmless (setting `None` to `None`).

### Files

- `src/healthy_heartrate_breathing/openai_realtime.py` — lines 415-479

## Fix 2: Reset idle flag on exception path

### Problem

`send_idle_signal()` sets `self.is_idle_tool_call = True` on entry (line 657). If the method then raises (e.g., connection closed), the exception is caught in `emit()` (line 539-540) but the flag is never reset. The next real user-initiated tool call is incorrectly tagged as idle, causing `ToolDispatcher` to apply idle-mode arg overrides.

### Fix

Reset the flag in the exception handler in `emit()`:

```python
except Exception as e:
    self.is_idle_tool_call = False
    logger.warning(...)
    return None
```

### Files

- `src/healthy_heartrate_breathing/openai_realtime.py` — line 539

## Fix 3: Targeted tests

New tests in `tests/test_openai_realtime.py` covering both bug paths:

| Test | Validates |
|------|-----------|
| `test_idle_flag_reset_on_send_idle_signal_exception` | Flag reset when `send_idle_signal` raises |
| `test_idle_flag_set_and_cleared_on_success` | Normal idle signal cycle |
| `test_connection_cleared_on_event_loop_exception` | `self.connection` is `None` after event loop crash |
| `test_connection_cleared_on_restart_failure` | Cleanup after `_restart_session()` failure |
| `test_emit_returns_none_when_idle_signal_fails` | `emit()` returns `None` on idle signal failure |
| `test_receive_guards_closed_connection` | `receive()` silently drops frames when disconnected |

Tests reuse the existing `FakeConn`/`FakeClient` pattern from `test_start_up_retries_on_abrupt_close`.
