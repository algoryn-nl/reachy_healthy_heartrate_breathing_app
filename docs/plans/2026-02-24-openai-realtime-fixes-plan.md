# openai_realtime.py Exception Handling Fixes — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Fix two exception-handling bugs in `openai_realtime.py` and add targeted regression tests.

**Architecture:** Two surgical edits — a `try/finally` wrapper around the event loop, and an idle-flag reset in the `except` branch of `emit()`. Tests use the existing `FakeConn`/`FakeClient` stub pattern from the test file.

**Tech Stack:** Python 3.12, pytest, pytest-asyncio, unittest.mock

---

### Task 1: Fix idle flag reset on exception path

The simpler fix. One line addition.

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py:539`

**Step 1: Write the failing test**

Add to `tests/test_openai_realtime.py`:

```python
@pytest.mark.asyncio
async def test_idle_flag_reset_on_send_idle_signal_exception(monkeypatch: Any) -> None:
    """is_idle_tool_call must be reset when send_idle_signal raises."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)

    # Force idle conditions: long idle duration, not moving
    handler.last_activity_time = 0.0
    handler.start_time = 0.0
    deps.movement_manager.is_idle.return_value = True

    # Make send_idle_signal raise
    async def _raise(*_a: Any, **_kw: Any) -> None:
        raise ConnectionError("simulated disconnect")

    monkeypatch.setattr(handler, "send_idle_signal", _raise)

    # Stub _has_tool to return False (no mmWave -> uses simple idle_interval)
    monkeypatch.setattr(handler, "_has_tool", lambda _name: False)

    # Stub wait_for_item so it doesn't block
    monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

    result = await handler.emit()

    assert result is None
    assert handler.is_idle_tool_call is False, "Flag must be reset after exception"
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_openai_realtime.py::test_idle_flag_reset_on_send_idle_signal_exception -v`
Expected: FAIL — `handler.is_idle_tool_call` is `True` because the flag is never reset.

**Step 3: Apply the fix**

In `src/healthy_heartrate_breathing/openai_realtime.py`, find the exception handler in `emit()` (line 539):

```python
# BEFORE (lines 538-541):
            except Exception as e:
                logger.warning("Idle signal skipped (connection closed?): %s", e)
                return None

# AFTER:
            except Exception as e:
                self.is_idle_tool_call = False
                logger.warning("Idle signal skipped (connection closed?): %s", e)
                return None
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_openai_realtime.py::test_idle_flag_reset_on_send_idle_signal_exception -v`
Expected: PASS

**Step 5: Add the complementary success-path test**

```python
@pytest.mark.asyncio
async def test_emit_returns_none_when_idle_signal_fails(monkeypatch: Any) -> None:
    """emit() returns None (not crash) when send_idle_signal fails."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)

    handler.last_activity_time = 0.0
    deps.movement_manager.is_idle.return_value = True
    monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
    monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

    async def _raise(*_a: Any, **_kw: Any) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(handler, "send_idle_signal", _raise)

    result = await handler.emit()
    assert result is None
```

**Step 6: Run all tests in this file**

Run: `pytest tests/test_openai_realtime.py -v`
Expected: All pass

**Step 7: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py tests/test_openai_realtime.py
git commit -m "fix: reset is_idle_tool_call flag on exception path in emit()"
```

---

### Task 2: Fix connection cleanup in `_run_realtime_session()`

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py:415-479`

**Step 1: Write the failing test**

Add to `tests/test_openai_realtime.py`. This test simulates an event loop crash and verifies `self.connection` is cleaned up — exercising the path used by `_restart_session()`.

```python
@pytest.mark.asyncio
async def test_connection_cleared_on_event_loop_exception(monkeypatch: Any) -> None:
    """self.connection must be None after event loop raises."""
    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    class CrashConn:
        """Connection that crashes during event iteration."""

        def __init__(self) -> None:
            class _Session:
                async def update(self, **_kw: Any) -> None:
                    return None

            self.session = _Session()

            class _Item:
                async def create(self, **_kw: Any) -> None:
                    return None

            class _Conversation:
                item = _Item()

            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None:
                    return None

            self.response = _Response()

        async def __aenter__(self) -> "CrashConn":
            return self

        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
            return False

        async def close(self) -> None:
            return None

        def __aiter__(self) -> "CrashConn":
            return self

        async def __anext__(self) -> None:
            raise RuntimeError("event loop crash")

    class FakeRealtime:
        def connect(self, **_kw: Any) -> CrashConn:
            return CrashConn()

    class FakeClient:
        def __init__(self, **_kw: Any) -> None:
            self.realtime = FakeRealtime()

    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)
    handler.client = FakeClient()

    # Call _run_realtime_session directly (as _restart_session does via create_task)
    with pytest.raises(RuntimeError, match="event loop crash"):
        await handler._run_realtime_session()

    assert handler.connection is None, "connection must be cleared after event loop crash"
    assert not handler._connected_event.is_set(), "connected event must be cleared"
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_openai_realtime.py::test_connection_cleared_on_event_loop_exception -v`
Expected: FAIL — `handler.connection` still points to the closed `CrashConn` object.

**Step 3: Apply the fix**

In `src/healthy_heartrate_breathing/openai_realtime.py`, restructure lines 415-479:

```python
# BEFORE (lines 415-479):
            # Event loop
            self.connection = conn
            try:
                self._connected_event.set()
            except Exception:
                logger.debug("Failed to set connected event after session init", exc_info=True)

            async for event in self.connection:
                ...event handling...

# AFTER:
            # Event loop
            self.connection = conn
            try:
                self._connected_event.set()

                async for event in self.connection:
                    ...event handling (unchanged)...
            finally:
                self.connection = None
                self._connected_event.clear()
```

The key change: the `try` block that wraps `_connected_event.set()` is extended to also cover the `async for` loop, and a `finally` clause is added to ensure `self.connection` and `_connected_event` are always cleaned up.

The bare `except Exception` previously swallowing `_connected_event.set()` failures is removed — if `set()` on an `asyncio.Event` raises, something is critically wrong and we should not proceed with the event loop.

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_openai_realtime.py::test_connection_cleared_on_event_loop_exception -v`
Expected: PASS

**Step 5: Add restart-path regression test**

```python
@pytest.mark.asyncio
async def test_connection_cleared_on_restart_failure(monkeypatch: Any) -> None:
    """_restart_session cleans up even when new session fails during event loop."""
    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    class FailConn:
        def __init__(self) -> None:
            class _Session:
                async def update(self, **_kw: Any) -> None:
                    return None

            self.session = _Session()

            class _Item:
                async def create(self, **_kw: Any) -> None:
                    return None

            class _Conversation:
                item = _Item()

            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None:
                    return None

            self.response = _Response()

        async def __aenter__(self) -> "FailConn":
            return self

        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
            return False

        async def close(self) -> None:
            return None

        def __aiter__(self) -> "FailConn":
            return self

        async def __anext__(self) -> None:
            raise RuntimeError("restart crash")

    class FakeRealtime:
        def connect(self, **_kw: Any) -> FailConn:
            return FailConn()

    class FakeClient:
        def __init__(self, **_kw: Any) -> None:
            self.realtime = FakeRealtime()

    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)
    handler.client = FakeClient()

    await handler._restart_session()
    # Give the background task time to run and fail
    await asyncio.sleep(0.1)

    assert handler.connection is None, "connection must be cleaned up after restart failure"
```

**Step 6: Run all tests in this file**

Run: `pytest tests/test_openai_realtime.py -v`
Expected: All pass

**Step 7: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py tests/test_openai_realtime.py
git commit -m "fix: ensure connection cleanup in _run_realtime_session() finally block"
```

---

### Task 3: Add receive guard test

**Files:**
- Test: `tests/test_openai_realtime.py`

**Step 1: Write the test**

```python
@pytest.mark.asyncio
async def test_receive_guards_closed_connection() -> None:
    """receive() silently drops frames when connection is None."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)
    handler.connection = None

    # Should not raise
    frame = (24000, np.zeros(480, dtype=np.int16))
    await handler.receive(frame)
    # No assertion needed — just verifying no exception
```

Note: add `import numpy as np` to the test file imports if not already present.

**Step 2: Run test**

Run: `pytest tests/test_openai_realtime.py::test_receive_guards_closed_connection -v`
Expected: PASS (this is a pre-existing guard at line 493-494, test documents it)

**Step 3: Commit**

```bash
git add tests/test_openai_realtime.py
git commit -m "test: add receive() connection guard regression test"
```

---

### Task 4: Update docs and final verification

**Files:**
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`

**Step 1: Update docs/TODO.md**

Move these items from Upcoming to Done:
- `Fix session setup exception handling gap` → Done with date
- `Reset idle tool call flag on exception path` → Done with date

**Step 2: Update docs/20260223_roadmap.md**

Strike through:
- "Session setup exception returns early but connection object may already be set"
- "Idle tool call flag not reset on exception path"

**Step 3: Run full test suite**

Run: `pytest tests/test_openai_realtime.py tests/test_env_utils.py tests/test_idle_policy.py tests/test_transcript_handler.py tests/test_audio_router.py tests/test_tool_dispatcher.py tests/test_light_orchestrator.py -v`
Expected: All pass

**Step 4: Lint**

Run: `ruff check src/healthy_heartrate_breathing/openai_realtime.py tests/test_openai_realtime.py`
Expected: Clean

**Step 5: Commit docs**

```bash
git add docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: mark session exception and idle flag fixes as done"
```

**Step 6: Push**

```bash
git push
```
