# Non-blocking Tool Dispatch Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make tool dispatch non-blocking so the event loop continues processing audio, transcripts, and errors while tools run in the background.

**Architecture:** Replace the inline `await dispatcher.on_tool_call_done(...)` with a fire-and-forget `dispatcher.dispatch(...)` backed by `asyncio.create_task()`. A `Semaphore(1)` serialises tool execution, and `asyncio.wait_for()` adds a configurable timeout. A `cancel()` method handles session teardown.

**Tech Stack:** Python asyncio (`create_task`, `Semaphore`, `wait_for`), pytest + pytest-asyncio

**Design doc:** `docs/plans/2026-02-23-nonblocking-tool-dispatch-design.md`

---

### Task 1: Add timeout test for tool dispatch

**Files:**
- Modify: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing test**

Add a new test class at the end of `tests/test_tool_dispatcher.py`:

```python
import asyncio


class TestTimeout:
    @pytest.mark.asyncio
    async def test_tool_timeout_returns_error(self, tmp_path) -> None:
        """A tool that exceeds timeout_s produces an error result."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(10)
            return {"status": "ok"}

        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=slow_tool, send_tool_result=send_result, timeout_s=0.1)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-t", is_idle=False)
        await asyncio.sleep(0.3)  # let the task complete

        call_args = send_result.call_args
        result = json.loads(call_args[0][1])
        assert "error" in result
        assert "timed out" in result["error"]
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_tool_dispatcher.py::TestTimeout::test_tool_timeout_returns_error -v`

Expected: FAIL — `_dispatcher` does not accept `timeout_s`, and `ToolDispatcher` has no `dispatch()` method yet.

**Step 3: Commit**

```bash
git add tests/test_tool_dispatcher.py
git commit -m "test: add failing timeout test for tool dispatch"
```

---

### Task 2: Add cancel test

**Files:**
- Modify: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing test**

Add to `TestTimeout` class:

```python
    @pytest.mark.asyncio
    async def test_cancel_stops_running_tool(self, tmp_path) -> None:
        """cancel() terminates an in-flight tool task."""
        started = asyncio.Event()

        async def blocking_tool(name: str, args: str) -> dict:
            started.set()
            await asyncio.sleep(60)
            return {"status": "ok"}

        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=blocking_tool, send_tool_result=send_result, timeout_s=60)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-c", is_idle=False)
        await started.wait()

        await d.cancel()

        assert d._active_task is None
        send_result.assert_not_called()
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_tool_dispatcher.py::TestTimeout::test_cancel_stops_running_tool -v`

Expected: FAIL — same reason as Task 1.

**Step 3: Commit**

```bash
git add tests/test_tool_dispatcher.py
git commit -m "test: add failing cancel test for tool dispatch"
```

---

### Task 3: Add serialisation (semaphore) test

**Files:**
- Modify: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing test**

Add to `TestTimeout` class (or create `TestSerialization`):

```python
class TestSerialization:
    @pytest.mark.asyncio
    async def test_tools_run_one_at_a_time(self, tmp_path) -> None:
        """Semaphore ensures only one tool runs at a time."""
        concurrency = 0
        max_concurrency = 0

        async def tracking_tool(name: str, args: str) -> dict:
            nonlocal concurrency, max_concurrency
            concurrency += 1
            max_concurrency = max(max_concurrency, concurrency)
            await asyncio.sleep(0.05)
            concurrency -= 1
            return {"status": "ok"}

        d = _dispatcher(tmp_path, dispatch_tool=tracking_tool, timeout_s=5)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-2", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-3", is_idle=False)
        await asyncio.sleep(0.3)  # let all 3 finish sequentially

        assert max_concurrency == 1
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_tool_dispatcher.py::TestSerialization::test_tools_run_one_at_a_time -v`

Expected: FAIL.

**Step 3: Commit**

```bash
git add tests/test_tool_dispatcher.py
git commit -m "test: add failing serialisation test for tool dispatch"
```

---

### Task 4: Add non-blocking event loop test

**Files:**
- Modify: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing test**

Add new class:

```python
class TestNonBlocking:
    @pytest.mark.asyncio
    async def test_dispatch_returns_immediately(self, tmp_path) -> None:
        """dispatch() must not block — returns before tool completes."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(5)
            return {"status": "ok"}

        d = _dispatcher(tmp_path, dispatch_tool=slow_tool, timeout_s=10)

        import time

        t0 = time.monotonic()
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-nb", is_idle=False)
        elapsed = time.monotonic() - t0

        assert elapsed < 0.05, f"dispatch() blocked for {elapsed:.3f}s"
        await d.cancel()  # cleanup
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_tool_dispatcher.py::TestNonBlocking::test_dispatch_returns_immediately -v`

Expected: FAIL.

**Step 3: Commit**

```bash
git add tests/test_tool_dispatcher.py
git commit -m "test: add failing non-blocking dispatch test"
```

---

### Task 5: Implement non-blocking dispatch in ToolDispatcher

**Files:**
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py`

**Step 1: Add `timeout_s` parameter to `__init__`**

In `ToolDispatcher.__init__`, add after `head_wobbler_reset` parameter:

```python
        timeout_s: float = 30.0,
```

And in the body, add after `self._head_wobbler_reset = head_wobbler_reset`:

```python
        self._timeout_s = timeout_s
        self._semaphore = asyncio.Semaphore(1)
        self._active_task: asyncio.Task[None] | None = None
```

**Step 2: Rename `on_tool_call_done` to `_run_tool`**

Rename the method from `on_tool_call_done` to `_run_tool`. Keep the same signature and body — we will wrap it in the next step.

**Step 3: Add timeout wrapper inside `_run_tool`**

Replace the `try/except` block around `self._dispatch_tool` (lines 153-159 in current code) with:

```python
            try:
                tool_result = await asyncio.wait_for(
                    self._dispatch_tool(tool_name, effective_args_json),
                    timeout=self._timeout_s,
                )
                logger.debug("Tool '%s' executed successfully", tool_name)
                logger.debug("Tool result: %s", tool_result)
            except asyncio.TimeoutError:
                logger.error("Tool '%s' timed out after %.1fs", tool_name, self._timeout_s)
                tool_result = {"error": f"tool '{tool_name}' timed out after {self._timeout_s:.0f}s"}
            except asyncio.CancelledError:
                logger.info("Tool '%s' cancelled", tool_name)
                return
            except Exception as e:
                logger.error("Tool '%s' failed", tool_name)
                tool_result = {"error": str(e)}
```

**Step 4: Wrap `_run_tool` with semaphore and task tracking**

Add a private wrapper method `_guarded_run` that handles the semaphore and task tracking, then calls `_run_tool`:

```python
    async def _guarded_run(
        self,
        *,
        tool_name: str,
        args_json: str,
        call_id: str | None,
        is_idle: bool,
    ) -> None:
        """Acquire semaphore, track active task, then run tool."""
        try:
            async with self._semaphore:
                self._active_task = asyncio.current_task()
                try:
                    await self._run_tool(
                        tool_name=tool_name,
                        args_json=args_json,
                        call_id=call_id,
                        is_idle=is_idle,
                    )
                finally:
                    self._active_task = None
        except asyncio.CancelledError:
            logger.info("Tool dispatch cancelled for '%s'", tool_name)
        except Exception:
            logger.exception("Unhandled error in tool dispatch for '%s'", tool_name)
```

**Step 5: Add `dispatch()` public method**

```python
    def dispatch(
        self,
        *,
        tool_name: str,
        args_json: str,
        call_id: str | None,
        is_idle: bool,
    ) -> None:
        """Fire-and-forget tool dispatch. Returns immediately."""
        logger.info("Dispatching tool (non-blocking): %s args=%s", tool_name, _short_text(args_json))
        asyncio.create_task(
            self._guarded_run(
                tool_name=tool_name,
                args_json=args_json,
                call_id=call_id,
                is_idle=is_idle,
            ),
            name=f"tool-dispatch-{tool_name}",
        )
```

**Step 6: Add `cancel()` method**

```python
    async def cancel(self) -> None:
        """Cancel any in-flight tool task (for session teardown)."""
        task = self._active_task
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._active_task = None
```

**Step 7: Remove return value from `_run_tool`**

The old `on_tool_call_done` returned `bool` (the idle consumed flag). This is no longer needed since `dispatch()` is fire-and-forget and the idle flag is consumed at the call site. Change `_run_tool`'s return type to `None` and remove the final `return is_idle` line.

**Step 8: Run tests to verify new tests pass**

Run: `pytest tests/test_tool_dispatcher.py -v`

Expected: The 4 new tests pass. The 8 existing tests **FAIL** because they still call `d.on_tool_call_done(...)` which no longer exists.

**Step 9: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py
git commit -m "feat: add non-blocking dispatch with semaphore, timeout, and cancel"
```

---

### Task 6: Migrate existing tests to use `dispatch()`

**Files:**
- Modify: `tests/test_tool_dispatcher.py`

**Step 1: Update `_dispatcher` helper to accept `timeout_s`**

In the `_dispatcher()` helper, add `timeout_s` handling:

```python
def _dispatcher(tmp_path, **overrides: object) -> ToolDispatcher:
    defaults = {
        "idle_policy": _idle_policy(),
        "light_orchestrator": _light_orchestrator(tmp_path),
        "has_tool": lambda name: name == "mmWave",
        "dispatch_tool": AsyncMock(return_value={"status": "ok"}),
        "send_tool_result": AsyncMock(),
        "create_response": AsyncMock(),
        "create_message": AsyncMock(),
        "enqueue_output": AsyncMock(),
        "get_camera_frame": lambda: None,
        "head_wobbler_reset": None,
        "timeout_s": 5.0,
    }
    defaults.update(overrides)
    return ToolDispatcher(**defaults)
```

**Step 2: Add a `_dispatch_and_wait` async helper**

Many existing tests need to dispatch and wait for the result. Add a helper after `_dispatcher`:

```python
async def _dispatch_and_wait(d: ToolDispatcher, *, tool_name: str, args_json: str, call_id: str | None, is_idle: bool) -> None:
    """Dispatch a tool and wait for it to complete."""
    d.dispatch(tool_name=tool_name, args_json=args_json, call_id=call_id, is_idle=is_idle)
    await asyncio.sleep(0.05)  # let the task run
```

**Step 3: Replace all `await d.on_tool_call_done(...)` calls**

In each existing test, replace:
```python
consumed = await d.on_tool_call_done(
    tool_name="mmWave",
    args_json='{"mode":"scan"}',
    call_id="call-1",
    is_idle=False,
)
```
with:
```python
await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=False)
```

Drop all `consumed` variable usage and `assert consumed is True/False` checks — the idle consumed flag is now handled at the event-loop level, not by the dispatcher.

For `test_idle_consumed_flag`: remove this test entirely — the concept of "consumed flag return value" no longer exists. The idle flag is consumed synchronously at the call site in `openai_realtime.py`.

**Step 4: Run all tests**

Run: `pytest tests/test_tool_dispatcher.py -v`

Expected: All tests pass (8 migrated + 4 new = ~11 tests after removing `test_idle_consumed_flag`).

**Step 5: Commit**

```bash
git add tests/test_tool_dispatcher.py
git commit -m "test: migrate existing dispatcher tests to non-blocking dispatch API"
```

---

### Task 7: Update event loop in openai_realtime.py

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py`

**Step 1: Add timeout_s to ToolDispatcher construction**

At the top of the file, add `env_float` to the import from `env_utils` (already imported). In `_run_realtime_session()`, update the `ToolDispatcher` constructor call (currently around line 378) to add:

```python
            dispatcher = ToolDispatcher(
                idle_policy=self.idle_policy,
                light_orchestrator=self.light_orchestrator,
                has_tool=self._has_tool,
                dispatch_tool=lambda name, args: dispatch_tool_call(name, args, self.deps),
                send_tool_result=_send_tool_result,
                create_response=_create_response,
                create_message=_create_message,
                enqueue_output=_enqueue_output,
                get_camera_frame=lambda: (
                    self.deps.camera_worker.get_latest_frame()
                    if self.deps.camera_worker is not None
                    else None
                ),
                head_wobbler_reset=(
                    self.deps.head_wobbler.reset
                    if self.deps.head_wobbler is not None
                    else None
                ),
                timeout_s=env_float("HEALTHY_TOOL_DISPATCH_TIMEOUT_S", 30.0, min_value=1.0),
            )
```

**Step 2: Replace blocking dispatch with fire-and-forget**

Replace the `response.function_call_arguments.done` handler (currently lines 455-469):

Before:
```python
                elif event.type == "response.function_call_arguments.done":
                    tool_name = getattr(event, "name", None)
                    args_json_str = getattr(event, "arguments", None)
                    call_id = getattr(event, "call_id", None)
                    if not isinstance(tool_name, str) or not isinstance(args_json_str, str):
                        logger.error("Invalid tool call: tool_name=%s, args=%s", tool_name, args_json_str)
                        continue
                    consumed = await dispatcher.on_tool_call_done(
                        tool_name=tool_name,
                        args_json=args_json_str,
                        call_id=call_id,
                        is_idle=self.is_idle_tool_call,
                    )
                    if consumed:
                        self.is_idle_tool_call = False
```

After:
```python
                elif event.type == "response.function_call_arguments.done":
                    tool_name = getattr(event, "name", None)
                    args_json_str = getattr(event, "arguments", None)
                    call_id = getattr(event, "call_id", None)
                    if not isinstance(tool_name, str) or not isinstance(args_json_str, str):
                        logger.error("Invalid tool call: tool_name=%s, args=%s", tool_name, args_json_str)
                        continue
                    dispatcher.dispatch(
                        tool_name=tool_name,
                        args_json=args_json_str,
                        call_id=call_id,
                        is_idle=self.is_idle_tool_call,
                    )
                    if self.is_idle_tool_call:
                        self.is_idle_tool_call = False
```

**Step 3: Add cancel on shutdown**

In `OpenaiRealtimeHandler.shutdown()` (line 545), add `await dispatcher.cancel()` before closing the connection. However, `dispatcher` is a local variable in `_run_realtime_session()` — it's not accessible from `shutdown()`.

Fix: store dispatcher as `self._dispatcher` after construction:

```python
            self._dispatcher = dispatcher
```

Then in `shutdown()`, add before the connection close:

```python
        if hasattr(self, "_dispatcher"):
            await self._dispatcher.cancel()
```

**Step 4: Run full test suite**

Run: `pytest tests/ --ignore=tests/vision -v`

Expected: All tests pass.

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "feat: non-blocking tool dispatch in event loop"
```

---

### Task 8: Update documentation

**Files:**
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`

**Step 1: Update TODO.md**

Move the tool dispatch timeout task from Upcoming High Priority to Done:

```
- [x] Non-blocking tool dispatch with timeout and cancellation (2026-02-23)
```

Remove:
```
- [ ] Add `asyncio.wait_for()` timeout to tool dispatch (openai_realtime.py:567)
```

**Step 2: Update roadmap**

In `docs/20260223_roadmap.md`, move the tool dispatch item from High Priority Known Issues to a completed note, and update the Robustness section:

Remove from High Priority:
```
- No timeout on tool dispatch — a hanging tool blocks the entire event loop (tool_dispatcher.py)
```

Update Robustness section to note completion:
```
- Tool dispatch timeout and non-blocking execution via asyncio.create_task() + Semaphore(1) + wait_for() (complete)
```

**Step 3: Commit**

```bash
git add docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: mark non-blocking tool dispatch as complete"
```

---

### Task 9: Run full test suite and verify

**Files:** (none — verification only)

**Step 1: Run full test suite**

Run: `pytest tests/ --ignore=tests/vision -v`

Expected: All tests pass, no regressions.

**Step 2: Run lint**

Run: `ruff check src/healthy_heartrate_breathing/tool_dispatcher.py src/healthy_heartrate_breathing/openai_realtime.py tests/test_tool_dispatcher.py`

Expected: No errors.

**Step 3: Run type check**

Run: `mypy src/`

Expected: No new errors.
