# Non-blocking Tool Dispatch Design

## Problem

Tool dispatch in `ToolDispatcher.on_tool_call_done()` is awaited inline in the
`async for event in self.connection:` event loop (`openai_realtime.py:462`).
While a tool runs (mmWave: 5-23 s, camera vision: several seconds), the entire
event loop stalls — no audio frames, transcripts, errors, or cancellation are
processed.

The mmWave tool already offloads serial I/O to a thread via
`asyncio.to_thread()`, but the event loop still blocks waiting for the result.

## Requirements

- **Full audio continuity**: robot keeps listening, speaking, and processing
  events while a tool runs in the background.
- **Queue results for speech gap**: when a tool finishes, deliver the result
  to the conversation at the next natural pause (OpenAI Realtime API handles
  this natively — `function_call_output` + `response.create` are queued behind
  any in-flight response).
- **One tool at a time**: serialise concurrent tool calls via semaphore, but
  never block the event loop.
- **Timeout**: configurable per-dispatch timeout so a hanging tool cannot block
  the queue forever.

## Approach

**`asyncio.create_task()` + `Semaphore(1)` + `wait_for()` timeout.**

Chosen over a dedicated worker thread (over-engineered given tools already use
`to_thread` for I/O) and auto-`to_thread` wrapping (changes `Tool` base class
contract unnecessarily).

## Design

### ToolDispatcher changes (`tool_dispatcher.py`)

1. **New state:**
   - `_semaphore: asyncio.Semaphore` — initialised to 1 (one tool at a time)
   - `_active_task: asyncio.Task | None` — tracks the running task for
     cancellation
   - `_timeout_s: float` — configurable via `HEALTHY_TOOL_DISPATCH_TIMEOUT_S`
     (default 30 s)

2. **New public method `dispatch()`** — synchronous, non-blocking:
   - Validates inputs
   - Calls `asyncio.create_task(self._run_tool(...))` to fire-and-forget
   - Returns immediately

3. **Private `_run_tool()` coroutine** — the actual work:
   - `async with self._semaphore:` — waits for previous tool to finish
   - `self._active_task = asyncio.current_task()`
   - `await asyncio.wait_for(self._dispatch_tool(name, args), timeout=...)`
   - On `TimeoutError`: returns `{"error": "tool 'X' timed out after Ys"}`
   - Runs all post-dispatch logic (idle tracking, light context, send result,
     enqueue UI, create response, reset head wobbler)
   - `finally: self._active_task = None`

4. **New `cancel()` method** — for session teardown:
   - Cancels `_active_task` if running
   - Awaits it with `suppress(CancelledError)`

5. **Existing `on_tool_call_done()`** becomes `_run_tool()` (private), with
   the timeout wrapper added around the `_dispatch_tool` call.

### Event loop changes (`openai_realtime.py`)

1. In `response.function_call_arguments.done` handler:
   ```python
   # Before (blocking):
   consumed = await dispatcher.on_tool_call_done(...)
   if consumed:
       self.is_idle_tool_call = False

   # After (non-blocking):
   dispatcher.dispatch(
       tool_name=tool_name,
       args_json=args_json_str,
       call_id=call_id,
       is_idle=self.is_idle_tool_call,
   )
   if self.is_idle_tool_call:
       self.is_idle_tool_call = False
   ```

2. In `shutdown()`: call `await dispatcher.cancel()` before closing connection.

3. Pass `timeout_s` when constructing `ToolDispatcher`.

### Idle flag handling

The `is_idle` boolean is captured as a parameter at dispatch time and passed
into `_run_tool()`. The event loop clears `is_idle_tool_call` immediately
(fire-and-forget semantics). No race — the flag is consumed synchronously
before the next event iteration.

### Error handling

- **Timeout**: tool result becomes `{"error": "tool 'X' timed out after Ys"}`,
  sent back to OpenAI as a normal `function_call_output`.
- **Cancellation** (session close): task is cancelled, no result sent (session
  is gone anyway).
- **Tool exception**: already handled by existing try/except in
  `on_tool_call_done` — unchanged.
- **Semaphore starvation**: if a tool hangs past timeout, the semaphore is
  released after timeout fires, unblocking the next queued tool.

### Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` | `30` | Max seconds per tool call |

## Files changed

| File | Change |
|------|--------|
| `tool_dispatcher.py` | Add semaphore, timeout, `dispatch()`, `cancel()`, rename `on_tool_call_done` to `_run_tool` |
| `openai_realtime.py` | Non-awaiting `dispatch()` call, `cancel()` on shutdown, pass timeout_s |
| `tests/test_tool_dispatcher.py` | Update tests for new dispatch/cancel API, add timeout test |

No changes to `core_tools.py`, `Tool` base class, or individual tool implementations.

## Not in scope

- Concurrent tool execution (future: raise semaphore limit)
- Auto-`to_thread` wrapping for tools that block (future: enforce in `Tool` base)
- Explicit speech-gap detection (OpenAI API queues responses natively)
