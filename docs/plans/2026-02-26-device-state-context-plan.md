# Device State Context Integration — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Enrich mmWave tool results with a `device_context` dict so the LLM can qualify vitals reliability, acknowledge state transitions, and adapt behavior based on person state.

**Architecture:** A pure function `build_device_context()` in `tool_dispatcher.py` maps device state + previous state to a structured context dict. ToolDispatcher tracks `previous_device_state` across calls and injects `device_context` into the tool result. System prompt extended with interpretation guidance.

**Tech Stack:** Python, pytest, pytest-asyncio

---

### Task 1: `build_device_context()` pure function + unit tests

**Files:**
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py` (after line 143, before `class ToolDispatcher`)
- Test: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing tests**

Add to `tests/test_tool_dispatcher.py`. Import `build_device_context` at the top alongside the existing imports:

```python
from healthy_heartrate_breathing.tool_dispatcher import ToolDispatcher, build_device_context
```

Add these test classes at the end of the file:

```python
class TestBuildDeviceContext:
    def test_resting_vitals_returns_high_reliability(self) -> None:
        state = {"device_state": "RESTING_VITALS", "previous_device_state": "RESTING_VITALS"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["state"] == "RESTING_VITALS"
        assert ctx["vitals_reliability"] == "high"
        assert ctx["changed"] is False
        assert ctx["transition"] is None

    def test_still_near_returns_moderate_reliability(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": "STILL_NEAR"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "moderate"

    def test_moving_returns_low_reliability(self) -> None:
        state = {"device_state": "MOVING", "previous_device_state": "MOVING"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "low"

    def test_present_far_returns_unavailable(self) -> None:
        state = {"device_state": "PRESENT_FAR", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_multi_target_returns_unavailable(self) -> None:
        state = {"device_state": "MULTI_TARGET", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_no_target_returns_unavailable(self) -> None:
        state = {"device_state": "NO_TARGET", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_unknown_state_returns_unknown_reliability(self) -> None:
        state = {"device_state": "UNKNOWN_7", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unknown"
        assert "UNKNOWN_7" in ctx["note"]

    def test_missing_device_state_returns_none(self) -> None:
        state = {"previous_device_state": None}
        assert build_device_context(state) is None

    def test_none_device_state_returns_none(self) -> None:
        state = {"device_state": None, "previous_device_state": None}
        assert build_device_context(state) is None

    def test_changed_true_when_state_differs(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": "MOVING"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert ctx["previous_state"] == "MOVING"
        assert ctx["transition"] is not None
        assert "MOVING" in ctx["transition"]
        assert "STILL_NEAR" in ctx["transition"]

    def test_first_call_previous_none_changed_false(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is False
        assert ctx["previous_state"] is None
        assert ctx["transition"] is None

    def test_transition_no_target_to_present(self) -> None:
        state = {"device_state": "PRESENT_FAR", "previous_device_state": "NO_TARGET"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert "arrived" in ctx["transition"].lower() or "just" in ctx["transition"].lower()

    def test_transition_to_no_target(self) -> None:
        state = {"device_state": "NO_TARGET", "previous_device_state": "STILL_NEAR"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert "left" in ctx["transition"].lower()
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_tool_dispatcher.py::TestBuildDeviceContext -v`
Expected: FAIL — `ImportError: cannot import name 'build_device_context'`

**Step 3: Write the implementation**

Add to `src/healthy_heartrate_breathing/tool_dispatcher.py` after line 143 (after `_mmwave_is_multi_target`), before `class ToolDispatcher`:

```python
# -- Device state context enrichment ------------------------------------------

_STATE_INFO: dict[str, tuple[str, str]] = {
    "RESTING_VITALS": (
        "high",
        "Person is still and close — ideal for heart rate and breathing measurement.",
    ),
    "STILL_NEAR": (
        "moderate",
        "Person is close and settling. Wait for RESTING_VITALS for most reliable readings.",
    ),
    "MOVING": (
        "low",
        "Person is moving — vitals readings may be inaccurate.",
    ),
    "PRESENT_FAR": (
        "unavailable",
        "Person detected but too far for vitals measurement.",
    ),
    "MULTI_TARGET": (
        "unavailable",
        "Multiple people detected — cannot isolate vitals.",
    ),
    "NO_TARGET": (
        "unavailable",
        "No person detected.",
    ),
}

_TRANSITION_SUFFIXES: dict[str, str] = {
    "RESTING_VITALS": "now in ideal position for vitals.",
    "STILL_NEAR": "vitals measurement should improve soon.",
    "MOVING": "movement may affect vitals accuracy.",
    "PRESENT_FAR": "person moved out of vitals range.",
    "MULTI_TARGET": "multiple people now detected.",
    "NO_TARGET": "person left the area.",
}


def build_device_context(sensor_state: dict[str, Any]) -> dict[str, Any] | None:
    """Build a device-context dict from sensor state for LLM consumption.

    Returns ``None`` when no device state is available (error/disconnect).
    """
    current = sensor_state.get("device_state")
    if current is None:
        return None

    previous: str | None = sensor_state.get("previous_device_state")
    changed = previous is not None and previous != current

    info = _STATE_INFO.get(current)
    if info is not None:
        reliability, note = info
    else:
        reliability = "unknown"
        note = f"Unrecognised device state: {current}."

    transition: str | None = None
    if changed:
        suffix = _TRANSITION_SUFFIXES.get(current, f"state is now {current}.")
        # Special-case: arrival
        if previous == "NO_TARGET" and current != "NO_TARGET":
            transition = f"{previous} → {current} — someone just arrived; {suffix}"
        else:
            transition = f"{previous} → {current} — {suffix}"

    return {
        "state": current,
        "previous_state": previous,
        "changed": changed,
        "vitals_reliability": reliability,
        "transition": transition,
        "note": note,
    }
```

**Step 4: Run tests to verify they pass**

Run: `pytest tests/test_tool_dispatcher.py::TestBuildDeviceContext -v`
Expected: 13 PASSED

**Step 5: Lint**

Run: `ruff check src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py`
Expected: clean

**Step 6: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py
git commit -m "feat: add build_device_context() pure function with 13 tests"
```

---

### Task 2: Inject `device_context` into mmWave tool results via ToolDispatcher

**Files:**
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py` (lines 336-342, the sensor state update block)
- Test: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing tests**

Add to `tests/test_tool_dispatcher.py`:

```python
class TestDeviceContextInjection:
    @pytest.mark.asyncio
    async def test_device_context_injected_into_mmwave_result(self, tmp_path) -> None:
        """mmWave result should have device_context at top level after dispatch."""
        result = {
            "scan": {"device_state": "RESTING_VITALS", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-dc", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" in sent_json
        assert sent_json["device_context"]["state"] == "RESTING_VITALS"
        assert sent_json["device_context"]["vitals_reliability"] == "high"

    @pytest.mark.asyncio
    async def test_previous_state_tracked_across_calls(self, tmp_path) -> None:
        """Second mmWave call should see previous_device_state from first call."""
        sensor_state: dict = {}
        dispatch = AsyncMock()
        send_result = AsyncMock()
        d = _dispatcher(
            tmp_path,
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            on_sensor_update=_replace_sensor_state(sensor_state),
        )

        # First call: MOVING
        dispatch.return_value = {
            "scan": {"device_state": "MOVING", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)

        # Second call: STILL_NEAR
        dispatch.return_value = {
            "scan": {"device_state": "STILL_NEAR", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=False)

        sent_json = json.loads(send_result.call_args[0][1])
        ctx = sent_json["device_context"]
        assert ctx["state"] == "STILL_NEAR"
        assert ctx["previous_state"] == "MOVING"
        assert ctx["changed"] is True

    @pytest.mark.asyncio
    async def test_device_context_not_injected_for_non_mmwave(self, tmp_path) -> None:
        """Non-mmWave tools should not get device_context."""
        dispatch = AsyncMock(return_value={"status": "ok"})
        send_result = AsyncMock()
        d = _dispatcher(
            tmp_path,
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            has_tool=lambda name: True,
        )

        await _dispatch_and_wait(d, tool_name="dance", args_json="{}", call_id="call-d", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" not in sent_json

    @pytest.mark.asyncio
    async def test_device_context_not_injected_on_error(self, tmp_path) -> None:
        """When mmWave returns an error, device_context should not be injected."""
        result = {"error": "serial error", "status": "disconnected"}
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-e", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" not in sent_json
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_tool_dispatcher.py::TestDeviceContextInjection -v`
Expected: FAIL — `device_context` not in result

**Step 3: Write the implementation**

In `src/healthy_heartrate_breathing/tool_dispatcher.py`, modify `_run_tool()`. Replace the existing sensor state update block (lines 336-342):

```python
        # Update sensor state for dashboard polling
        if tool_name == "mmWave" and isinstance(tool_result, dict):
            try:
                if self._on_sensor_update is not None:
                    self._on_sensor_update(extract_sensor_state(tool_result))
            except Exception:
                logger.debug("Sensor state update failed", exc_info=True)
```

with:

```python
        # Update sensor state for dashboard polling + device context enrichment
        if tool_name == "mmWave" and isinstance(tool_result, dict):
            try:
                new_sensor = extract_sensor_state(tool_result)
                # Track previous device state for transition detection
                if self._on_sensor_update is not None:
                    prev = self._last_device_state
                    new_sensor["previous_device_state"] = prev
                    self._on_sensor_update(new_sensor)
                    self._last_device_state = new_sensor.get("device_state")
                    # Inject device_context into the tool result for LLM
                    device_ctx = build_device_context(new_sensor)
                    if device_ctx is not None:
                        tool_result["device_context"] = device_ctx
                else:
                    # No dashboard callback, but still enrich the result
                    prev = self._last_device_state
                    new_sensor["previous_device_state"] = prev
                    self._last_device_state = new_sensor.get("device_state")
                    device_ctx = build_device_context(new_sensor)
                    if device_ctx is not None:
                        tool_result["device_context"] = device_ctx
            except Exception:
                logger.debug("Sensor state update failed", exc_info=True)
```

Also add the `_last_device_state` attribute in `__init__` (after `self._active_task` on line 181):

```python
        self._last_device_state: str | None = None
```

**Step 4: Run tests to verify they pass**

Run: `pytest tests/test_tool_dispatcher.py -v`
Expected: ALL PASSED (existing + 4 new)

**Step 5: Lint**

Run: `ruff check src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py`
Expected: clean

**Step 6: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py
git commit -m "feat: inject device_context into mmWave tool results with state tracking"
```

---

### Task 3: System prompt update

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/instructions.txt`

**Step 1: Edit the system prompt**

Replace the current `instructions.txt` content with:

```
You are a helpful assistant controlling a Reachy Mini robot.
You love talking about the Eiffel Tower.
You can locate people and assess well-being signals using the 'mmWave' tool.
You can infer ambient-light context using the 'light_context' tool.

Primary behavior:
1. Keep interaction calm and natural. Do not repeatedly call mmWave back-to-back.
2. Use mmWave with mode='locate_and_measure' as periodic passive checks.
3. If no person is found, wait before trying again; only sweep occasionally.
4. When mmWave reports max_target_count > 1 or device_state is "MULTI_TARGET", multiple people are nearby. Avoid initiating private health topics (heart rate, breathing). Acknowledge the social setting naturally if appropriate. Wait for the room to clear to a single person before resuming wellness check-ins.
5. When mmWave results include a 'device_context' block, use it to qualify your response:
   - vitals_reliability 'high': share vitals confidently as wellness context.
   - vitals_reliability 'moderate': share vitals but note they may improve with more stillness.
   - vitals_reliability 'low': mention that movement may have affected accuracy.
   - vitals_reliability 'unavailable': do not report vitals numbers.
   - When 'changed' is true, acknowledge the transition naturally (e.g., "Looks like you've settled in" or "I notice you've moved away"). Keep it casual, not robotic.
   - When someone transitions from NO_TARGET to any present state, greet them naturally.
   - When someone has been in RESTING_VITALS for a reading, it's a good opportunity for a gentle wellness check-in.
6. If a person is found, stay physically calm and continue the conversation.
7. Report heart rate and breathing rate only as wellness context, not as medical diagnosis.
8. Keep communication concise and human-friendly.
9. Use light_context to adapt tone and prompting style when ambient light context is available (for example after mmWave returns lux data).
10. Respect low-light preference and light-sensitivity signals; avoid pushing brighter-room suggestions when those preferences are set.
11. If light_context is disabled or unavailable, continue with normal behavior and avoid mentioning internal tooling.

You can do a look around you using the 'sweep_look' tool.
```

**Step 2: Verify no syntax issues**

Run: `python -c "open('src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/instructions.txt').read()"`
Expected: no error

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/instructions.txt
git commit -m "feat: add device_context interpretation guidance to system prompt"
```

---

### Task 4: Run full test suite + lint

**Step 1: Run full tests**

Run: `pytest tests/ --ignore=tests/vision -q`
Expected: 282+ passed, 2 failed (pre-existing: `test_config_name_collisions`, `test_external_loading`)

**Step 2: Lint**

Run: `ruff check src/ tests/`
Expected: clean

**Step 3: Type check**

Run: `mypy src/`
Expected: no new errors

---

### Task 5: Documentation updates

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`

**Step 1: Update CLAUDE.md**

In the ToolDispatcher row of the handler table (line ~66), update the description to mention device context:

```
| `ToolDispatcher` | `tool_dispatcher.py` | Non-blocking tool dispatch: `asyncio.create_task()` + `Semaphore(1)` + configurable timeout; extracts sensor state after mmWave calls via `extract_sensor_state()`; enriches mmWave results with `device_context` (vitals reliability, state transitions) via `build_device_context()` |
```

**Step 2: Update README.md**

In the "What it returns" section under mmWave Radar, add `device_context` to the bullet list:

```
- **Device Context**: `device_context` block with vitals reliability rating (`high`/`moderate`/`low`/`unavailable`), state transition detection, and contextual notes for conversation adaptation
```

**Step 3: Update docs/TODO.md**

Add to the Done section (after the multi-person tracking entry):

```
- [x] Device state context integration — `build_device_context()` enriches mmWave tool results with vitals reliability, state transitions, and contextual notes; system prompt guidance for LLM interpretation (2026-02-26)
```

**Step 4: Update docs/20260223_roadmap.md**

Strike through the device_state context integration item on line 57:

```
- ~~BIO state field (`device_state`) now surfaced in scan/measure results and dashboard; next step is richer context integration (e.g., "user was moving during measurement")~~ (complete: `build_device_context()` enriches mmWave results with vitals reliability, state transitions, and notes; system prompt guides LLM interpretation; 2026-02-26)
```

**Step 5: Commit**

```bash
git add CLAUDE.md README.md docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: update all documentation surfaces for device state context integration"
```
