# Multi-Person Tracking Logic Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add multi-target awareness to IdlePolicy so it backs off probing and suggests scan-only mode when multiple people are present, and enrich the LLM system prompt with multi-person conversational guidance.

**Architecture:** IdlePolicy gains a `record_multi_target(now)` method and `suggest_scan_only` property. ToolDispatcher routes multi-target results to this new method (priority over `record_target_found`). The idle signal in `send_idle_signal()` uses `suggest_scan_only` to pick `scan` vs `locate_and_measure` mode. A prompt hint in `instructions.txt` teaches the LLM to avoid private health topics when multiple people are present.

**Tech Stack:** Python 3.12, pytest, pytest-asyncio

---

### Task 1: IdlePolicy — Add `record_multi_target()` and `suggest_scan_only`

**Files:**
- Modify: `src/healthy_heartrate_breathing/idle_policy.py`
- Test: `tests/test_idle_policy.py`

**Step 1: Write the failing tests**

Add to `tests/test_idle_policy.py` at the bottom, a new test class:

```python
class TestMultiTarget:
    def test_record_multi_target_resets_misses(self) -> None:
        p = _policy()
        p.consecutive_misses = 5
        p.record_multi_target(now=100.0)
        assert p.consecutive_misses == 0
        assert p.last_focus_time is None  # multi-target does NOT set focus

    def test_record_multi_target_resets_errors(self) -> None:
        p = _policy(errors_before_suppression=2)
        p.record_error(now=90.0)
        p.record_error(now=95.0)
        assert p.sensor_suppressed is True
        p.record_multi_target(now=100.0)
        assert p.consecutive_errors == 0
        assert p.sensor_suppressed is False

    def test_record_multi_target_sets_timestamp(self) -> None:
        p = _policy()
        assert p.last_multi_target_time is None
        p.record_multi_target(now=100.0)
        assert p.last_multi_target_time == 100.0

    def test_multi_target_multiplies_probe_interval(self) -> None:
        p = _policy(probe_interval_s=40.0, multi_target_interval_multiplier=2.0)
        # Without multi-target: triggers at idle_duration > 40
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True
        # Record multi-target
        p.record_multi_target(now=100.0)
        # Now needs idle_duration > 40 * 2.0 = 80
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=101.0) is False
        assert p.should_trigger(idle_duration=90.0, is_moving=False, now=101.0) is True

    def test_multi_target_decays_to_normal_after_single_target(self) -> None:
        p = _policy(probe_interval_s=40.0, multi_target_interval_multiplier=2.0)
        p.record_multi_target(now=100.0)
        # Multiplied interval active
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=101.0) is False
        # Single target clears multi-target
        p.record_target_found(now=200.0)
        # After post-focus quiet window expires, normal interval applies
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=300.0) is True

    def test_suggest_scan_only_true_when_multi_target(self) -> None:
        p = _policy()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True

    def test_suggest_scan_only_false_initially(self) -> None:
        p = _policy()
        assert p.suggest_scan_only is False

    def test_suggest_scan_only_false_after_single_target(self) -> None:
        p = _policy()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True
        p.record_target_found(now=200.0)
        assert p.suggest_scan_only is False

    def test_suggest_scan_only_false_after_no_target(self) -> None:
        p = _policy()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True
        p.record_no_target(sweep_was_used=False)
        assert p.suggest_scan_only is False
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_idle_policy.py::TestMultiTarget -v`
Expected: FAIL — `IdlePolicy.__init__()` got unexpected keyword argument `multi_target_interval_multiplier`, and `record_multi_target` doesn't exist.

**Step 3: Implement in IdlePolicy**

In `src/healthy_heartrate_breathing/idle_policy.py`:

a) Add `multi_target_interval_multiplier` to the parameter table in the module docstring (after `error_backoff_s` row):

```
multi_target_interval_multiplier    2.0    HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER   >= 1.0
```

b) Add default constant and `__init__` parameter:

```python
# After DEFAULT_ERROR_BACKOFF_S
DEFAULT_MULTI_TARGET_INTERVAL_MULTIPLIER: float = 2.0
```

Add to `__init__` signature (after `error_backoff_s` param):

```python
multi_target_interval_multiplier: float = DEFAULT_MULTI_TARGET_INTERVAL_MULTIPLIER,
```

Add to `__init__` body (after `self.error_backoff_s = error_backoff_s`):

```python
self.multi_target_interval_multiplier = multi_target_interval_multiplier
```

Add to mutable state section (after `self.last_error_time`):

```python
self.last_multi_target_time: float | None = None
```

c) Add `suggest_scan_only` property (after `sensor_suppressed` property):

```python
@property
def suggest_scan_only(self) -> bool:
    """Return True when the last result was multi-target (measure would fail)."""
    return self.last_multi_target_time is not None
```

d) Add `record_multi_target` method (after `record_target_found`):

```python
def record_multi_target(self, now: float) -> None:
    """Record that mmWave detected multiple targets.

    Resets miss and error counters (the sensor is working), but does NOT
    set ``last_focus_time`` — multi-target is not a usable focus event.
    """
    self.consecutive_misses = 0
    self.last_multi_target_time = now
    if self.consecutive_errors > 0:
        logger.info("Idle mmWave sensor recovered after %d consecutive errors.", self.consecutive_errors)
    self.consecutive_errors = 0
    self.last_error_time = None
    logger.info("Idle mmWave detected multiple targets; backing off probing.")
```

e) In `record_target_found`, clear multi-target state — add after `self.last_focus_time = now`:

```python
self.last_multi_target_time = None  # single target clears multi-target
```

f) In `record_no_target`, clear multi-target state — add after `self.last_error_time = None`:

```python
self.last_multi_target_time = None
```

g) Modify `should_trigger` to apply interval multiplier. Replace the first check:

```python
# Before:
if idle_duration <= self.probe_interval_s:
    return False

# After:
effective_interval = self.probe_interval_s
if self.last_multi_target_time is not None:
    effective_interval *= self.multi_target_interval_multiplier
if idle_duration <= effective_interval:
    return False
```

**Step 4: Run tests to verify they pass**

Run: `pytest tests/test_idle_policy.py -v`
Expected: ALL PASS (existing + new)

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/idle_policy.py tests/test_idle_policy.py
git commit -m "feat: add multi-target awareness to IdlePolicy"
```

---

### Task 2: ToolDispatcher — Route multi-target results to `record_multi_target`

**Files:**
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py`
- Test: `tests/test_tool_dispatcher.py`

**Step 1: Write the failing tests**

Add to `tests/test_tool_dispatcher.py` at the bottom, a new test class:

```python
class TestMultiTargetRouting:
    @pytest.mark.asyncio
    async def test_multi_target_calls_record_multi_target(self, tmp_path) -> None:
        """When max_target_count > 1, record_multi_target is called instead of record_target_found."""
        policy = _idle_policy()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 3,
                "targets_seen": 2,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-mt", is_idle=True)
        assert policy.last_multi_target_time is not None
        assert policy.last_focus_time is None  # NOT set for multi-target
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_single_target_still_calls_record_target_found(self, tmp_path) -> None:
        """When max_target_count == 1, record_target_found is still called."""
        policy = _idle_policy()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 1,
                "targets_seen": 1,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-st", is_idle=True)
        assert policy.last_focus_time is not None
        assert policy.last_multi_target_time is None
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_tool_dispatcher.py::TestMultiTargetRouting -v`
Expected: FAIL — `test_multi_target_calls_record_multi_target` will fail because `_mmwave_has_target` returns True and `record_target_found` is called, setting `last_focus_time`.

**Step 3: Implement multi-target routing in ToolDispatcher**

In `src/healthy_heartrate_breathing/tool_dispatcher.py`, modify the idle mmWave result tracking block (lines 287-298). Add a helper function before `class ToolDispatcher`:

```python
def _mmwave_is_multi_target(result: Any) -> bool:
    """Return True when mmWave output indicates multiple targets."""
    if not isinstance(result, dict):
        return False
    scan = result.get("scan")
    if not isinstance(scan, dict):
        return False
    max_count = scan.get("max_target_count", 0)
    return isinstance(max_count, int) and max_count > 1
```

Then modify the result routing block (lines 288-298) to check multi-target first:

```python
# Idle mmWave result tracking
if is_idle and tool_name == "mmWave":
    now = asyncio.get_event_loop().time()
    if isinstance(tool_result, dict) and tool_result.get("error"):
        logger.warning("Idle mmWave failed: %s", _short_text(tool_result.get("error")))
        self._idle_policy.record_error(now)
    elif _mmwave_is_multi_target(tool_result):
        self._idle_policy.record_multi_target(now)
    elif _mmwave_has_target(tool_result):
        self._idle_policy.record_target_found(now)
    elif _mmwave_is_no_target(tool_result):
        self._idle_policy.record_no_target(sweep_was_used=idle_mmwave_sweep_used)
    else:
        self._idle_policy.record_inconclusive()
```

**Step 4: Run tests to verify they pass**

Run: `pytest tests/test_tool_dispatcher.py -v`
Expected: ALL PASS (existing + new)

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py
git commit -m "feat: route multi-target mmWave results to IdlePolicy"
```

---

### Task 3: Idle signal — Use `suggest_scan_only` for mode selection

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py` (lines 663-712)
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py` (lines 252-258)

**Step 1: Modify `send_idle_signal()` in `openai_realtime.py`**

In the `has_mmwave` branch of `send_idle_signal()` (line 668+), add scan-only mode:

After line 671 (`strategy = self.idle_policy.build_strategy_message(sweep_allowed)`), add:

```python
scan_only = self.idle_policy.suggest_scan_only
idle_mode = "scan" if scan_only else "locate_and_measure"
```

Replace line 679:
```python
# Before:
f"Call mmWave exactly once with mode='locate_and_measure', duration_s={self.idle_policy.probe_duration_s}, "

# After:
f"Call mmWave exactly once with mode='{idle_mode}', duration_s={self.idle_policy.probe_duration_s}, "
```

When `scan_only` is True, also suppress the sweep flag by replacing line 680:
```python
# Before:
f"sweep_if_unseen={sweep_flag}. Then stop. "

# After (only include sweep_if_unseen when not scan-only):
f"{'sweep_if_unseen=' + sweep_flag + '. ' if not scan_only else ''}Then stop. "
```

Update the log line at 684-688 to include scan_only:
```python
logger.info(
    "Idle schedule: mmWave probe (misses=%d/%d, sweep_allowed=%s, scan_only=%s)",
    self.idle_policy.consecutive_misses,
    self.idle_policy.misses_before_sweep,
    sweep_allowed,
    scan_only,
)
```

**Step 2: Modify idle arg override in `tool_dispatcher.py`**

In `_run_tool()` (line 256), change the hardcoded mode to respect `suggest_scan_only`:

```python
# Before:
idle_args["mode"] = "locate_and_measure"

# After:
idle_args["mode"] = "scan" if self._idle_policy.suggest_scan_only else "locate_and_measure"
```

**Step 3: Run full test suite**

Run: `pytest tests/test_idle_policy.py tests/test_tool_dispatcher.py tests/test_openai_realtime.py -v`
Expected: ALL PASS

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py src/healthy_heartrate_breathing/tool_dispatcher.py
git commit -m "feat: idle probes use scan-only mode when multi-target"
```

---

### Task 4: System prompt hint

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/instructions.txt`

**Step 1: Add multi-person guidance**

After line 9 (`3. If no person is found, wait before trying again; only sweep occasionally.`), add a new rule:

```
4. When mmWave reports max_target_count > 1 or device_state is "MULTI_TARGET", multiple people are nearby. Avoid initiating private health topics (heart rate, breathing). Acknowledge the social setting naturally if appropriate. Wait for the room to clear to a single person before resuming wellness check-ins.
```

Renumber the remaining rules (old 4→5, 5→6, 6→7, 7→8, 8→9, 9→10).

**Step 2: Verify no test breakage**

Run: `pytest tests/ -v`
Expected: ALL PASS (prompt content isn't tested directly)

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/instructions.txt
git commit -m "feat: add multi-person conversational guidance to system prompt"
```

---

### Task 5: Lint, type-check, and full test verification

**Step 1: Lint**

Run: `ruff check src/ tests/test_idle_policy.py tests/test_tool_dispatcher.py`
Expected: Clean (0 errors). If import order issues, run `ruff check --fix`.

**Step 2: Type check**

Run: `mypy src/`
Expected: No new errors (pre-existing count unchanged).

**Step 3: Full test suite**

Run: `pytest tests/ -v`
Expected: ALL PASS

**Step 4: Commit any fixes if needed**

---

### Task 6: Documentation updates

**Files:**
- Modify: `CLAUDE.md`
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`

**Step 1: Update `CLAUDE.md`**

a) In the env var table under "mmWave Idle Scanning Policy", add after `HEALTHY_MM_WAVE_ERROR_BACKOFF_S`:

```
| `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER` | `2.0` | Probe interval multiplier when multi-target is active |
```

b) In the IdlePolicy row of the handler table (around line with `idle_policy.py`), append to description:

```
; multi-target aware (backs off probing, suggests scan-only when >1 target)
```

c) Update test count if it changed (add 10 for IdlePolicy + 2 for ToolDispatcher = +12).

**Step 2: Update `docs/TODO.md`**

Add to the Done section:

```
- [x] Multi-person tracking logic — IdlePolicy multi-target awareness (record_multi_target, suggest_scan_only, interval multiplier), ToolDispatcher routing, scan-only idle mode, system prompt hint; 12 new tests (YYYY-MM-DD)
```

**Step 3: Update `docs/20260223_roadmap.md`**

In the "Sensor Intelligence" section, strike through the multi-person tracking item:

```
- ~~Multi-person tracking: targets truncation flag and target count now surfaced in scan results and dashboard; next step is behavioral logic for >1 target~~ (complete: IdlePolicy multi-target awareness with configurable interval multiplier, scan-only mode, ToolDispatcher routing, system prompt hint; YYYY-MM-DD)
```

**Step 4: Commit**

```bash
git add -f CLAUDE.md docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: update CLAUDE.md, TODO.md, roadmap for multi-person tracking"
```
