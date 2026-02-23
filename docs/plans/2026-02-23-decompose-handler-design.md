# Decompose openai_realtime.py and Fix Silent Exception Swallowing

**Goal:** Extract idle policy and light-context orchestration from the 1293-line
`openai_realtime.py` into standalone, testable classes, and add logging to all
silent `except Exception: pass` blocks across the codebase.

**Approach:** Two-phase (Approach A). Phase 1 is a logging-only sweep with no
behavioral changes. Phase 2 is a structural refactor extracting two classes.

## Phase 1: Exception Logging Sweep

Add a log call to every bare `except Exception: pass` site. No behavior changes.

### Triage Rules

| Category | Level | `exc_info` |
|---|---|---|
| Config / file I/O failures | `logger.warning` | `True` |
| HTTP request body parsing | `logger.warning` | `True` |
| Connection state bookkeeping | `logger.debug` | `True` |
| Voice discovery fallbacks | `logger.debug` | `True` |
| Best-effort env writes | `logger.debug` | `True` |

### Sites to Fix

**openai_realtime.py** (~8 sites):

| Line | Operation | Level |
|---|---|---|
| 561 | `_connected_event.clear()` in shutdown | debug |
| 573 | Connection close in restart | debug |
| 586 | `_connected_event.clear()` in restart | debug |
| 647 | `_connected_event.set()` after session init | debug |
| 1053 | Model serialization attempt (voice list) | debug |
| 1058 | `dict()` conversion of model | debug |
| 1079 | Recursive voice candidate collection | debug |
| 1089 | Entire voice discovery fallback | debug |

**console.py** (~12 sites):

| Line(s) | Operation | Level |
|---|---|---|
| 82-83 | `.env` file read | warning |
| 89-90, 96-97, 103-107 | `.env.example` lookups | warning |
| 128-129, 132-133 | `os.environ` assignment | debug |
| 158-159 | dotenv reload after write | warning |
| 172-173 | `set_custom_profile()` call | warning |
| 200-201, 217-218 | dotenv loading | warning |
| 239-240 | Static files mount | warning |
| 267-268 | Tools initialized check | debug |
| 331-332, 338-339 | Config attribute assignment | warning |
| 391-392 | Personality route mounting | warning |

**headless_personality.py** (~3 sites):

| Line(s) | Operation | Level |
|---|---|---|
| 55-56 | Directory iteration (available profiles) | warning |
| 85-86 | Directory iteration (profile listing) | warning |
| 92-93 | Directory iteration (variant) | warning |

**headless_personality_ui.py** (~6 sites):

| Line(s) | Operation | Level |
|---|---|---|
| 42-43, 65-66, 73-74 | Directory operations | warning |
| 116-117 | JSON body parse from HTTP request | warning |
| 157-159, 166-167, 176-177, 183-184 | File operations | warning |
| 239-246 | Personality initialization | warning |

**gradio_personality.py** (~6 sites):

| Line(s) | Operation | Level |
|---|---|---|
| 55-56 | Personality list loading | warning |
| 73-74 | Instruction file reading | warning |
| 155-157 | File operations in UI callbacks | warning |
| 166-167, 176-177, 183-184 | File I/O in event handlers | warning |
| 229-237 | New personality initialization | warning |

### Pattern

```python
# Before
except Exception:
    pass

# After (dangerous)
except Exception:
    logger.warning("Failed to read .env file", exc_info=True)

# After (benign)
except Exception:
    logger.debug("Failed to clear connected event", exc_info=True)
```

Files that lack a `logger` will need `logger = logging.getLogger(__name__)` added.

## Phase 2: Extract IdlePolicy and LightOrchestrator

### New File: `src/healthy_heartrate_breathing/idle_policy.py`

Standalone class owning all idle scanning state and decision logic.

**State moved from handler:**
- `_idle_default_interval_s`
- `_idle_mmwave_probe_interval_s`
- `_idle_mmwave_probe_duration_s`
- `_idle_mmwave_misses_before_sweep`
- `_idle_mmwave_sweep_cooldown_s`
- `_idle_mmwave_post_focus_quiet_s`
- `_idle_mmwave_consecutive_misses`
- `_idle_mmwave_last_sweep_time`
- `_idle_mmwave_last_focus_time`

**Public interface:**

```python
class IdlePolicy:
    def __init__(self, *,
        interval_s: float = 15.0,
        probe_interval_s: float = 40.0,
        probe_duration_s: float = 5.0,
        misses_before_sweep: int = 3,
        sweep_cooldown_s: float = 150.0,
        post_focus_quiet_s: float = 45.0,
    ): ...

    def sweep_allowed(self, now: float) -> bool
    def should_trigger(self, idle_duration: float, is_moving: bool, now: float) -> bool
    def build_strategy_message(self) -> str
    def record_target_found(self) -> None
    def record_no_target(self, sweep_was_used: bool) -> None
    def record_inconclusive(self) -> None
    def record_sweep_used(self, now: float) -> None
    def record_focus(self, now: float) -> None
```

**Handler integration points:**
- `emit()`: call `idle_policy.should_trigger()`
- `send_idle_signal()`: call `idle_policy.build_strategy_message()`
- Post-mmWave result handling: call `record_target_found()` / `record_no_target()` / `record_inconclusive()`

### New File: `src/healthy_heartrate_breathing/light_orchestrator.py`

Standalone class owning lux baseline tracking, analytics, and auto light_context dispatch.

**State moved from handler:**
- `_light_context_auto_enabled`
- `_light_analytics_enabled`
- `_light_user_id`
- `_light_prefers_dim`, `_light_sensitive`, `_light_allow_wellness_nudges`
- `_light_day_start_hour`, `_light_night_start_hour`
- `_light_low_lux_threshold`
- `_light_baseline_alpha`, `_light_baseline_min_samples`
- `_light_baseline_path`, `_light_analytics_path`
- `_light_baseline_state`
- `_light_last_lux`, `_light_last_lux_time`
- `_light_low_since_time`

**Public interface:**

```python
class LightOrchestrator:
    def __init__(self, *,
        enabled: bool = True,
        analytics_enabled: bool = False,
        user_id: str = "default",
        prefers_dim: bool = False,
        light_sensitive: bool = False,
        allow_wellness_nudges: bool = True,
        day_start_hour: int = 7,
        night_start_hour: int = 20,
        low_lux_threshold: float = 40.0,
        baseline_alpha: float = 0.15,
        baseline_min_samples: int = 5,
        baseline_path: Path | None = None,
        analytics_path: Path | None = None,
    ): ...

    # Baseline management
    def load_baseline(self) -> None
    def save_baseline(self) -> None

    # Lux tracking
    def compute_lux_delta_60s(self, lux: float, now: float) -> float | None
    def update_baseline(self, lux: float, local_hour: int) -> None
    def get_typical_day_low_lux(self, local_hour: int) -> float | None

    # Core orchestration
    async def run_from_mmwave(
        self,
        mmwave_result: dict,
        *,
        is_idle: bool,
        dispatch_fn: Callable,
    ) -> dict | None
```

The `dispatch_fn` callback lets the orchestrator invoke the `light_context` tool
without importing or depending on `core_tools` directly.

**Handler integration points:**
- `__init__`: create `self.light_orchestrator = LightOrchestrator(...)`
- Post-mmWave result handling (line ~823): call `light_orchestrator.run_from_mmwave()`

**Methods removed from handler:** `_load_light_baseline_state`,
`_save_light_baseline_state`, `_is_daytime_hour`, `_compute_lux_delta_60s`,
`_get_or_create_user_baseline_entry`, `_update_user_lux_baseline`,
`_get_user_typical_day_low_lux`, `_append_light_analytics_event`,
`_run_light_context_from_mmwave`.

### Changes to openai_realtime.py

- **`__init__`**: Replace ~25 config lines with two instantiations
- **`emit()`**: Replace inline idle logic with `self.idle_policy.should_trigger()`
- **`send_idle_signal()`**: Delegate to `self.idle_policy.build_strategy_message()`
- **Tool result handling (lines 796-832)**: Call `idle_policy.record_*()` and
  `light_orchestrator.run_from_mmwave()`
- **Remove** 9 light methods + 1 idle method from the class

Estimated reduction: ~200 lines (1293 -> ~1090).

### Testing

Both new classes are pure state machines testable without OpenAI connections:

- `tests/test_idle_policy.py`: sweep_allowed logic, should_trigger timing,
  miss counting, strategy message content
- `tests/test_light_orchestrator.py`: baseline EMA, lux delta computation,
  run_from_mmwave dispatch with mock dispatch_fn

## File Summary

| File | Phase | Action |
|---|---|---|
| `openai_realtime.py` | 1 + 2 | Add logging (P1), extract classes (P2) |
| `console.py` | 1 | Add logging |
| `headless_personality.py` | 1 | Add logging |
| `headless_personality_ui.py` | 1 | Add logging |
| `gradio_personality.py` | 1 | Add logging |
| `idle_policy.py` | 2 | New file |
| `light_orchestrator.py` | 2 | New file |
| `tests/test_idle_policy.py` | 2 | New file |
| `tests/test_light_orchestrator.py` | 2 | New file |
