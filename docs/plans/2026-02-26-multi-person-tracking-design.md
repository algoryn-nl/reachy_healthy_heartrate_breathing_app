# Design: Multi-Person Tracking Logic

## Context

The mmWave firmware already handles multi-target detection: when `nTargets > 1`, it enters `MULTI_TARGET` state, resets `vitalsStreak`, suppresses bio output (`allowed=0, valid=0, null sentinels`), and reports the count via `EVT_STATE` and `EVT_TARGETS`. The Python side surfaces `max_target_count`, `targets_truncated`, and `device_state` in scan results and the sensor dashboard.

However, the **idle policy treats multi-target exactly like single-target** — `_mmwave_has_target()` returns True (since `latest_target` is set from the closest target), triggering `record_target_found()` and the post-focus quiet window. This suppresses further probes for 45s even though no individual could be measured. In a persistent multi-person scenario, the system enters a loop of "found target → quiet window → probe → found target" without ever successfully measuring vitals.

## Design Decision

**Approach A (chosen): IdlePolicy-centric.** Add multi-target state tracking directly to `IdlePolicy`. The `ToolDispatcher` feeds a new `record_multi_target(now)` classification, and IdlePolicy applies a configurable interval multiplier and suggests scan-only mode.

Rejected: Approach B (ToolDispatcher-centric) — would break encapsulation by having ToolDispatcher reach into IdlePolicy internals for timing decisions.

## Changes

### 1. IdlePolicy — Multi-Target Awareness

**New state:**
- `last_multi_target_time: float | None` — timestamp of most recent multi-target result

**New config:**
- `multi_target_interval_multiplier: float` (default `2.0`, from `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER`)

**New method:**
- `record_multi_target(now: float)` — sets `last_multi_target_time = now`, resets `consecutive_misses = 0` (a target was found, just not a usable one)

**Interval adjustment:**
- In probe timing logic, if `last_multi_target_time` is recent (within the adjusted interval window), multiply the base probe interval by `multi_target_interval_multiplier`. This backs off probing since the social context makes vitals measurement unlikely.

**Mode hint:**
- `suggest_scan_only: bool` property — returns True when multi-target is the most recent result. Tells ToolDispatcher to use `scan` instead of `locate_and_measure`, since the measure phase can't succeed with >1 target.

### 2. ToolDispatcher — New Result Classification

In `_run_tool()` mmWave result routing (currently lines 288-298), add multi-target check with highest priority:

```python
if max_target_count > 1:
    idle_policy.record_multi_target(now)
elif _mmwave_has_target(result):
    idle_policy.record_target_found(now)
elif _mmwave_is_no_target(result):
    idle_policy.record_no_target(sweep_was_used)
else:
    idle_policy.record_inconclusive()
```

When `idle_policy.suggest_scan_only` is True, dispatch idle probes as `scan` instead of `locate_and_measure`.

No changes to `extract_sensor_state()` — it already surfaces all needed fields.

### 3. System Prompt Hint

Add to `instructions.txt` (locked profile) alongside existing sensor guidance:

> When the mmWave sensor reports `max_target_count > 1` or `device_state: "MULTI_TARGET"`, multiple people are in the room. In this situation: avoid initiating private health-related topics (heart rate, breathing), acknowledge the social setting naturally if appropriate, and wait for the room to clear before resuming wellness check-ins. Vitals measurements require a single stationary person.

### 4. Testing

**IdlePolicy tests** (`test_idle_policy.py`):
- `test_record_multi_target_resets_misses`
- `test_multi_target_multiplies_probe_interval`
- `test_multi_target_decays_to_normal`
- `test_suggest_scan_only_true_when_multi_target`
- `test_suggest_scan_only_false_after_single_target`

**ToolDispatcher tests** (`test_tool_dispatcher.py`):
- `test_multi_target_result_calls_record_multi_target`
- `test_multi_target_prefers_scan_mode`

### 5. Configuration

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER` | `2.0` | Multiplier for probe interval when multi-target is active |

### 6. Documentation

Update all four surfaces per CLAUDE.md rules:
- `CLAUDE.md` — env var table, IdlePolicy description
- `docs/TODO.md` — add to Done
- `docs/20260223_roadmap.md` — strike through multi-person tracking item
- `README.md` — sync if env var table present

## Data Flow (Updated)

```
mmWave result → ToolDispatcher._run_tool()
  ├─ max_target_count > 1 → idle_policy.record_multi_target(now)
  │                           → interval *= 2.0
  │                           → suggest_scan_only = True
  ├─ _mmwave_has_target()  → idle_policy.record_target_found(now)
  ├─ _mmwave_is_no_target()→ idle_policy.record_no_target(sweep)
  └─ else                  → idle_policy.record_inconclusive()

idle probe scheduling:
  if suggest_scan_only → dispatch "scan" (skip measure phase)
  else                 → dispatch "locate_and_measure"
```

## No Firmware Changes

The firmware already handles multi-target correctly. This design is purely Python-side behavioral logic.
