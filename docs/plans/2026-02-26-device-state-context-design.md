# Device State Context Integration — Design

## Goal

Enrich mmWave tool results with a `device_context` dict so the LLM can qualify vitals reliability, acknowledge state transitions, and adapt conversational behavior based on the person's physical state (resting, moving, arriving, leaving).

## Architecture

ToolDispatcher already tracks `sensor_state` (including `device_state`) after every mmWave call for the dashboard. We extend this with `previous_device_state` tracking and a pure function `build_device_context()` that produces a structured context dict injected into the tool result at the top level — same pattern as `light_context`.

The system prompt is extended with a single rule explaining how to interpret the `device_context` fields.

## Changes

### 1. ToolDispatcher (`tool_dispatcher.py`)

**State tracking**: Before replacing `sensor_state`, snapshot the current `device_state` as `previous_device_state`:

```python
previous_device_state = (self._handler.sensor_state or {}).get("device_state")
new_sensor_state = extract_sensor_state(tool_result)
new_sensor_state["previous_device_state"] = previous_device_state
self._handler.sensor_state = new_sensor_state
```

**Context building**: New pure function `build_device_context(sensor_state) -> dict | None`:

- Reads `device_state` and `previous_device_state` from sensor_state
- Returns `None` if device_state is missing (error/disconnect)
- Returns a dict with: `state`, `previous_state`, `changed`, `vitals_reliability`, `transition`, `note`

**Injection**: After building, merge into tool_result:

```python
device_ctx = build_device_context(self._handler.sensor_state)
if device_ctx is not None:
    tool_result["device_context"] = device_ctx
```

### 2. `build_device_context` output

```json
{
    "state": "STILL_NEAR",
    "previous_state": "MOVING",
    "changed": true,
    "vitals_reliability": "moderate",
    "transition": "Target settled from MOVING to STILL_NEAR — vitals measurement should improve soon.",
    "note": "Person is close and settling. Wait for RESTING_VITALS for most reliable readings."
}
```

**Reliability mapping** (module-level constant dict):

| State | `vitals_reliability` | `note` |
|---|---|---|
| `RESTING_VITALS` | `high` | Person is still and close — ideal for heart rate and breathing measurement. |
| `STILL_NEAR` | `moderate` | Person is close and settling. Wait for RESTING_VITALS for most reliable readings. |
| `MOVING` | `low` | Person is moving — vitals readings may be inaccurate. |
| `PRESENT_FAR` | `unavailable` | Person detected but too far for vitals measurement. |
| `MULTI_TARGET` | `unavailable` | Multiple people detected — cannot isolate vitals. |
| `NO_TARGET` | `unavailable` | No person detected. |

Unknown states get `vitals_reliability: "unknown"` and a generic note. No crash.

**Transition notes**: Only when `changed` is True. Pattern: `"{previous} → {current}"` plus contextual suffix based on new state:

- MOVING → STILL_NEAR: "...vitals measurement should improve soon."
- STILL_NEAR → RESTING_VITALS: "...now in ideal position for vitals."
- anything → NO_TARGET: "...person left the area."
- NO_TARGET → PRESENT_FAR: "...someone just arrived."

### 3. System prompt (`instructions.txt`)

New rule 5 (existing 5-10 renumbered to 6-11):

```
5. When mmWave results include a 'device_context' block, use it to qualify your response:
   - vitals_reliability 'high': share vitals confidently as wellness context.
   - vitals_reliability 'moderate': share vitals but note they may improve with more stillness.
   - vitals_reliability 'low': mention that movement may have affected accuracy.
   - vitals_reliability 'unavailable': do not report vitals numbers.
   - When 'changed' is true, acknowledge the transition naturally (e.g., "Looks like you've settled in" or "I notice you've moved away"). Keep it casual, not robotic.
   - When someone transitions from NO_TARGET to any present state, greet them naturally.
   - When someone has been in RESTING_VITALS for a reading, it's a good opportunity for a gentle wellness check-in.
```

### 4. Testing

**`build_device_context()` unit tests** (~10 tests in `test_tool_dispatcher.py`):
- Each of 6 known states → correct reliability and note
- Unknown state → `reliability: "unknown"`, no error
- `changed` True when previous differs, False when same
- `previous_state` is None on first call
- Transition note only when `changed` is True
- None/missing device_state → returns None

**`_run_tool()` integration tests** (~4 tests in `test_tool_dispatcher.py`):
- `device_context` injected into mmWave tool result
- `previous_device_state` tracked across consecutive calls
- Not injected for non-mmWave tools
- Not injected on mmWave error

### 5. Documentation

- `CLAUDE.md`: Add device_context to ToolDispatcher description
- `README.md`: Mention device_context in mmWave returns section
- `docs/TODO.md`: Move to Done
- `docs/20260223_roadmap.md`: Strike through device_state context item
