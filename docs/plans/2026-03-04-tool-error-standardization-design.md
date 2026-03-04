# PY-MED-10: Standardize Tool Error Dict Format

## Problem

Tool `__call__` methods return inconsistent error/success dicts:
- Errors: some use `{"error": "..."}`, others `{"error": "...", "status": "disconnected"}`, others `{"status": "disabled", ...}`
- Success: some use `{"status": "queued", ...}`, others `{"status": "ok"}`, others omit status entirely

Consumers (`tool_dispatcher.py`, `extract_sensor_state()`) check `result.get("error")` — this works but there's no enforced contract.

## Design

### Approach: Convention + Helper Functions

Add two helpers to `core_tools.py`:

```python
def tool_error(message: str, **extra: Any) -> dict[str, Any]:
    """Standardized tool error dict. Always contains 'error' key."""
    return {"error": message, **extra}

def tool_ok(status: str = "ok", **extra: Any) -> dict[str, Any]:
    """Standardized tool success dict. Always contains 'status', never 'error'."""
    return {"status": status, **extra}
```

### Contract

- **Error**: dict always has `"error"` key (string). May have extra keys (e.g. `"status": "disconnected"` for hardware state).
- **Success**: dict always has `"status"` key (string). Never has `"error"` key.
- **Consumer test**: `result.get("error")` is truthy → error. This is unchanged.

### Migration Scope

~25 call sites across 13 files. All mechanical replacements.

**Simple tools** (1-3 sites each): dance, stop_dance, play_emotion, stop_emotion, camera, do_nothing, head_tracking, move_head, sweep_look, custom_tool.

**Rich tools**: mmWave (5 errors, success paths adopt `tool_ok()`), light_context (disabled path).

**Dispatcher**: `core_tools.py:dispatch_tool_call()` — 3 error returns.

### Not Changed

- Consumer code (`tool_dispatcher.py`, `extract_sensor_state()`) — already compatible.
- REST endpoints — different contract.
- Tests — dict shapes are compatible; no assertion changes needed.

### Edge Cases

- `light_context.py` disabled path: currently returns `{"enabled": False, "status": "disabled", ...}`. This is graceful degradation, not an error — use `tool_ok("disabled", enabled=False, ...)`.
- `camera.py` success: returns image content dict without `"status"`. Add `tool_ok()` wrapper or leave as-is since it has `"content"` key (no ambiguity). Decision: leave as-is — the image content list is the success signal.
- mmWave success paths: already have `"status": "ok"`. Adopt `tool_ok("ok", **data)`.
