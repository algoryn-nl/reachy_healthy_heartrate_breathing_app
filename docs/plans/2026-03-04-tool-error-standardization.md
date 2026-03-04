# PY-MED-10: Tool Error Dict Standardization — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Standardize all tool return dicts to use `tool_ok()` and `tool_error()` helpers from `core_tools.py`, so every error has an `"error"` key and every success has a `"status"` key.

**Architecture:** Add two small helper functions (`tool_ok`, `tool_error`) to `core_tools.py`. Migrate all tool `__call__` methods and `dispatch_tool_call()` to use them. No new types, no consumer changes.

**Tech Stack:** Python 3.12, pytest, ruff

---

### Task 1: Add helper functions and tests

**Files:**
- Modify: `src/healthy_heartrate_breathing/tools/core_tools.py:35-37` (after ToolRegistryError)
- Create: `tests/test_tool_helpers.py`

**Step 1: Write the failing tests**

Create `tests/test_tool_helpers.py`:

```python
"""Tests for tool_ok / tool_error helper functions."""

from __future__ import annotations

from healthy_heartrate_breathing.tools.core_tools import tool_error, tool_ok


class TestToolOk:
    def test_default_status(self) -> None:
        assert tool_ok() == {"status": "ok"}

    def test_custom_status(self) -> None:
        assert tool_ok("queued") == {"status": "queued"}

    def test_extra_keys(self) -> None:
        result = tool_ok("queued", move="nod", repeat=2)
        assert result == {"status": "queued", "move": "nod", "repeat": 2}

    def test_no_error_key(self) -> None:
        result = tool_ok("ok", data=123)
        assert "error" not in result


class TestToolError:
    def test_basic_error(self) -> None:
        assert tool_error("boom") == {"error": "boom"}

    def test_extra_keys(self) -> None:
        result = tool_error("serial fail", status="disconnected")
        assert result == {"error": "serial fail", "status": "disconnected"}

    def test_always_has_error_key(self) -> None:
        result = tool_error("fail", tool="mmWave")
        assert "error" in result
        assert result["error"] == "fail"
```

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_tool_helpers.py -v`
Expected: FAIL with `ImportError: cannot import name 'tool_error'`

**Step 3: Implement the helpers**

In `src/healthy_heartrate_breathing/tools/core_tools.py`, add after line 37 (after `ToolRegistryError`):

```python
def tool_ok(status: str = "ok", **extra: Any) -> Dict[str, Any]:
    """Build a standardized tool success dict.

    Always contains ``"status"``; never contains ``"error"``.
    """
    return {"status": status, **extra}


def tool_error(message: str, **extra: Any) -> Dict[str, Any]:
    """Build a standardized tool error dict.

    Always contains ``"error"``; may carry additional keys
    (e.g. ``status="disconnected"`` for hardware faults).
    """
    return {"error": message, **extra}
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_tool_helpers.py -v`
Expected: PASS (7 tests)

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/tools/core_tools.py tests/test_tool_helpers.py
git commit -m "feat: add tool_ok/tool_error helper functions (PY-MED-10)"
```

---

### Task 2: Migrate `dispatch_tool_call()` in `core_tools.py`

**Files:**
- Modify: `src/healthy_heartrate_breathing/tools/core_tools.py:338-356`

**Step 1: Apply changes**

Replace the three return sites in `dispatch_tool_call()`:

Line 339-343 — disabled tool:
```python
# before
return {
    "status": "disabled",
    "tool": tool_name,
    "reason": "tool disabled via HEALTHY_DISABLED_TOOLS",
}
# after
return tool_ok("disabled", tool=tool_name, reason="tool disabled via HEALTHY_DISABLED_TOOLS")
```

Line 348 — unknown tool:
```python
# before
return {"error": f"unknown tool: {tool_name}"}
# after
return tool_error(f"unknown tool: {tool_name}")
```

Line 356 — exception during dispatch:
```python
# before
return {"error": msg}
# after
return tool_error(msg)
```

**Step 2: Run full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all pass (no behavior change — same dict shapes)

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/tools/core_tools.py
git commit -m "refactor: migrate dispatch_tool_call to tool_ok/tool_error"
```

---

### Task 3: Migrate simple core tools (dance, stop_dance, play_emotion, stop_emotion, do_nothing, head_tracking)

**Files:**
- Modify: `src/healthy_heartrate_breathing/tools/dance.py:4,65,78,86`
- Modify: `src/healthy_heartrate_breathing/tools/stop_dance.py:4,31`
- Modify: `src/healthy_heartrate_breathing/tools/play_emotion.py:4,61,65,73,80,84`
- Modify: `src/healthy_heartrate_breathing/tools/stop_emotion.py:4,31`
- Modify: `src/healthy_heartrate_breathing/tools/do_nothing.py:4,30`
- Modify: `src/healthy_heartrate_breathing/tools/head_tracking.py:4,31`

**Step 1: Apply changes — each file needs an updated import and return-site swaps**

`dance.py`:
- Import: add `tool_ok, tool_error` to import from `core_tools`
- Line 65: `return {"error": "Dance system not available"}` → `return tool_error("Dance system not available")`
- Line 78: `return {"error": f"Unknown dance move ..."}` → `return tool_error(f"Unknown dance move ...")`
- Line 86: `return {"status": "queued", "move": move_name, "repeat": repeat}` → `return tool_ok("queued", move=move_name, repeat=repeat)`

`stop_dance.py`:
- Import: add `tool_ok` to import from `core_tools`
- Line 31: `return {"status": "stopped dance and cleared queue"}` → `return tool_ok("stopped dance and cleared queue")`

`play_emotion.py`:
- Import: add `tool_ok, tool_error` to import from `core_tools`
- Line 61: `return {"error": "Emotion system not available"}` → `return tool_error("Emotion system not available")`
- Line 65: `return {"error": "Emotion name is required"}` → `return tool_error("Emotion name is required")`
- Line 73: `return {"error": f"Unknown emotion ..."}` → `return tool_error(f"Unknown emotion ...")`
- Line 80: `return {"status": "queued", "emotion": emotion_name}` → `return tool_ok("queued", emotion=emotion_name)`
- Line 84: `return {"error": f"Failed to play emotion: {e!s}"}` → `return tool_error(f"Failed to play emotion: {e!s}")`

`stop_emotion.py`:
- Import: add `tool_ok` to import from `core_tools`
- Line 31: `return {"status": "stopped emotion and cleared queue"}` → `return tool_ok("stopped emotion and cleared queue")`

`do_nothing.py`:
- Import: add `tool_ok` to import from `core_tools`
- Line 30: `return {"status": "doing nothing", "reason": reason}` → `return tool_ok("doing nothing", reason=reason)`

`head_tracking.py`:
- Import: add `tool_ok` to import from `core_tools`
- Line 31: `return {"status": f"head tracking {status}"}` → `return tool_ok(f"head tracking {status}")`

**Step 2: Run test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all pass

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/tools/dance.py \
        src/healthy_heartrate_breathing/tools/stop_dance.py \
        src/healthy_heartrate_breathing/tools/play_emotion.py \
        src/healthy_heartrate_breathing/tools/stop_emotion.py \
        src/healthy_heartrate_breathing/tools/do_nothing.py \
        src/healthy_heartrate_breathing/tools/head_tracking.py
git commit -m "refactor: migrate 6 core tools to tool_ok/tool_error"
```

---

### Task 4: Migrate `move_head.py` and `camera.py`

**Files:**
- Modify: `src/healthy_heartrate_breathing/tools/move_head.py:2,5,43,75,79`
- Modify: `src/healthy_heartrate_breathing/tools/camera.py:4,8,35,44,47`

**Step 1: Apply changes**

`move_head.py`:
- Import: add `tool_ok, tool_error` to import from `core_tools`
- Line 43: `return {"error": "direction must be a string"}` → `return tool_error("direction must be a string")`
- Line 75: `return {"status": f"looking {direction}"}` → `return tool_ok(f"looking {direction}")`
- Line 79: `return {"error": f"move_head failed: ..."}` → `return tool_error(f"move_head failed: {type(e).__name__}: {e}")`

`camera.py`:
- Import: add `tool_error` to import from `core_tools` (success paths return image data, left as-is)
- Line 35: `return {"error": "question must be a non-empty string"}` → `return tool_error("question must be a non-empty string")`
- Line 44: `return {"error": "No frame available"}` → `return tool_error("No frame available")`
- Line 47: `return {"error": "Camera worker not available"}` → `return tool_error("Camera worker not available")`

Note: `camera.py` line 61 `{"error": "vision returned non-string"}` also needs migration. Line 56-57 returns `vision_result` directly which is outside our control — leave it.

**Step 2: Run test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all pass

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/tools/move_head.py \
        src/healthy_heartrate_breathing/tools/camera.py
git commit -m "refactor: migrate move_head and camera to tool_ok/tool_error"
```

---

### Task 5: Migrate profile tools (sweep_look, custom_tool, light_context)

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/sweep_look.py:9,14,47,146`
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/custom_tool.py:6,39`
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/light_context.py:19,69-79,181-196`

**Step 1: Apply changes**

`sweep_look.py`:
- Import: add `tool_ok, tool_error` to import from `core_tools`
- Line 47: `return {"error": f"{type(e).__name__}: {e}"}` → `return tool_error(f"{type(e).__name__}: {e}")`
- Line 146: `return {"status": f"sweeping look..."}` → `return tool_ok(f"sweeping look left-right-center, total {total_duration:.1f}s")`

`custom_tool.py`:
- Import: add `tool_ok` to import from `core_tools`
- Line 39: `return {"status": "ok"}` → `return tool_ok()`

`light_context.py`:
- Import: add `tool_ok` to import from `core_tools`
- Lines 69-79 (disabled path): wrap in `tool_ok()`:
  ```python
  return tool_ok(
      "disabled",
      enabled=False,
      context_state="neutral",
      recommended_mode="balanced",
      recommended_actions=["standard_conversation_policy"],
      confidence=0.0,
      cooldown_hint_s=120,
      reason_codes=["tool_disabled"],
      reason="HEALTHY_LIGHT_CONTEXT_ENABLED=false",
  )
  ```
- Lines 181-196 (success path): wrap in `tool_ok()`:
  ```python
  return tool_ok(
      context_state,
      enabled=True,
      policy_version="proximity_context_v1",
      recommended_mode=recommended_mode,
      recommended_actions=recommended_actions,
      confidence=round(confidence, 2),
      cooldown_hint_s=cooldown_hint_s,
      reason_codes=reason_codes,
      observations={
          "lux": lux,
          "lux_delta_60s": lux_delta_60s,
          "presence_detected": presence_detected,
          "target_distance_cm": target_distance_cm,
      },
  )
  ```
  Note: `status` will be the `context_state` value (e.g. "clear_path", "close_presence") which is descriptive and useful for the LLM.

**Step 2: Run test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all pass. If light_context tests assert exact dict shapes, update them to expect `"status"` key.

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/sweep_look.py \
        src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/custom_tool.py \
        src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/light_context.py
git commit -m "refactor: migrate profile tools to tool_ok/tool_error"
```

---

### Task 6: Migrate mmWave tool

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py:647,653,658,698-702,757,760,763`

**Step 1: Apply changes**

Import: add `tool_ok, tool_error` to import from `core_tools`

Error paths (5 sites):
- Line 647: `return {"error": f"invalid mode..."}` → `return tool_error(f"invalid mode '{mode}', use scan, measure, or locate_and_measure")`
- Line 653: `return {"error": str(e), "status": "disconnected"}` → `return tool_error(str(e), status="disconnected")`
- Line 658: `return {"error": f"missing dependency..."}` → `return tool_error(f"missing dependency: {exc.name or 'pyserial'}")`
- Line 757: `return {"error": f"serial error...", "status": "disconnected"}` → `return tool_error(f"serial error on {serial_port}: {e}", status="disconnected")`
- Line 760: `return {"error": f"device I/O error...", "status": "disconnected"}` → `return tool_error(f"device I/O error on {serial_port}: {e}", status="disconnected")`
- Line 763: `return {"error": str(e)}` → `return tool_error(str(e))`

Handshake error (inside `run_session()` closure, line 698-702):
```python
return tool_error(handshake_err, serial_port=serial_port, status="version_mismatch")
```

Note: The success paths inside `run_session()` build a `response` dict incrementally with `response["status"] = "ok"` etc. These already have `"status"` and never have `"error"`, so they satisfy the contract without needing `tool_ok()`. Leave them as-is to avoid restructuring the incremental dict-building pattern.

**Step 2: Run test suite**

Run: `uv run pytest tests/test_mmwave.py -v && uv run pytest tests/ --ignore=tests/vision -x -q`
Expected: all pass

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py
git commit -m "refactor: migrate mmWave error paths to tool_error"
```

---

### Task 7: Lint, type-check, and final verification

**Step 1: Lint and format**

Run: `uv run ruff check src/ tests/ && uv run ruff format --check src/ tests/`

**Step 2: Type-check**

Run: `uv run mypy src/ tests/`

**Step 3: Full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -q`
Expected: all 375+ tests pass

**Step 4: Commit any fixes if needed**

---

### Task 8: Update documentation

**Files:**
- Modify: `CLAUDE.md` — add `tool_ok`/`tool_error` to Tool System section
- Modify: `docs/TODO.md` — mark PY-MED-10 done, update technical debt
- Modify: `docs/20260223_roadmap.md` — mark PY-MED-10 resolved

**Step 1: Update CLAUDE.md**

In the Tool System section, after the bullet about `dispatch_tool_call`, add:
- **`tool_ok(status, **extra)`** / **`tool_error(message, **extra)`**: standardized return dict builders — errors always have `"error"` key, successes always have `"status"` key

**Step 2: Update docs/TODO.md**

Mark `PY-MED-10` as done with date.

**Step 3: Update docs/20260223_roadmap.md**

Strike through PY-MED-10 in Medium Priority and add to Resolved section.

**Step 4: Commit**

```bash
git add CLAUDE.md docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: mark PY-MED-10 tool error standardization complete"
```
