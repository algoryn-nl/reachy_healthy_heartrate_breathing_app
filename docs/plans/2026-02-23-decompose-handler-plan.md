# Decompose Handler and Fix Silent Exceptions — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add logging to all silent `except Exception: pass` blocks, then extract idle policy and light-context orchestration from `openai_realtime.py` into standalone testable classes.

**Architecture:** Two-phase approach. Phase 1 is a non-behavioral logging sweep across 5 files. Phase 2 extracts `IdlePolicy` and `LightOrchestrator` into new modules with composition in the handler. Both new classes are pure state machines testable without OpenAI connections.

**Tech Stack:** Python 3.12, pytest, pytest-asyncio, logging stdlib

---

### Task 1: Add logging to silent exceptions in openai_realtime.py

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py`

**Step 1: Add debug logging to connection event handlers**

Replace the 4 bare `except Exception: pass` blocks around `_connected_event` operations and connection close:

```python
# Line 560-562: in start_up() finally block
# Before:
                try:
                    self._connected_event.clear()
                except Exception:
                    pass
# After:
                try:
                    self._connected_event.clear()
                except Exception:
                    logger.debug("Failed to clear connected event in start_up", exc_info=True)

# Line 571-574: in _restart_session() connection close
# Before:
                try:
                    await self.connection.close()
                except Exception:
                    pass
# After:
                try:
                    await self.connection.close()
                except Exception:
                    logger.debug("Failed to close connection during restart", exc_info=True)

# Line 584-587: in _restart_session() event clear
# Before:
            try:
                self._connected_event.clear()
            except Exception:
                pass
# After:
            try:
                self._connected_event.clear()
            except Exception:
                logger.debug("Failed to clear connected event during restart", exc_info=True)

# Line 645-648: after session init
# Before:
            try:
                self._connected_event.set()
            except Exception:
                pass
# After:
            try:
                self._connected_event.set()
            except Exception:
                logger.debug("Failed to set connected event after session init", exc_info=True)
```

**Step 2: Add debug logging to voice discovery fallbacks**

```python
# Line 1050-1054: model serialization attempt
# Before:
                    try:
                        raw = fn()
                        break
                    except Exception:
                        pass
# After:
                    try:
                        raw = fn()
                        break
                    except Exception:
                        logger.debug("Model serialization via %s failed", attr, exc_info=True)

# Line 1055-1059: dict() conversion
# Before:
            if raw is None:
                try:
                    raw = dict(model)
                except Exception:
                    raw = None
# After:
            if raw is None:
                try:
                    raw = dict(model)
                except Exception:
                    logger.debug("dict() conversion of model object failed", exc_info=True)
                    raw = None

# Line 1078-1080: recursive voice collection
# Before:
                except Exception:
                    pass
# After:
                except Exception:
                    logger.debug("Voice candidate collection failed for object", exc_info=True)

# Line 1088-1090: entire voice discovery
# Before:
        except Exception:
            return fallback
# After:
        except Exception:
            logger.debug("Voice discovery failed; returning fallback list", exc_info=True)
            return fallback
```

**Step 3: Add debug logging to best-effort env write**

```python
# Line 1176-1181: os.environ assignment in _persist_api_key_if_needed
# Before:
            try:
                import os

                os.environ["OPENAI_API_KEY"] = key
            except Exception:  # best-effort
                pass
# After:
            try:
                import os

                os.environ["OPENAI_API_KEY"] = key
            except Exception:
                logger.debug("Best-effort os.environ OPENAI_API_KEY assignment failed", exc_info=True)
```

**Step 4: Verify no regressions**

Run: `python -m ruff check src/healthy_heartrate_breathing/openai_realtime.py`
Expected: All checks passed (or only pre-existing issues)

Run: `python -m pytest tests/ -x -q 2>&1 | tail -5`
Expected: Same pass/fail count as before (63 passed, 2 failed)

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "fix: add logging to silent exception handlers in openai_realtime.py"
```

---

### Task 2: Add logging to silent exceptions in console.py

**Files:**
- Modify: `src/healthy_heartrate_breathing/console.py`

**Step 1: Add warning/debug logging to _read_env_lines**

```python
# Line 80-83: .env file read
# Before:
                try:
                    return env_path.read_text(encoding="utf-8").splitlines()
                except Exception:
                    return []
# After:
                try:
                    return env_path.read_text(encoding="utf-8").splitlines()
                except Exception:
                    logger.warning("Failed to read .env at %s", env_path, exc_info=True)
                    return []

# Line 87-90: .env.example read (instance dir)
# Before:
                try:
                    template_text = ex.read_text(encoding="utf-8")
                except Exception:
                    template_text = None
# After:
                try:
                    template_text = ex.read_text(encoding="utf-8")
                except Exception:
                    logger.warning("Failed to read .env.example at %s", ex, exc_info=True)
                    template_text = None

# Line 93-97: .env.example read (cwd)
# Before:
                try:
                    cwd_example = Path.cwd() / ".env.example"
                    if cwd_example.exists():
                        template_text = cwd_example.read_text(encoding="utf-8")
                except Exception:
                    template_text = None
# After:
                try:
                    cwd_example = Path.cwd() / ".env.example"
                    if cwd_example.exists():
                        template_text = cwd_example.read_text(encoding="utf-8")
                except Exception:
                    logger.warning("Failed to read .env.example from cwd", exc_info=True)
                    template_text = None

# Line 101-104: .env.example read (packaged)
# Before:
                    try:
                        template_text = packaged.read_text(encoding="utf-8")
                    except Exception:
                        template_text = None
# After:
                    try:
                        template_text = packaged.read_text(encoding="utf-8")
                    except Exception:
                        logger.warning("Failed to read packaged .env.example at %s", packaged, exc_info=True)
                        template_text = None

# Line 106-107: outer catch-all
# Before:
        except Exception:
            return []
# After:
        except Exception:
            logger.warning("Unexpected error in _read_env_lines", exc_info=True)
            return []
```

**Step 2: Add logging to _persist_api_key**

```python
# Line 126-129: os.environ assignment
# Before:
        try:
            os.environ["OPENAI_API_KEY"] = k
        except Exception:  # best-effort
            pass
# After:
        try:
            os.environ["OPENAI_API_KEY"] = k
        except Exception:
            logger.debug("Best-effort os.environ OPENAI_API_KEY assignment failed", exc_info=True)

# Line 130-133: config assignment
# Before:
        try:
            config.OPENAI_API_KEY = k
        except Exception:
            pass
# After:
        try:
            config.OPENAI_API_KEY = k
        except Exception:
            logger.warning("Failed to set config.OPENAI_API_KEY", exc_info=True)

# Line 154-159: dotenv reload
# Before:
            try:
                from dotenv import load_dotenv

                load_dotenv(dotenv_path=str(env_path), override=True)
            except Exception:
                pass
# After:
            try:
                from dotenv import load_dotenv

                load_dotenv(dotenv_path=str(env_path), override=True)
            except Exception:
                logger.warning("Failed to reload .env after write at %s", env_path, exc_info=True)
```

**Step 3: Add logging to _persist_personality**

```python
# Line 168-173: set_custom_profile call
# Before:
        try:
            from healthy_heartrate_breathing.config import set_custom_profile

            set_custom_profile(selection)
        except Exception:
            pass
# After:
        try:
            from healthy_heartrate_breathing.config import set_custom_profile

            set_custom_profile(selection)
        except Exception:
            logger.warning("Failed to set custom profile to %r", selection, exc_info=True)

# Line 196-201: dotenv reload in _persist_personality
# Before:
            try:
                from dotenv import load_dotenv

                load_dotenv(dotenv_path=str(env_path), override=True)
            except Exception:
                pass
# After:
            try:
                from dotenv import load_dotenv

                load_dotenv(dotenv_path=str(env_path), override=True)
            except Exception:
                logger.warning("Failed to reload .env after personality persist at %s", env_path, exc_info=True)
```

**Step 4: Add logging to _read_persisted_personality**

```python
# Line 217-218
# Before:
        except Exception:
            pass
# After:
        except Exception:
            logger.warning("Failed to read persisted personality from %s", env_path, exc_info=True)
```

**Step 5: Add logging to settings UI and launch**

```python
# Line 238-240: static files mount
# Before:
            try:
                # Serve /static/* assets
                self._settings_app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
            except Exception:
                pass
# After:
            try:
                self._settings_app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
            except Exception:
                logger.warning("Failed to mount static files from %s", static_dir, exc_info=True)

# Line 264-268: tools initialized check
# Before:
            try:
                mod = sys.modules.get("healthy_heartrate_breathing.tools.core_tools")
                ready = bool(getattr(mod, "_TOOLS_INITIALIZED", False)) if mod else False
            except Exception:
                ready = False
# After:
            try:
                mod = sys.modules.get("healthy_heartrate_breathing.tools.core_tools")
                ready = bool(getattr(mod, "_TOOLS_INITIALIZED", False)) if mod else False
            except Exception:
                logger.debug("Failed to check tools initialization status", exc_info=True)
                ready = False

# Line 329-332: config.OPENAI_API_KEY assignment in launch
# Before:
                        try:
                            config.OPENAI_API_KEY = new_key
                        except Exception:
                            pass
# After:
                        try:
                            config.OPENAI_API_KEY = new_key
                        except Exception:
                            logger.warning("Failed to set config.OPENAI_API_KEY from instance .env", exc_info=True)

# Line 336-339: set_custom_profile in launch
# Before:
                            try:
                                set_custom_profile(new_profile.strip() or None)
                            except Exception:
                                pass  # Best-effort profile update
# After:
                            try:
                                set_custom_profile(new_profile.strip() or None)
                            except Exception:
                                logger.debug("Best-effort profile update from instance .env failed", exc_info=True)

# Line 340-341: outer instance .env loading
# Before:
            except Exception:
                pass  # Instance .env loading is optional; continue with defaults
# After:
            except Exception:
                logger.debug("Instance .env loading failed; continuing with defaults", exc_info=True)

# Line 391-392: personality route mounting
# Before:
            except Exception:
                pass
# After:
            except Exception:
                logger.warning("Failed to mount personality routes", exc_info=True)
```

**Step 6: Lint and verify**

Run: `python -m ruff check src/healthy_heartrate_breathing/console.py`
Expected: All checks passed

Run: `python -m pytest tests/ -x -q 2>&1 | tail -5`
Expected: Same pass/fail count

**Step 7: Commit**

```bash
git add src/healthy_heartrate_breathing/console.py
git commit -m "fix: add logging to silent exception handlers in console.py"
```

---

### Task 3: Add logging to silent exceptions in personality modules

**Files:**
- Modify: `src/healthy_heartrate_breathing/headless_personality.py`
- Modify: `src/healthy_heartrate_breathing/headless_personality_ui.py`
- Modify: `src/healthy_heartrate_breathing/gradio_personality.py`

**Step 1: Add logger to headless_personality.py**

This file has no logger. Add one after the imports:

```python
# After line 3 (from pathlib import Path), add:
import logging


logger = logging.getLogger(__name__)
```

Then update the 3 sites:

```python
# Line 55-56: list_personalities directory iteration
# Before:
    except Exception:
        pass
# After:
    except Exception:
        logger.warning("Failed to list personality profiles", exc_info=True)

# Line 85-86: available_tools_for shared tools
# Before:
    except Exception:
        pass
# After:
    except Exception:
        logger.warning("Failed to list shared tools", exc_info=True)

# Line 92-93: available_tools_for local tools
# Before:
    except Exception:
        pass
# After:
    except Exception:
        logger.warning("Failed to list local tools for %r", selected, exc_info=True)
```

**Step 2: Update headless_personality_ui.py**

Note: this file has `logger = logging.getLogger(__name__)` at line 206 (inside the function). The sites at lines 42, 65, 73 are inside `mount_personality_routes` which already has access to `logging` import. However, the logger is defined late. Move it earlier or use the module-level logger from the `logging` import.

Add module-level logger after imports (line 27):

```python
logger = logging.getLogger(__name__)
```

Remove the duplicate at line 206 (`logger = logging.getLogger(__name__)`).

Then update sites:

```python
# Line 42-43: FastAPI/pydantic import guard
# This is a pragma: no cover import guard — leave as-is, it's intentional

# Line 65-66: _startup_choice config read
# Before:
        except Exception:
            pass
# After:
        except Exception:
            logger.debug("Failed to read startup personality choice", exc_info=True)

# Line 73-74: _current_choice config read
# Before:
        except Exception:
            return DEFAULT_OPTION
# After:
        except Exception:
            logger.debug("Failed to read current personality choice", exc_info=True)
            return DEFAULT_OPTION

# Line 116-117: JSON body parse
# Before:
        try:
            raw = await request.json()
        except Exception:
            raw = {}
# After:
        try:
            raw = await request.json()
        except Exception:
            logger.warning("Failed to parse JSON body in /personalities/save", exc_info=True)
            raw = {}

# Line 157-158: form parse in save_raw
# Before:
        except Exception:
            pass
# After:
        except Exception:
            logger.debug("Failed to parse form data in /personalities/save_raw", exc_info=True)

# Line 166-167: JSON parse in save_raw
# Before:
        except Exception:
            pass
# After:
        except Exception:
            logger.debug("Failed to parse JSON body in /personalities/save_raw", exc_info=True)

# Line 239-240: request.json() in apply
# Before:
            except Exception:
                sel_name = None
# After:
            except Exception:
                logger.debug("Failed to parse JSON body in /personalities/apply", exc_info=True)
                sel_name = None

# Line 246-247: query param parse in apply
# Before:
            except Exception:
                pass
# After:
            except Exception:
                logger.debug("Failed to read persist query param", exc_info=True)

# Line 280-281: get_available_voices in _get_v
# Before:
            except Exception:
                return ["cedar"]
# After:
            except Exception:
                logger.debug("Voice discovery failed", exc_info=True)
                return ["cedar"]

# Line 286-287: run_coroutine_threadsafe in _voices
# Before:
        except Exception:
            return ["cedar"]
# After:
        except Exception:
            logger.debug("Voice endpoint failed", exc_info=True)
            return ["cedar"]
```

**Step 3: Update gradio_personality.py**

Add logger after imports:

```python
# After line 4 (from pathlib import Path), add:
import logging


logger = logging.getLogger(__name__)
```

Then update sites:

```python
# Line 55-56: _list_personalities
# Before:
        except Exception:
            pass
# After:
        except Exception:
            logger.warning("Failed to list personality profiles", exc_info=True)

# Line 155-156: _read_voice_for
# Before:
            except Exception:
                pass
# After:
            except Exception:
                logger.debug("Failed to read voice for %r", name, exc_info=True)

# Line 166-167: _fetch_voices outer
# Before:
            except Exception:
                return gr.update(choices=["cedar"], value="cedar")
# After:
            except Exception:
                logger.debug("Failed to fetch voices", exc_info=True)
                return gr.update(choices=["cedar"], value="cedar")

# Line 176-177: _available_tools_for shared
# Before:
            except Exception:
                pass
# After:
            except Exception:
                logger.warning("Failed to list shared tools", exc_info=True)

# Line 183-184: _available_tools_for local
# Before:
            except Exception:
                pass
# After:
            except Exception:
                logger.warning("Failed to list local tools for %r", selected, exc_info=True)

# Line 229-237: _new_personality
# Before:
            except Exception:
                return (
                    gr.update(),
                    ...
                    "Failed to initialize new personality.",
                    gr.update(),
                )
# After:
            except Exception:
                logger.warning("Failed to initialize new personality form", exc_info=True)
                return (
                    gr.update(),
                    ...
                    "Failed to initialize new personality.",
                    gr.update(),
                )
```

**Step 4: Lint all 3 files**

Run: `python -m ruff check src/healthy_heartrate_breathing/headless_personality.py src/healthy_heartrate_breathing/headless_personality_ui.py src/healthy_heartrate_breathing/gradio_personality.py`
Expected: All checks passed (may need `ruff check --fix` for import sorting)

**Step 5: Verify tests**

Run: `python -m pytest tests/ -x -q 2>&1 | tail -5`
Expected: Same pass/fail count

**Step 6: Commit**

```bash
git add src/healthy_heartrate_breathing/headless_personality.py src/healthy_heartrate_breathing/headless_personality_ui.py src/healthy_heartrate_breathing/gradio_personality.py
git commit -m "fix: add logging to silent exception handlers in personality modules"
```

---

### Task 4: Write failing tests for IdlePolicy

**Files:**
- Create: `tests/test_idle_policy.py`

**Step 1: Write the failing tests**

```python
"""Tests for IdlePolicy state machine."""
# ruff: noqa: D103

from __future__ import annotations

import pytest

from healthy_heartrate_breathing.idle_policy import IdlePolicy


def _policy(**overrides: object) -> IdlePolicy:
    defaults = {
        "interval_s": 15.0,
        "probe_interval_s": 40.0,
        "probe_duration_s": 5.0,
        "misses_before_sweep": 3,
        "sweep_cooldown_s": 150.0,
        "post_focus_quiet_s": 45.0,
    }
    defaults.update(overrides)
    return IdlePolicy(**defaults)


class TestSweepAllowed:
    def test_not_enough_misses(self) -> None:
        p = _policy(misses_before_sweep=3)
        p.consecutive_misses = 2
        assert p.sweep_allowed(now=100.0) is False

    def test_enough_misses_no_prior_sweep(self) -> None:
        p = _policy(misses_before_sweep=3)
        p.consecutive_misses = 3
        assert p.sweep_allowed(now=100.0) is True

    def test_enough_misses_within_cooldown(self) -> None:
        p = _policy(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=100.0) is False  # 50s < 150s cooldown

    def test_enough_misses_past_cooldown(self) -> None:
        p = _policy(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=250.0) is True  # 200s > 150s cooldown


class TestRecordResults:
    def test_record_target_found_resets_misses(self) -> None:
        p = _policy()
        p.consecutive_misses = 5
        p.record_target_found(now=100.0)
        assert p.consecutive_misses == 0
        assert p.last_focus_time == 100.0

    def test_record_no_target_increments_misses(self) -> None:
        p = _policy()
        p.consecutive_misses = 1
        p.record_no_target(sweep_was_used=False)
        assert p.consecutive_misses == 2

    def test_record_no_target_after_sweep_resets(self) -> None:
        p = _policy()
        p.consecutive_misses = 5
        p.record_no_target(sweep_was_used=True)
        assert p.consecutive_misses == 0

    def test_record_inconclusive_no_change(self) -> None:
        p = _policy()
        p.consecutive_misses = 3
        p.record_inconclusive()
        assert p.consecutive_misses == 3


class TestShouldTrigger:
    def test_not_enough_idle_time(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=10.0, is_moving=False, now=100.0) is False

    def test_enough_idle_time_not_moving(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True

    def test_enough_idle_time_but_moving(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=True, now=100.0) is False

    def test_suppressed_during_post_focus_quiet(self) -> None:
        p = _policy(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 80.0
        # now=100, so only 20s since focus < 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is False

    def test_allowed_after_post_focus_quiet(self) -> None:
        p = _policy(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 50.0
        # now=100, so 50s since focus > 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True


class TestBuildStrategyMessage:
    def test_contains_probe_duration(self) -> None:
        p = _policy(probe_duration_s=5.0)
        msg = p.build_strategy_message(sweep_allowed=False)
        assert "5.0" in msg or "5" in msg

    def test_sweep_allowed_message(self) -> None:
        p = _policy()
        msg = p.build_strategy_message(sweep_allowed=True)
        assert "sweep" in msg.lower()

    def test_no_sweep_message(self) -> None:
        p = _policy()
        msg = p.build_strategy_message(sweep_allowed=False)
        assert "passive" in msg.lower() or "still" in msg.lower()
```

**Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_idle_policy.py -v 2>&1 | head -10`
Expected: FAIL with `ModuleNotFoundError: No module named 'healthy_heartrate_breathing.idle_policy'`

---

### Task 5: Implement IdlePolicy

**Files:**
- Create: `src/healthy_heartrate_breathing/idle_policy.py`

**Step 1: Write the implementation**

```python
"""Idle scanning policy for mmWave-based wellness probes."""

from __future__ import annotations
import logging


logger = logging.getLogger(__name__)


class IdlePolicy:
    """Pure state machine managing idle mmWave scanning decisions.

    Tracks consecutive misses, sweep cooldowns, and post-focus quiet windows
    to decide when and how to probe.
    """

    def __init__(
        self,
        *,
        interval_s: float = 15.0,
        probe_interval_s: float = 40.0,
        probe_duration_s: float = 5.0,
        misses_before_sweep: int = 3,
        sweep_cooldown_s: float = 150.0,
        post_focus_quiet_s: float = 45.0,
    ) -> None:
        self.interval_s = interval_s
        self.probe_interval_s = probe_interval_s
        self.probe_duration_s = probe_duration_s
        self.misses_before_sweep = misses_before_sweep
        self.sweep_cooldown_s = sweep_cooldown_s
        self.post_focus_quiet_s = post_focus_quiet_s

        # Mutable state
        self.consecutive_misses: int = 0
        self.last_sweep_time: float | None = None
        self.last_focus_time: float | None = None

    def sweep_allowed(self, now: float) -> bool:
        """Return True if a sweep is allowed given miss count and cooldown."""
        if self.consecutive_misses < self.misses_before_sweep:
            return False
        if self.last_sweep_time is None:
            return True
        return (now - self.last_sweep_time) >= self.sweep_cooldown_s

    def should_trigger(self, idle_duration: float, is_moving: bool, now: float) -> bool:
        """Return True if an idle probe should fire now."""
        if idle_duration <= self.probe_interval_s:
            return False
        if is_moving:
            return False
        if self.last_focus_time is not None:
            since_focus = now - self.last_focus_time
            if since_focus < self.post_focus_quiet_s:
                logger.debug(
                    "Idle probe suppressed: post-focus quiet (%.1fs remaining)",
                    self.post_focus_quiet_s - since_focus,
                )
                return False
        return True

    def record_target_found(self, now: float) -> None:
        """Record that mmWave detected a target."""
        self.consecutive_misses = 0
        self.last_focus_time = now
        logger.info("Idle mmWave detected target; miss counter reset.")

    def record_no_target(self, sweep_was_used: bool) -> None:
        """Record that mmWave found no target."""
        if sweep_was_used:
            self.consecutive_misses = 0
            logger.info("Idle mmWave sweep found no target; miss counter reset.")
        else:
            self.consecutive_misses += 1
            logger.info(
                "Idle mmWave no target (miss %d/%d before sweep).",
                self.consecutive_misses,
                self.misses_before_sweep,
            )

    def record_inconclusive(self) -> None:
        """Record an inconclusive mmWave result (no state change)."""
        logger.info("Idle mmWave inconclusive; miss counter unchanged at %d.", self.consecutive_misses)

    def record_sweep_used(self, now: float) -> None:
        """Record that a sweep was performed."""
        self.last_sweep_time = now

    def build_strategy_message(self, sweep_allowed: bool) -> str:
        """Build the strategy fragment for the idle signal instructions."""
        if sweep_allowed:
            return "run one slow scan sweep if no target is found"
        return "do a passive check only and stay still if no target is found"
```

**Step 2: Run tests to verify they pass**

Run: `python -m pytest tests/test_idle_policy.py -v`
Expected: All 15 tests PASS

**Step 3: Lint**

Run: `python -m ruff check src/healthy_heartrate_breathing/idle_policy.py tests/test_idle_policy.py`
Expected: All checks passed (run `ruff check --fix` if import order issues)

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/idle_policy.py tests/test_idle_policy.py
git commit -m "feat: add IdlePolicy class with tests"
```

---

### Task 6: Write failing tests for LightOrchestrator

**Files:**
- Create: `tests/test_light_orchestrator.py`

**Step 1: Write the failing tests**

```python
"""Tests for LightOrchestrator baseline tracking and dispatch."""
# ruff: noqa: D103

from __future__ import annotations
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator


def _orchestrator(tmp_path: Path, **overrides: object) -> LightOrchestrator:
    defaults = {
        "enabled": True,
        "analytics_enabled": False,
        "user_id": "test-user",
        "prefers_dim": False,
        "light_sensitive": False,
        "allow_wellness_nudges": True,
        "day_start_hour": 7,
        "night_start_hour": 20,
        "low_lux_threshold": 40.0,
        "baseline_alpha": 0.15,
        "baseline_min_samples": 5,
        "baseline_path": tmp_path / "baseline.json",
        "analytics_path": tmp_path / "analytics.jsonl",
    }
    defaults.update(overrides)
    return LightOrchestrator(**defaults)


class TestLuxDelta:
    def test_no_previous_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        assert o.compute_lux_delta_60s(100.0, now=10.0) is None

    def test_normal_delta(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o._last_lux = 200.0
        o._last_lux_time = 0.0
        delta = o.compute_lux_delta_60s(100.0, now=30.0)
        # Drop of 100 in 30s, scaled to 60s = -200.0
        assert delta is not None
        assert abs(delta - (-200.0)) < 0.1

    def test_stale_reading_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o._last_lux = 200.0
        o._last_lux_time = 0.0
        # 700s gap exceeds 600s max
        assert o.compute_lux_delta_60s(100.0, now=700.0) is None


class TestBaseline:
    def test_save_and_load_roundtrip(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o.update_baseline(lux=100.0, local_hour=12)
        o.save_baseline()

        o2 = _orchestrator(tmp_path)
        o2.load_baseline()
        val = o2.get_typical_day_low_lux(12)
        # With 1 sample, min_samples=5, should fall back to avg
        # Actually first sample ema = 100.0, but samples=1 < 5
        # Fallback averages all daytime hours → still 100.0
        assert val is not None
        assert abs(val - 100.0) < 1.0

    def test_nighttime_not_recorded(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, day_start_hour=7, night_start_hour=20)
        o.update_baseline(lux=50.0, local_hour=22)  # nighttime
        o.save_baseline()

        o2 = _orchestrator(tmp_path)
        o2.load_baseline()
        assert o2.get_typical_day_low_lux(22) is None

    def test_is_daytime_hour(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, day_start_hour=7, night_start_hour=20)
        assert o.is_daytime_hour(12) is True
        assert o.is_daytime_hour(6) is False
        assert o.is_daytime_hour(20) is False
        assert o.is_daytime_hour(7) is True


class TestRunFromMmwave:
    @pytest.mark.asyncio
    async def test_disabled_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, enabled=False)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_no_lux_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_dispatches_light_context(self, tmp_path: Path) -> None:
        mock_result = {"context_state": "bright_active", "recommended_mode": "active"}
        dispatch = AsyncMock(return_value=mock_result)
        o = _orchestrator(tmp_path)
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result == mock_result
        dispatch.assert_called_once()
        # Verify the dispatched args contain lux
        call_args = dispatch.call_args
        args_json = call_args[0][1]  # second positional arg
        parsed = json.loads(args_json)
        assert parsed["lux"] == 300.0

    @pytest.mark.asyncio
    async def test_no_tool_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=False,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()
```

**Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_light_orchestrator.py -v 2>&1 | head -10`
Expected: FAIL with `ModuleNotFoundError: No module named 'healthy_heartrate_breathing.light_orchestrator'`

---

### Task 7: Implement LightOrchestrator

**Files:**
- Create: `src/healthy_heartrate_breathing/light_orchestrator.py`

**Step 1: Write the implementation**

Move the 9 methods from `openai_realtime.py` (lines 241-443) into a standalone class. The code is a direct extraction — same logic, same behavior, new home.

```python
"""Light-context orchestration: baseline tracking, analytics, auto-dispatch."""

from __future__ import annotations
import json
import logging
from typing import Any, Callable, Awaitable
from pathlib import Path
from datetime import datetime, timezone

from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result


logger = logging.getLogger(__name__)


class LightOrchestrator:
    """Standalone lux baseline tracker and auto light_context dispatcher.

    Owns all light-related state previously spread across OpenaiRealtimeHandler.
    Testable without OpenAI connections — dispatch is done via a callback.
    """

    def __init__(
        self,
        *,
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
    ) -> None:
        self.enabled = enabled
        self.analytics_enabled = analytics_enabled
        self.user_id = user_id
        self.prefers_dim = prefers_dim
        self.light_sensitive = light_sensitive
        self.allow_wellness_nudges = allow_wellness_nudges
        self.day_start_hour = day_start_hour
        self.night_start_hour = night_start_hour
        self.low_lux_threshold = low_lux_threshold
        self.baseline_alpha = baseline_alpha
        self.baseline_min_samples = baseline_min_samples
        self.baseline_path = baseline_path
        self.analytics_path = analytics_path

        # Mutable state
        self._baseline_state: dict[str, Any] = {"schema_version": 1, "users": {}}
        self._last_lux: float | None = None
        self._last_lux_time: float | None = None
        self._low_since_time: float | None = None

        if self.baseline_path is not None:
            self.load_baseline()

    # ---- Baseline persistence ----

    def load_baseline(self) -> None:
        """Load baseline JSON state from disk with safe fallback."""
        default: dict[str, Any] = {"schema_version": 1, "users": {}}
        if self.baseline_path is None:
            self._baseline_state = default
            return
        try:
            if not self.baseline_path.exists():
                self._baseline_state = default
                return
            parsed = json.loads(self.baseline_path.read_text(encoding="utf-8"))
            if not isinstance(parsed, dict):
                self._baseline_state = default
                return
            users = parsed.get("users")
            if not isinstance(users, dict):
                parsed["users"] = {}
            self._baseline_state = parsed
        except Exception as e:
            logger.warning("Failed loading light baseline from %s: %s", self.baseline_path, e)
            self._baseline_state = default

    def save_baseline(self) -> None:
        """Persist baseline JSON state to disk."""
        if self.baseline_path is None:
            return
        try:
            self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
            self.baseline_path.write_text(
                json.dumps(self._baseline_state, ensure_ascii=True, indent=2),
                encoding="utf-8",
            )
        except Exception as e:
            logger.warning("Failed writing light baseline to %s: %s", self.baseline_path, e)

    # ---- Time helpers ----

    def is_daytime_hour(self, hour: int) -> bool:
        """Return True for daytime hours using configured day/night cutoffs."""
        if self.day_start_hour == self.night_start_hour:
            return True
        if self.day_start_hour < self.night_start_hour:
            return self.day_start_hour <= hour < self.night_start_hour
        return hour >= self.day_start_hour or hour < self.night_start_hour

    # ---- Lux tracking ----

    def compute_lux_delta_60s(self, lux: float, now: float) -> float | None:
        """Estimate lux delta normalized to a 60s window."""
        if self._last_lux is None or self._last_lux_time is None:
            return None
        dt = now - self._last_lux_time
        if dt < 1.0 or dt > 600.0:
            return None
        raw_delta = lux - self._last_lux
        return float(raw_delta * (60.0 / max(1.0, dt)))

    def _get_or_create_user_entry(self) -> dict[str, Any]:
        """Get mutable baseline entry for the configured user id."""
        users = self._baseline_state.setdefault("users", {})
        if not isinstance(users, dict):
            users = {}
            self._baseline_state["users"] = users
        entry = users.get(self.user_id)
        if not isinstance(entry, dict):
            entry = {"hours": {}, "updated_at": None}
            users[self.user_id] = entry
        hours = entry.get("hours")
        if not isinstance(hours, dict):
            entry["hours"] = {}
        return entry

    def update_baseline(self, lux: float, local_hour: int) -> None:
        """Update per-user rolling lux baseline for daytime behavior."""
        if not self.is_daytime_hour(local_hour):
            return
        entry = self._get_or_create_user_entry()
        hours: dict[str, Any] = entry["hours"]
        bucket_key = str(local_hour)
        bucket = hours.get(bucket_key)
        if not isinstance(bucket, dict):
            bucket = {"ema_lux": None, "samples": 0}
            hours[bucket_key] = bucket

        prev_ema = bucket.get("ema_lux")
        prev_val = float(prev_ema) if isinstance(prev_ema, (int, float)) else None
        next_ema = lux if prev_val is None else (self.baseline_alpha * lux) + ((1.0 - self.baseline_alpha) * prev_val)
        samples = int(bucket.get("samples", 0)) + 1
        bucket["ema_lux"] = round(float(next_ema), 3)
        bucket["samples"] = samples
        entry["updated_at"] = datetime.now(timezone.utc).isoformat()
        self.save_baseline()

    def get_typical_day_low_lux(self, local_hour: int) -> float | None:
        """Return a user baseline lux value for personalization."""
        entry = self._get_or_create_user_entry()
        hours = entry.get("hours")
        if not isinstance(hours, dict):
            return None

        bucket = hours.get(str(local_hour))
        if isinstance(bucket, dict):
            samples = int(bucket.get("samples", 0))
            ema = bucket.get("ema_lux")
            if samples >= self.baseline_min_samples and isinstance(ema, (int, float)):
                return float(ema)

        fallback_values: list[float] = []
        for hour_str, raw_bucket in hours.items():
            if not isinstance(raw_bucket, dict):
                continue
            try:
                hour = int(hour_str)
            except Exception:
                continue
            if not self.is_daytime_hour(hour):
                continue
            ema = raw_bucket.get("ema_lux")
            samples = int(raw_bucket.get("samples", 0))
            if samples > 0 and isinstance(ema, (int, float)):
                fallback_values.append(float(ema))
        if not fallback_values:
            return None
        return float(sum(fallback_values) / len(fallback_values))

    # ---- Analytics ----

    def _append_analytics_event(self, *, source_tool: str, lux: float | None, result: dict[str, Any]) -> None:
        """Append one light-context analytics row as JSONL."""
        if not self.analytics_enabled or self.analytics_path is None:
            return
        payload = {
            "event": "light_context_decision",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "user_id": self.user_id,
            "source_tool": source_tool,
            "context_state": result.get("context_state"),
            "recommended_mode": result.get("recommended_mode"),
            "recommended_actions": result.get("recommended_actions"),
            "confidence": result.get("confidence"),
            "cooldown_hint_s": result.get("cooldown_hint_s"),
            "reason_codes": result.get("reason_codes"),
            "lux": lux,
            "observations": result.get("observations"),
        }
        try:
            self.analytics_path.parent.mkdir(parents=True, exist_ok=True)
            with self.analytics_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(payload, ensure_ascii=True) + "\n")
        except Exception as e:
            logger.warning("Failed writing light analytics event to %s: %s", self.analytics_path, e)

    # ---- Core orchestration ----

    async def run_from_mmwave(
        self,
        mmwave_result: dict[str, Any],
        *,
        is_idle: bool,
        has_tool: bool,
        dispatch_fn: Callable[[str, str, Any], Awaitable[dict[str, Any]]],
    ) -> dict[str, Any] | None:
        """Auto-run light_context after mmWave when lux data is available.

        Args:
            mmwave_result: Raw mmWave tool result dict.
            is_idle: Whether this was an idle (background) tool call.
            has_tool: Whether the light_context tool is available.
            dispatch_fn: Callable(tool_name, args_json, deps) -> result dict.
        """
        if not self.enabled:
            return None
        if not has_tool:
            return None

        lux = extract_lux_from_mmwave_result(mmwave_result)
        if lux is None:
            return None

        import asyncio

        now = asyncio.get_event_loop().time()
        local_hour = datetime.now().hour
        lux_delta_60s = self.compute_lux_delta_60s(lux, now)

        if lux <= self.low_lux_threshold:
            if self._low_since_time is None:
                self._low_since_time = now
        else:
            self._low_since_time = None

        low_light_duration_min = ((now - self._low_since_time) / 60.0) if self._low_since_time is not None else 0.0
        user_typical = self.get_typical_day_low_lux(local_hour)

        # Detect presence from mmWave result
        has_target = False
        scan = mmwave_result.get("scan")
        if isinstance(scan, dict):
            if isinstance(scan.get("latest_target"), dict):
                has_target = True
            recent = scan.get("recent_targets")
            if isinstance(recent, list) and len(recent) > 0:
                has_target = True
        measure = mmwave_result.get("measure")
        if isinstance(measure, dict) and bool(measure.get("success")):
            has_target = True

        args = {
            "lux": lux,
            "previous_lux": self._last_lux,
            "lux_delta_60s": lux_delta_60s,
            "presence_detected": has_target,
            "active_interaction": not is_idle,
            "low_light_duration_min": low_light_duration_min,
            "local_hour": local_hour,
            "prefers_dim": self.prefers_dim,
            "light_sensitive": self.light_sensitive,
            "allow_wellness_nudges": self.allow_wellness_nudges,
            "user_typical_day_low_lux": user_typical,
            "mmwave_result": mmwave_result,
        }
        result = await dispatch_fn("light_context", json.dumps(args))
        if isinstance(result, dict):
            self._append_analytics_event(source_tool="mmWave", lux=lux, result=result)

        self.update_baseline(lux=lux, local_hour=local_hour)
        self._last_lux = lux
        self._last_lux_time = now
        return result if isinstance(result, dict) else None
```

**Step 2: Run tests to verify they pass**

Run: `python -m pytest tests/test_light_orchestrator.py -v`
Expected: All 8 tests PASS

**Step 3: Lint**

Run: `python -m ruff check src/healthy_heartrate_breathing/light_orchestrator.py tests/test_light_orchestrator.py`
Expected: All checks passed

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/light_orchestrator.py tests/test_light_orchestrator.py
git commit -m "feat: add LightOrchestrator class with tests"
```

---

### Task 8: Integrate IdlePolicy into openai_realtime.py

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py`

**Step 1: Replace idle state initialization in __init__**

Replace lines 162-197 (idle scanning policy config) with:

```python
        from healthy_heartrate_breathing.idle_policy import IdlePolicy

        self.idle_policy = IdlePolicy(
            interval_s=env_float("HEALTHY_MM_WAVE_IDLE_DEFAULT_INTERVAL_S", 15.0, min_value=1.0),
            probe_interval_s=env_float("HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S", 40.0, min_value=1.0),
            probe_duration_s=env_float("HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S", 5.0, min_value=0.5),
            misses_before_sweep=env_int("HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP", 3, min_value=1),
            sweep_cooldown_s=env_float("HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S", 150.0, min_value=1.0),
            post_focus_quiet_s=env_float("HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S", 45.0, min_value=0.0),
        )
        logger.info(
            "mmWave idle policy: probe_interval=%.1fs, probe_duration=%.1fs, "
            "misses_before_sweep=%d, sweep_cooldown=%.1fs, post_focus_quiet=%.1fs",
            self.idle_policy.probe_interval_s,
            self.idle_policy.probe_duration_s,
            self.idle_policy.misses_before_sweep,
            self.idle_policy.sweep_cooldown_s,
            self.idle_policy.post_focus_quiet_s,
        )
```

**Step 2: Remove `_idle_mmwave_sweep_allowed` method** (lines 233-239)

Delete entirely — now lives in `IdlePolicy.sweep_allowed()`.

**Step 3: Update `emit()` method** (lines 962-988)

Replace idle check block with delegation:

```python
        now = asyncio.get_event_loop().time()
        has_mmwave = self._has_tool("mmWave")
        idle_interval = self.idle_policy.probe_interval_s if has_mmwave else self.idle_policy.interval_s
        idle_duration = now - self.last_activity_time
        if idle_duration > idle_interval and self.deps.movement_manager.is_idle():
            if has_mmwave and not self.idle_policy.should_trigger(idle_duration, is_moving=False, now=now):
                self.last_activity_time = now
                return None
            try:
                await self.send_idle_signal(idle_duration)
            except Exception as e:
                logger.warning("Idle signal skipped (connection closed?): %s", e)
                return None
            self.last_activity_time = now
```

**Step 4: Update `send_idle_signal()` method** (lines 1092-1148)

Replace strategy message building with delegation:

```python
        if has_mmwave:
            now = asyncio.get_event_loop().time()
            sweep_allowed = self.idle_policy.sweep_allowed(now)
            strategy = self.idle_policy.build_strategy_message(sweep_allowed)
            sweep_flag = "true" if sweep_allowed else "false"
            timestamp_msg = (
                f"[Idle time update: {self.format_timestamp()} - No activity for {idle_duration:.1f}s] "
                "Do one calm wellness scan cycle."
            )
            idle_instructions = (
                "You MUST respond with function calls only - no speech or text. "
                f"Call mmWave exactly once with mode='locate_and_measure', duration_s={self.idle_policy.probe_duration_s}, "
                f"sweep_if_unseen={sweep_flag}. Then stop. "
                f"Current strategy: {strategy}. "
                "Do not call dance, play_emotion, or sweep_look for this idle cycle."
            )
            logger.info(
                "Idle schedule: mmWave probe (misses=%d/%d, sweep_allowed=%s)",
                self.idle_policy.consecutive_misses,
                self.idle_policy.misses_before_sweep,
                sweep_allowed,
            )
```

**Step 5: Update tool result handling** (lines 754-819)

Replace inline idle arg rewriting and state tracking:

```python
                    if self.is_idle_tool_call and tool_name == "mmWave":
                        now = asyncio.get_event_loop().time()
                        idle_args = _safe_parse_args(args_json_str)
                        idle_mmwave_sweep_used = self.idle_policy.sweep_allowed(now)
                        idle_args["mode"] = "locate_and_measure"
                        idle_args["duration_s"] = self.idle_policy.probe_duration_s
                        idle_args["sweep_if_unseen"] = idle_mmwave_sweep_used
                        effective_args_json = json.dumps(idle_args)
                        if idle_mmwave_sweep_used:
                            self.idle_policy.record_sweep_used(now)
                        logger.info(
                            "Idle mmWave policy: misses=%d/%d, sweep_if_unseen=%s, args=%s",
                            self.idle_policy.consecutive_misses,
                            self.idle_policy.misses_before_sweep,
                            idle_mmwave_sweep_used,
                            _short_text(effective_args_json),
                        )
```

And the post-result recording block:

```python
                    if self.is_idle_tool_call and tool_name == "mmWave":
                        now = asyncio.get_event_loop().time()
                        if isinstance(tool_result, dict) and tool_result.get("error"):
                            logger.warning("Idle mmWave failed: %s", _short_text(tool_result.get("error")))
                        elif _mmwave_has_target(tool_result):
                            self.idle_policy.record_target_found(now)
                        elif _mmwave_is_no_target(tool_result):
                            self.idle_policy.record_no_target(idle_mmwave_sweep_used)
                        else:
                            self.idle_policy.record_inconclusive()
```

**Step 6: Verify tests**

Run: `python -m pytest tests/ -x -q 2>&1 | tail -5`
Expected: All existing tests still pass. IdlePolicy tests pass.

**Step 7: Lint**

Run: `python -m ruff check src/healthy_heartrate_breathing/openai_realtime.py`
Expected: All checks passed

**Step 8: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "refactor: integrate IdlePolicy into openai_realtime handler"
```

---

### Task 9: Integrate LightOrchestrator into openai_realtime.py

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py`

**Step 1: Replace light state initialization in __init__**

Replace lines 199-223 (light-context policy config) with:

```python
        from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator

        self.light_orchestrator = LightOrchestrator(
            enabled=env_flag("HEALTHY_AUTO_LIGHT_CONTEXT_ENABLED", True),
            analytics_enabled=env_flag("HEALTHY_LIGHT_ANALYTICS_ENABLED", True),
            user_id=(os.getenv("HEALTHY_LIGHT_CONTEXT_USER_ID", "default") or "default").strip() or "default",
            prefers_dim=env_flag("HEALTHY_LIGHT_PREFERS_DIM", False),
            light_sensitive=env_flag("HEALTHY_LIGHT_SENSITIVE", False),
            allow_wellness_nudges=env_flag("HEALTHY_LIGHT_ALLOW_WELLNESS_NUDGES", True),
            day_start_hour=env_int("HEALTHY_LIGHT_DAY_START_HOUR", 7, min_value=0, max_value=23),
            night_start_hour=env_int("HEALTHY_LIGHT_NIGHT_START_HOUR", 20, min_value=0, max_value=23),
            low_lux_threshold=env_float("HEALTHY_LIGHT_LOW_LUX_THRESHOLD", 40.0, min_value=0.0),
            baseline_alpha=env_float("HEALTHY_LIGHT_BASELINE_ALPHA", 0.15, min_value=0.01, max_value=1.0),
            baseline_min_samples=env_int("HEALTHY_LIGHT_BASELINE_MIN_SAMPLES", 5, min_value=1),
            baseline_path=self._resolve_runtime_data_path("light_context_baseline.json"),
            analytics_path=self._resolve_runtime_data_path("light_context_analytics.jsonl"),
        )
        logger.info(
            "Light-context policy: auto_enabled=%s, analytics=%s, user_id=%s, baseline=%s",
            self.light_orchestrator.enabled,
            self.light_orchestrator.analytics_enabled,
            self.light_orchestrator.user_id,
            self.light_orchestrator.baseline_path,
        )
```

**Step 2: Remove the 9 light-context methods from the handler**

Delete these methods entirely:
- `_load_light_baseline_state` (lines 246-262)
- `_save_light_baseline_state` (lines 264-271)
- `_is_daytime_hour` (lines 273-280)
- `_compute_lux_delta_60s` (lines 282-291)
- `_get_or_create_user_baseline_entry` (lines 293-306)
- `_update_user_lux_baseline` (lines 308-329)
- `_get_user_typical_day_low_lux` (lines 331-361)
- `_append_light_analytics_event` (lines 363-393)
- `_run_light_context_from_mmwave` (lines 395-443)

**Step 3: Update post-mmWave light context call** (line ~821-832)

Replace with delegation to orchestrator:

```python
                    if tool_name == "mmWave" and isinstance(tool_result, dict):
                        try:
                            async def _dispatch_light(name: str, args_json: str) -> dict[str, Any]:
                                return await dispatch_tool_call(name, args_json, self.deps)

                            auto_light_context = await self.light_orchestrator.run_from_mmwave(
                                tool_result,
                                is_idle=self.is_idle_tool_call,
                                has_tool=self._has_tool("light_context"),
                                dispatch_fn=_dispatch_light,
                            )
                            if auto_light_context is not None:
                                tool_result["light_context"] = auto_light_context
                                logger.info(
                                    "Auto light_context state=%s mode=%s",
                                    auto_light_context.get("context_state"),
                                    auto_light_context.get("recommended_mode"),
                                )
                        except Exception as e:
                            logger.warning("Auto light_context failed after mmWave: %s", e)
```

**Step 4: Clean up unused imports**

Remove `extract_lux_from_mmwave_result` from the import line if no longer used directly in this file. Keep `env_int`, `env_flag`, `env_float` as they're still used.

**Step 5: Verify all tests**

Run: `python -m pytest tests/ -v 2>&1 | tail -15`
Expected: All tests pass (63+ existing + 15 idle_policy + 8 light_orchestrator)

**Step 6: Lint**

Run: `python -m ruff check src/healthy_heartrate_breathing/openai_realtime.py`
Expected: All checks passed

**Step 7: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "refactor: integrate LightOrchestrator into openai_realtime handler"
```

---

### Task 10: Final verification and cleanup

**Files:**
- All modified files

**Step 1: Run full test suite**

Run: `python -m pytest tests/ -v`
Expected: All tests pass

**Step 2: Lint entire project**

Run: `python -m ruff check src/healthy_heartrate_breathing/`
Expected: No new issues (only pre-existing ones in untouched files)

**Step 3: Verify line count reduction**

Run: `wc -l src/healthy_heartrate_breathing/openai_realtime.py`
Expected: ~1050-1100 lines (down from 1216)

**Step 4: Verify no leftover private state**

Run: `grep -n "_idle_mmwave_\|_light_context_auto\|_light_analytics\|_light_user_id\|_light_prefers\|_light_sensitive\|_light_allow\|_light_day_start\|_light_night_start\|_light_low_lux\|_light_baseline" src/healthy_heartrate_breathing/openai_realtime.py`
Expected: No matches (all moved to IdlePolicy/LightOrchestrator)

**Step 5: Commit if any cleanup was needed**

```bash
git add -A
git commit -m "chore: final cleanup after handler decomposition"
```
