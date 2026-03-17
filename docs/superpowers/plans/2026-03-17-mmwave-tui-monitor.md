# mmWave TUI Monitor Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Textual-based TUI to `mmwave_decode.py` as the default output format, with shared data models for reuse by the Gradio dashboard.

**Architecture:** New `sensor_models.py` at package top level provides dataclasses and event processing. New `mmwave_monitor.py` in `hardware/tools/` is the Textual app. Existing `mmwave_decode.py` gains `--format tui` as default. Three parallel workstreams: data models, TUI widgets, CLI integration.

**Tech Stack:** Python 3.12, Textual, textual-plotext, pyserial, existing mmwave_protocol.py

**Spec:** `docs/superpowers/specs/2026-03-16-mmwave-tui-monitor-design.md`

---

## Parallel Execution Strategy

Three agents can work in parallel after Task 1 (deps):

| Agent | Tasks | Scope |
|-------|-------|-------|
| **A: Data Layer** | 2, 3, 4 | `sensor_models.py` — all dataclasses, event processing, serial reader |
| **B: TUI Widgets** | 5, 6, 7, 8, 9 | `mmwave_monitor.py` — app skeleton + all main tab widgets (uses mock data) |
| **C: Diag + CLI** | 10, 11 | Diagnostics tab + `mmwave_decode.py` CLI changes |

After all three complete, Task 12 (wiring) connects everything.

---

## Chunk 1: Foundation

### Task 1: Add Dependencies

**Files:**
- Modify: `pyproject.toml:16-17` (dev dependency group)

- [ ] **Step 1: Add textual and textual-plotext to dev group**

In `pyproject.toml`, change the `[dependency-groups]` dev list:

```toml
[dependency-groups]
dev = [ "pytest", "pytest-asyncio", "ruff==0.12.0", "mypy==1.18.2", "pre-commit", "types-requests", "python-semantic-release>=10.5.3", "textual>=0.80.0", "textual-plotext>=0.3.0",]
```

- [ ] **Step 2: Install and verify**

Run: `uv sync --group dev`
Expected: installs textual and textual-plotext without errors

- [ ] **Step 3: Verify textual imports**

Run: `uv run python -c "from textual.app import App; from textual_plotext import PlotextPlot; print('ok')"`
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add pyproject.toml uv.lock
git commit -m "chore: add textual and textual-plotext to dev dependencies"
```

### Task 2: Data Models

**Files:**
- Create: `src/healthy_heartrate_breathing/sensor_models.py`
- Create: `tests/test_sensor_models.py`

Reference: `mmwave_protocol.py:223-374` for field names in `decode_event()`.

- [ ] **Step 1: Write tests for all dataclasses**

```python
# tests/test_sensor_models.py
"""Tests for sensor_models data models and event processing."""

from __future__ import annotations

import time

import pytest

from healthy_heartrate_breathing.sensor_models import (
    BioReading,
    ConnectionInfo,
    DecodedEvent,
    DiagCounters,
    FrameStats,
    LightReading,
    SensorSnapshot,
    TargetInfo,
    TargetsEvent,
)


class TestDataModels:
    """Dataclass construction and defaults."""

    def test_sensor_snapshot_from_protocol_dict(self) -> None:
        """Fields match decode_event() EVT_STATE output."""
        s = SensorSnapshot(
            t_ms=12345,
            state="STILL_NEAR",
            pose="SITTING",
            human=1,
            n_targets=1,
            dist_cm=40.2,
            head_moving=0,
            dist_new=1,
        )
        assert s.state == "STILL_NEAR"
        assert s.dist_cm == 40.2

    def test_sensor_snapshot_none_dist(self) -> None:
        s = SensorSnapshot(
            t_ms=0, state="NO_TARGET", pose="UNKNOWN",
            human=0, n_targets=0, dist_cm=None, head_moving=0, dist_new=0,
        )
        assert s.dist_cm is None

    def test_bio_reading(self) -> None:
        b = BioReading(hr=83.0, br=6.0, allowed=1, valid=1, hr_new=1, br_new=1)
        assert b.hr == 83.0
        assert b.allowed == 1

    def test_bio_reading_none_values(self) -> None:
        b = BioReading(hr=None, br=None, allowed=0, valid=0, hr_new=0, br_new=0)
        assert b.hr is None

    def test_light_reading(self) -> None:
        lr = LightReading(lux=21.7, valid=1)
        assert lr.lux == 21.7

    def test_light_reading_invalid(self) -> None:
        lr = LightReading(lux=None, valid=0)
        assert lr.lux is None

    def test_target_info(self) -> None:
        t = TargetInfo(cluster=1, x=0.078, y=0.43, r=0.437, bearing=10.3, velocity=0.0)
        assert t.r == 0.437
        assert t.velocity == 0.0

    def test_targets_event(self) -> None:
        focus = TargetInfo(cluster=1, x=0.1, y=0.4, r=0.41, bearing=14.0, velocity=0.0)
        te = TargetsEvent(
            n_targets=2,
            forced_focus=0,
            focus=focus,
            targets=[focus, TargetInfo(cluster=2, x=0.5, y=1.0, r=1.12, bearing=26.5, velocity=0.1)],
            targets_truncated=False,
        )
        assert te.n_targets == 2
        assert te.focus is not None
        assert len(te.targets) == 2

    def test_targets_event_no_focus(self) -> None:
        te = TargetsEvent(n_targets=0, forced_focus=0, focus=None, targets=[], targets_truncated=False)
        assert te.focus is None

    def test_diag_counters_field_names_match_protocol(self) -> None:
        """Field names must match mmwave_protocol decode_event() EVT_DIAG keys."""
        d = DiagCounters(mmwave_fail_count=5, mmwave_consecutive_fails=2, tx_drop_count=1)
        assert d.mmwave_fail_count == 5
        assert d.mmwave_consecutive_fails == 2
        assert d.tx_drop_count == 1

    def test_connection_info(self) -> None:
        c = ConnectionInfo(port="/dev/cu.usbmodem2101", baud=115200, proto_version=1, feature_bits=0x0000)
        assert c.port == "/dev/cu.usbmodem2101"

    def test_frame_stats_defaults(self) -> None:
        f = FrameStats()
        assert f.frames_per_sec == 0.0
        assert f.bad_frame_count == 0

    def test_decoded_event(self) -> None:
        snap = SensorSnapshot(
            t_ms=100, state="MOVING", pose="STANDING",
            human=1, n_targets=1, dist_cm=80.0, head_moving=0, dist_new=1,
        )
        ev = DecodedEvent(event_type="state", host_ts=time.monotonic(), seq=42, data=snap)
        assert ev.event_type == "state"
        assert isinstance(ev.data, SensorSnapshot)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_sensor_models.py -v`
Expected: FAIL with import errors

- [ ] **Step 3: Implement dataclasses**

```python
# src/healthy_heartrate_breathing/sensor_models.py
"""Shared data models and event processing for mmWave sensor data.

Consumed by both the TUI monitor (hardware/tools/mmwave_monitor.py)
and the Gradio web dashboard. Field names match the protocol dict keys
from mmwave_protocol.decode_event().
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class SensorSnapshot:
    """EVT_STATE fields."""

    t_ms: int
    state: str
    pose: str
    human: int
    n_targets: int
    dist_cm: float | None
    head_moving: int
    dist_new: int


@dataclass(frozen=True, slots=True)
class BioReading:
    """EVT_BIO fields."""

    hr: float | None
    br: float | None
    allowed: int
    valid: int
    hr_new: int
    br_new: int


@dataclass(frozen=True, slots=True)
class LightReading:
    """EVT_LIGHT fields."""

    lux: float | None
    valid: int


@dataclass(frozen=True, slots=True)
class TargetInfo:
    """Single target from EVT_TARGETS. velocity mapped from protocol 'v' key."""

    cluster: int
    x: float
    y: float
    r: float
    bearing: float
    velocity: float


@dataclass(frozen=True, slots=True)
class TargetsEvent:
    """EVT_TARGETS envelope."""

    n_targets: int
    forced_focus: int
    focus: TargetInfo | None
    targets: list[TargetInfo]
    targets_truncated: bool


@dataclass(frozen=True, slots=True)
class DiagCounters:
    """EVT_DIAG fields. Names match protocol wire format exactly."""

    mmwave_fail_count: int
    mmwave_consecutive_fails: int
    tx_drop_count: int


@dataclass(frozen=True, slots=True)
class ConnectionInfo:
    """Sensor connection metadata."""

    port: str
    baud: int
    proto_version: int
    feature_bits: int


@dataclass(slots=True)
class FrameStats:
    """Locally computed frame statistics."""

    frames_per_sec: float = 0.0
    bytes_per_sec: float = 0.0
    bad_frame_count: int = 0


@dataclass(frozen=True, slots=True)
class DecodedEvent:
    """Core event type flowing through the system.

    Produced by read_events(), consumed by all widgets and processing classes.
    """

    event_type: str
    host_ts: float
    seq: int
    data: SensorSnapshot | BioReading | LightReading | TargetsEvent | DiagCounters | dict
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_sensor_models.py -v`
Expected: all PASS

- [ ] **Step 5: Lint**

Run: `ruff check src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py && ruff format --check src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py`

- [ ] **Step 6: Commit**

```bash
git add src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py
git commit -m "feat: add sensor data models for TUI monitor"
```

---

## Chunk 2: Event Processing (Agent A continues)

### Task 3: Event Processing Classes

**Files:**
- Modify: `src/healthy_heartrate_breathing/sensor_models.py`
- Modify: `tests/test_sensor_models.py`

- [ ] **Step 1: Write tests for EventBuffer**

Append to `tests/test_sensor_models.py`:

```python
from healthy_heartrate_breathing.sensor_models import EventBuffer


class TestEventBuffer:
    def test_append_and_iter(self) -> None:
        buf = EventBuffer(max_size=3)
        for i in range(3):
            buf.append(DecodedEvent("state", float(i), i, {}))
        assert len(buf) == 3

    def test_overflow_drops_oldest(self) -> None:
        buf = EventBuffer(max_size=2)
        for i in range(5):
            buf.append(DecodedEvent("state", float(i), i, {}))
        assert len(buf) == 2
        items = list(buf)
        assert items[0].seq == 3

    def test_clear(self) -> None:
        buf = EventBuffer(max_size=10)
        buf.append(DecodedEvent("state", 0.0, 0, {}))
        buf.clear()
        assert len(buf) == 0
```

- [ ] **Step 2: Run test, verify fail, implement EventBuffer**

```python
# Add to sensor_models.py
from collections import deque
from typing import Iterator

class EventBuffer:
    """Ring buffer of recent DecodedEvents."""

    def __init__(self, max_size: int = 1000) -> None:
        self._buf: deque[DecodedEvent] = deque(maxlen=max_size)

    def append(self, event: DecodedEvent) -> None:
        self._buf.append(event)

    def clear(self) -> None:
        self._buf.clear()

    def __len__(self) -> int:
        return len(self._buf)

    def __iter__(self) -> Iterator[DecodedEvent]:
        return iter(self._buf)
```

- [ ] **Step 3: Run test, verify pass**

- [ ] **Step 4: Write tests for NotableFilter**

```python
from healthy_heartrate_breathing.sensor_models import NotableFilter


class TestNotableFilter:
    def test_state_transition_is_notable(self) -> None:
        f = NotableFilter()
        snap1 = SensorSnapshot(0, "STILL_NEAR", "SITTING", 1, 1, 40.0, 0, 1)
        snap2 = SensorSnapshot(0, "MOVING", "STANDING", 1, 1, 50.0, 0, 1)
        assert not f.is_notable(DecodedEvent("state", 0.0, 1, snap1))  # first is always notable? or baseline?
        # Actually first event sets baseline — not notable (no transition)
        assert f.is_notable(DecodedEvent("state", 1.0, 2, snap2))  # transition

    def test_same_state_not_notable(self) -> None:
        f = NotableFilter()
        snap = SensorSnapshot(0, "STILL_NEAR", "SITTING", 1, 1, 40.0, 0, 1)
        f.is_notable(DecodedEvent("state", 0.0, 1, snap))
        assert not f.is_notable(DecodedEvent("state", 1.0, 2, snap))

    def test_bio_with_new_values_notable(self) -> None:
        f = NotableFilter()
        bio = BioReading(hr=83.0, br=6.0, allowed=1, valid=1, hr_new=1, br_new=1)
        assert f.is_notable(DecodedEvent("bio", 0.0, 1, bio))

    def test_bio_without_new_values_not_notable(self) -> None:
        f = NotableFilter()
        bio = BioReading(hr=83.0, br=6.0, allowed=1, valid=1, hr_new=0, br_new=0)
        assert not f.is_notable(DecodedEvent("bio", 0.0, 1, bio))

    def test_light_significant_change_notable(self) -> None:
        f = NotableFilter()
        f.is_notable(DecodedEvent("light", 0.0, 1, LightReading(lux=100.0, valid=1)))
        assert f.is_notable(DecodedEvent("light", 1.0, 2, LightReading(lux=90.0, valid=1)))  # 10 lux delta > 5

    def test_light_small_change_not_notable(self) -> None:
        f = NotableFilter()
        f.is_notable(DecodedEvent("light", 0.0, 1, LightReading(lux=100.0, valid=1)))
        assert not f.is_notable(DecodedEvent("light", 1.0, 2, LightReading(lux=98.0, valid=1)))  # 2 lux < 5

    def test_error_events_always_notable(self) -> None:
        f = NotableFilter()
        assert f.is_notable(DecodedEvent("err", 0.0, 1, {"cmd_id": 1, "err_code": 2}))
```

- [ ] **Step 5: Implement NotableFilter**

```python
class NotableFilter:
    """Determines if an event is notable vs repetitive."""

    def __init__(self, lux_delta_threshold: float = 5.0) -> None:
        self._last_state: str | None = None
        self._last_lux: float | None = None
        self._lux_threshold = lux_delta_threshold

    def is_notable(self, event: DecodedEvent) -> bool:
        if event.event_type in ("err", "ack", "hello"):
            return True

        if event.event_type == "state" and isinstance(event.data, SensorSnapshot):
            prev = self._last_state
            self._last_state = event.data.state
            return prev is not None and event.data.state != prev

        if event.event_type == "bio" and isinstance(event.data, BioReading):
            return bool(event.data.hr_new or event.data.br_new)

        if event.event_type == "light" and isinstance(event.data, LightReading):
            lux = event.data.lux
            if lux is None:
                return False
            prev = self._last_lux
            self._last_lux = lux
            if prev is None:
                return False
            return abs(lux - prev) >= self._lux_threshold

        return False
```

- [ ] **Step 6: Run tests, verify pass**

- [ ] **Step 7: Write tests for TimeSeriesBuffer and EventRates**

```python
from healthy_heartrate_breathing.sensor_models import EventRates, TimeSeriesBuffer


class TestTimeSeriesBuffer:
    def test_append_and_len(self) -> None:
        buf = TimeSeriesBuffer(max_size=100)
        buf.append(1.0, 83.0, "STILL_NEAR")
        buf.append(2.0, 84.0, "STILL_NEAR")
        assert len(buf) == 2

    def test_overflow(self) -> None:
        buf = TimeSeriesBuffer(max_size=3)
        for i in range(5):
            buf.append(float(i), float(60 + i), "STILL_NEAR")
        assert len(buf) == 3

    def test_values(self) -> None:
        buf = TimeSeriesBuffer(max_size=10)
        buf.append(1.0, 80.0, "STILL_NEAR")
        buf.append(2.0, 85.0, "MOVING")
        timestamps, values, states = buf.get_series()
        assert values == [80.0, 85.0]
        assert states == ["STILL_NEAR", "MOVING"]


class TestEventRates:
    def test_record_and_rate(self) -> None:
        rates = EventRates(window_s=10.0)
        now = time.monotonic()
        for _ in range(10):
            rates.record("state", now)
        assert rates.rate("state", now) == pytest.approx(10.0 / 10.0, abs=0.5)

    def test_unknown_type_zero(self) -> None:
        rates = EventRates(window_s=10.0)
        assert rates.rate("bio", time.monotonic()) == 0.0
```

- [ ] **Step 8: Implement TimeSeriesBuffer and EventRates**

```python
class TimeSeriesBuffer:
    """Fixed-size ring buffer for sparkline chart data."""

    def __init__(self, max_size: int = 300) -> None:
        self._timestamps: deque[float] = deque(maxlen=max_size)
        self._values: deque[float] = deque(maxlen=max_size)
        self._states: deque[str] = deque(maxlen=max_size)

    def append(self, timestamp: float, value: float, state: str) -> None:
        self._timestamps.append(timestamp)
        self._values.append(value)
        self._states.append(state)

    def get_series(self) -> tuple[list[float], list[float], list[str]]:
        return list(self._timestamps), list(self._values), list(self._states)

    def __len__(self) -> int:
        return len(self._values)


class EventRates:
    """Per-event-type rates via sliding window."""

    def __init__(self, window_s: float = 10.0) -> None:
        self._window_s = window_s
        self._events: dict[str, deque[float]] = {}

    def record(self, event_type: str, now: float) -> None:
        if event_type not in self._events:
            self._events[event_type] = deque()
        self._events[event_type].append(now)

    def rate(self, event_type: str, now: float) -> float:
        q = self._events.get(event_type)
        if not q:
            return 0.0
        cutoff = now - self._window_s
        while q and q[0] < cutoff:
            q.popleft()
        return len(q) / self._window_s
```

- [ ] **Step 9: Write tests for BioAcceptanceTracker and StateTransitionLog**

```python
from healthy_heartrate_breathing.sensor_models import BioAcceptanceTracker, StateTransitionLog


class TestBioAcceptanceTracker:
    def test_acceptance_rate(self) -> None:
        t = BioAcceptanceTracker()
        t.record(allowed=1, valid=1)
        t.record(allowed=1, valid=0)
        t.record(allowed=0, valid=0)
        assert t.acceptance_rate() == pytest.approx(1 / 3)

    def test_empty(self) -> None:
        t = BioAcceptanceTracker()
        assert t.acceptance_rate() == 0.0


class TestStateTransitionLog:
    def test_transitions(self) -> None:
        log = StateTransitionLog()
        log.record("STILL_NEAR", 1.0)
        log.record("MOVING", 3.0)
        log.record("STILL_NEAR", 5.0)
        transitions = log.transitions()
        assert len(transitions) == 3
        assert transitions[0] == ("STILL_NEAR", 1.0, 2.0)  # state, start, duration
        assert transitions[1] == ("MOVING", 3.0, 2.0)

    def test_no_transitions(self) -> None:
        log = StateTransitionLog()
        assert log.transitions() == []
```

- [ ] **Step 10: Implement BioAcceptanceTracker and StateTransitionLog**

```python
class BioAcceptanceTracker:
    """Rolling bio acceptance rate."""

    def __init__(self) -> None:
        self._total = 0
        self._accepted = 0

    def record(self, allowed: int, valid: int) -> None:
        self._total += 1
        if allowed and valid:
            self._accepted += 1

    def acceptance_rate(self) -> float:
        return self._accepted / self._total if self._total else 0.0


class StateTransitionLog:
    """Compact log of state changes with durations."""

    def __init__(self) -> None:
        self._entries: list[tuple[str, float]] = []  # (state, start_ts)

    def record(self, state: str, timestamp: float) -> None:
        if self._entries and self._entries[-1][0] == state:
            return  # same state, skip
        self._entries.append((state, timestamp))

    def transitions(self) -> list[tuple[str, float, float]]:
        """Return list of (state, start_ts, duration_s)."""
        result: list[tuple[str, float, float]] = []
        for i, (state, start) in enumerate(self._entries):
            if i + 1 < len(self._entries):
                duration = self._entries[i + 1][1] - start
            else:
                duration = 0.0  # current/last state, no end yet
            result.append((state, start, duration))
        return result
```

- [ ] **Step 11: Run all tests, lint, commit**

Run: `uv run pytest tests/test_sensor_models.py -v`
Run: `ruff check src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py`

```bash
git add src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py
git commit -m "feat: add event processing classes to sensor_models"
```

### Task 4: Async Serial Reader + dict-to-model converter

**Files:**
- Modify: `src/healthy_heartrate_breathing/sensor_models.py`
- Modify: `tests/test_sensor_models.py`

- [ ] **Step 1: Write tests for dict-to-model conversion**

The protocol returns plain dicts. We need a converter function.

```python
from healthy_heartrate_breathing.sensor_models import event_from_dict


class TestEventFromDict:
    def test_state_event(self) -> None:
        d = {"type": "state", "t_ms": 100, "state": "MOVING", "pose": "STANDING",
             "human": 1, "n_targets": 1, "dist_cm": 80.0, "head_moving": 0, "dist_new": 1}
        ev = event_from_dict(d, seq=5)
        assert ev.event_type == "state"
        assert isinstance(ev.data, SensorSnapshot)
        assert ev.data.state == "MOVING"

    def test_bio_event(self) -> None:
        d = {"type": "bio", "t_ms": 200, "hr": 83.0, "br": 6.0,
             "allowed": 1, "valid": 1, "hr_new": 1, "br_new": 1}
        ev = event_from_dict(d, seq=6)
        assert isinstance(ev.data, BioReading)
        assert ev.data.hr == 83.0

    def test_light_event(self) -> None:
        d = {"type": "light", "t_ms": 300, "lux": 21.7, "valid": 1}
        ev = event_from_dict(d, seq=7)
        assert isinstance(ev.data, LightReading)

    def test_targets_event(self) -> None:
        d = {"type": "targets", "t_ms": 400, "n": 1, "n_targets": 1, "forced_focus": 0,
             "focus": {"cluster": 1, "x": 0.1, "y": 0.4, "r": 0.41, "bearing": 14.0, "v": 0.0},
             "targets": [{"cluster": 1, "x": 0.1, "y": 0.4, "r": 0.41, "bearing": 14.0, "v": 0.0}],
             "targets_truncated": False}
        ev = event_from_dict(d, seq=8)
        assert isinstance(ev.data, TargetsEvent)
        assert ev.data.focus is not None
        assert ev.data.focus.velocity == 0.0  # mapped from "v"

    def test_targets_no_focus(self) -> None:
        d = {"type": "targets", "t_ms": 400, "n": 0, "n_targets": 0, "forced_focus": 0,
             "focus": None, "targets": [], "targets_truncated": False}
        ev = event_from_dict(d, seq=9)
        assert isinstance(ev.data, TargetsEvent)
        assert ev.data.focus is None

    def test_diag_event(self) -> None:
        d = {"type": "diag", "t_ms": 500, "mmwave_fail_count": 3,
             "mmwave_consecutive_fails": 1, "tx_drop_count": 0}
        ev = event_from_dict(d, seq=10)
        assert isinstance(ev.data, DiagCounters)

    def test_unknown_event_passes_through(self) -> None:
        d = {"type": "pong", "t_ms": 600}
        ev = event_from_dict(d, seq=11)
        assert ev.event_type == "pong"
        assert isinstance(ev.data, dict)
```

- [ ] **Step 2: Implement event_from_dict**

```python
import time as _time

def _target_from_dict(d: dict) -> TargetInfo:
    return TargetInfo(
        cluster=d["cluster"], x=d["x"], y=d["y"],
        r=d["r"], bearing=d["bearing"], velocity=d["v"],
    )

def event_from_dict(raw: dict, seq: int) -> DecodedEvent:
    """Convert a protocol decode_event() dict into a typed DecodedEvent."""
    evt_type = raw.get("type", "unknown")
    host_ts = _time.monotonic()
    data: SensorSnapshot | BioReading | LightReading | TargetsEvent | DiagCounters | dict

    if evt_type == "state":
        data = SensorSnapshot(
            t_ms=raw["t_ms"], state=raw["state"], pose=raw["pose"],
            human=raw["human"], n_targets=raw["n_targets"],
            dist_cm=raw.get("dist_cm"), head_moving=raw["head_moving"],
            dist_new=raw["dist_new"],
        )
    elif evt_type == "bio":
        data = BioReading(
            hr=raw.get("hr"), br=raw.get("br"),
            allowed=raw["allowed"], valid=raw["valid"],
            hr_new=raw["hr_new"], br_new=raw["br_new"],
        )
    elif evt_type == "light":
        data = LightReading(lux=raw.get("lux"), valid=raw["valid"])
    elif evt_type == "targets":
        focus_dict = raw.get("focus")
        focus = _target_from_dict(focus_dict) if focus_dict else None
        targets = [_target_from_dict(t) for t in raw.get("targets", [])]
        data = TargetsEvent(
            n_targets=raw.get("n_targets", raw.get("n", 0)),
            forced_focus=raw.get("forced_focus", 0),
            focus=focus, targets=targets,
            targets_truncated=raw.get("targets_truncated", False),
        )
    elif evt_type == "diag":
        data = DiagCounters(
            mmwave_fail_count=raw["mmwave_fail_count"],
            mmwave_consecutive_fails=raw["mmwave_consecutive_fails"],
            tx_drop_count=raw["tx_drop_count"],
        )
    else:
        data = raw

    return DecodedEvent(event_type=evt_type, host_ts=host_ts, seq=seq, data=data)
```

- [ ] **Step 3: Run tests, verify pass**

- [ ] **Step 4: Write tests for read_events async iterator**

```python
class TestReadEvents:
    @pytest.mark.asyncio
    async def test_read_events_yields_decoded_events(self) -> None:
        """read_events wraps serial decode loop into async DecodedEvent stream."""
        from unittest.mock import MagicMock, patch
        from healthy_heartrate_breathing.sensor_models import read_events

        # Create a mock serial port that returns a few COBS-framed packets then raises KeyboardInterrupt
        # Simplification: patch the lower-level serial read to yield known protocol dicts
        fake_events = [
            {"type": "state", "t_ms": 100, "state": "MOVING", "pose": "STANDING",
             "human": 1, "n_targets": 1, "dist_cm": 80.0, "head_moving": 0, "dist_new": 1},
            {"type": "bio", "t_ms": 200, "hr": 83.0, "br": 6.0,
             "allowed": 1, "valid": 1, "hr_new": 1, "br_new": 1},
        ]
        results = []
        async for ev in read_events(port=None, baud=115200, _test_events=fake_events):
            results.append(ev)
        assert len(results) == 2
        assert results[0].event_type == "state"
        assert results[1].event_type == "bio"
```

- [ ] **Step 5: Implement read_events async iterator**

```python
from typing import AsyncIterator
import asyncio

async def read_events(
    port: str | None,
    baud: int = 115200,
    *,
    _test_events: list[dict] | None = None,
) -> AsyncIterator[DecodedEvent]:
    """Async generator yielding DecodedEvents from serial or test data.

    Uses asyncio.to_thread() for blocking serial reads, matching mmWave.py pattern.
    Pass _test_events for testing without hardware.
    """
    if _test_events is not None:
        for i, raw in enumerate(_test_events):
            yield event_from_dict(raw, seq=i)
        return

    import serial as _serial
    from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
        PROTO_VERSION, ProtocolError, decode_event, decode_frame, extract_encoded_frames,
    )

    ser = _serial.Serial(port, baud, timeout=0.2)
    buffer = bytearray()
    seq = 0
    try:
        while True:
            chunk = await asyncio.to_thread(ser.read, max(ser.in_waiting, 1))
            if not chunk:
                continue
            buffer.extend(chunk)
            for encoded in extract_encoded_frames(buffer):
                try:
                    version, msg_type, seq_wire, payload = decode_frame(encoded)
                    if version != PROTO_VERSION:
                        continue
                    raw = decode_event(msg_type, payload)
                except ProtocolError:
                    continue
                yield event_from_dict(raw, seq=seq)
                seq += 1
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        ser.close()
```

- [ ] **Step 6: Run tests, verify pass**

- [ ] **Step 7: Lint and commit**

```bash
git add src/healthy_heartrate_breathing/sensor_models.py tests/test_sensor_models.py
git commit -m "feat: add event_from_dict converter and async serial reader"
```

---

## Chunk 3: TUI App Skeleton (Agent B)

### Task 5: Textual App Skeleton

**Files:**
- Create: `hardware/tools/mmwave_monitor.py`
- Create: `tests/test_mmwave_monitor.py`

- [ ] **Step 1: Write test for app mounting and keybindings**

```python
# tests/test_mmwave_monitor.py
"""Tests for mmWave TUI monitor."""

from __future__ import annotations

import pytest
from textual.testing import AppTest


# Import path assumes hardware/tools/ is on sys.path via the test runner
# or the module is importable. If not, adjust conftest.py.

@pytest.fixture
def app_test():
    """Create an AppTest instance with mock data mode."""
    # Deferred import to handle sys.path
    import importlib
    import sys
    from pathlib import Path

    tools_dir = Path(__file__).parent.parent / "hardware" / "tools"
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    mod = importlib.import_module("mmwave_monitor")
    return mod


class TestAppMounting:
    @pytest.mark.asyncio
    async def test_app_mounts(self, app_test) -> None:
        app = app_test.MmwaveMonitorApp(port=None, baud=115200, filter_types=None)
        async with app.run_test() as pilot:
            assert app.is_mounted

    @pytest.mark.asyncio
    async def test_quit_keybinding(self, app_test) -> None:
        app = app_test.MmwaveMonitorApp(port=None, baud=115200, filter_types=None)
        async with app.run_test() as pilot:
            await pilot.press("q")
            # App should exit
```

- [ ] **Step 2: Implement minimal Textual app**

```python
# hardware/tools/mmwave_monitor.py
"""Textual TUI for mmWave sensor monitoring."""

from __future__ import annotations

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import Footer, Header, Static, TabbedContent, TabPane


class MmwaveMonitorApp(App):
    """mmWave sensor monitor TUI."""

    TITLE = "mmWave Monitor"
    CSS = """
    Screen {
        background: $surface;
    }
    #main-grid {
        layout: grid;
        grid-size: 2 2;
        grid-gutter: 1;
        padding: 1;
    }
    .panel {
        border: solid $primary;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("l", "toggle_log", "Toggle Log"),
        Binding("f", "toggle_firehose", "Firehose/Notable"),
        Binding("d", "show_diag", "Diagnostics"),
        Binding("m", "show_main", "Main"),
        Binding("?", "show_help", "Help"),
    ]

    def __init__(
        self,
        port: str | None,
        baud: int = 115200,
        filter_types: set[str] | None = None,
    ) -> None:
        super().__init__()
        self._port = port
        self._baud = baud
        self._filter_types = filter_types

    def compose(self) -> ComposeResult:
        yield Header()
        with TabbedContent(initial="main"):
            with TabPane("Main", id="main"):
                yield Static("Main tab placeholder", id="main-content")
            with TabPane("Diag", id="diag"):
                yield Static("Diagnostics tab placeholder", id="diag-content")
        yield Footer()

    def action_toggle_log(self) -> None:
        pass  # TODO: implement

    def action_toggle_firehose(self) -> None:
        pass  # TODO: implement

    def action_show_diag(self) -> None:
        self.query_one(TabbedContent).active = "diag"

    def action_show_main(self) -> None:
        self.query_one(TabbedContent).active = "main"

    def action_show_help(self) -> None:
        pass  # TODO: help overlay
```

- [ ] **Step 3: Run tests, verify pass**
- [ ] **Step 4: Commit**

```bash
git add hardware/tools/mmwave_monitor.py tests/test_mmwave_monitor.py
git commit -m "feat: add Textual app skeleton with tabs and keybindings"
```

### Task 6: State Panel Widget

**Files:**
- Create: `hardware/tools/widgets/__init__.py`
- Create: `hardware/tools/widgets/state_panel.py`

The state panel shows: state icon (Rich Text art), state name, pose, lux bar, and detail metrics. Uses pre-drawn ASCII art per (state, pose) pair.

- [ ] **Step 1: Write tests for state icon lookup and widget rendering**
- [ ] **Step 2: Implement state icons as Rich Text blocks**
- [ ] **Step 3: Implement StatePanel widget extending Static**
- [ ] **Step 4: Test with mock SensorSnapshot + LightReading data**
- [ ] **Step 5: Commit**

```bash
git commit -m "feat: add state panel widget with iconic person figures"
```

### Task 7: Radar Widget

**Files:**
- Create: `hardware/tools/widgets/radar_panel.py`

Braille character rendering of radar view with range rings (1m intervals, 6m max), vitals zone (0.35-1.5m), focus target with glow, other targets colored by distance.

- [ ] **Step 1: Write tests for coordinate-to-braille mapping**
- [ ] **Step 2: Implement braille rendering helpers (plot_point, draw_arc)**
- [ ] **Step 3: Implement RadarPanel widget extending Static**
- [ ] **Step 4: Test with mock TargetsEvent data (1 target, 3 targets, 0 targets)**
- [ ] **Step 5: Commit**

```bash
git commit -m "feat: add radar panel with braille character rendering"
```

### Task 8: Vitals Panel Widget

**Files:**
- Create: `hardware/tools/widgets/vitals_panel.py`

Full-width panel with side-by-side HR/BR charts (textual-plotext), current value display, state annotation bar.

- [ ] **Step 1: Write tests for TimeSeriesBuffer → chart data conversion**
- [ ] **Step 2: Implement VitalsPanel with two PlotextPlot widgets**
- [ ] **Step 3: Implement state annotation bar as Rich Text colored blocks**
- [ ] **Step 4: Test with mock time series data**
- [ ] **Step 5: Commit**

```bash
git commit -m "feat: add vitals panel with sparkline charts and state annotation"
```

### Task 9: Log Panel Widget

**Files:**
- Create: `hardware/tools/widgets/log_panel.py`

Scrolling log with notable/firehose toggle, 3-line collapsed mode, filter support.

- [ ] **Step 1: Write tests for log formatting and notable filter integration**
- [ ] **Step 2: Implement LogPanel with RichLog widget, collapsed/expanded modes**
- [ ] **Step 3: Test toggle between firehose and notable modes**
- [ ] **Step 4: Test filter_types restricting visible events**
- [ ] **Step 5: Commit**

```bash
git commit -m "feat: add log panel with notable/firehose toggle"
```

---

## Chunk 4: Diagnostics + CLI (Agent C)

### Task 10: Diagnostics Tab

**Files:**
- Modify: `hardware/tools/mmwave_monitor.py` (replace diag placeholder)
- Create: `hardware/tools/widgets/diag_panel.py`

- [ ] **Step 1: Write tests for diagnostics panel rendering**
- [ ] **Step 2: Implement DiagPanel with protocol log, counters, connection info, computed stats**
- [ ] **Step 3: Test with mock DiagCounters, FrameStats, EventRates data**
- [ ] **Step 4: Commit**

```bash
git commit -m "feat: add diagnostics tab with protocol log and stats"
```

### Task 11: CLI Integration

**Files:**
- Modify: `hardware/tools/mmwave_decode.py`

- [ ] **Step 1: Fix pre-existing diag field name bug**

In `mmwave_decode.py:95-101`, the diag formatter uses wrong field names:

```python
# BEFORE (wrong):
f" consec={event.get('consecutive_fails', '?')}"
f" tx_drop={event.get('tx_drops', '?')}"

# AFTER (matches protocol):
f" consec={event.get('mmwave_consecutive_fails', '?')}"
f" tx_drop={event.get('tx_drop_count', '?')}"
```

- [ ] **Step 2: Add `tui` to format choices and make it default**

Change `main()` in `mmwave_decode.py`:

```python
parser.add_argument("--format", choices=["tui", "pretty", "json"], default="tui")
```

- [ ] **Step 3: Make `--port` / `--input-file` both optional for TUI mode**

Replace the mutually exclusive group with optional args:

```python
parser.add_argument("--port", help="Serial device path (auto-detected in TUI mode)")
parser.add_argument("--input-file", type=Path, help="Capture file path")
```

Add validation after parsing:
```python
if args.format in ("pretty", "json") and args.port is None and args.input_file is None:
    parser.error("--port or --input-file is required for --format pretty/json")
```

- [ ] **Step 4: Add TUI launch logic**

```python
if args.format == "tui":
    from mmwave_monitor import MmwaveMonitorApp
    app = MmwaveMonitorApp(
        port=args.port,
        baud=args.baud,
        filter_types=allowed_types,
        input_file=args.input_file,
    )
    app.run()
    return 0
```

- [ ] **Step 5: Test plain-text modes still work**

Run: `uv run python hardware/tools/mmwave_decode.py --port /dev/null --format pretty --show-bad-frames` (will error on /dev/null but validates arg parsing)

- [ ] **Step 6: Commit**

```bash
git add hardware/tools/mmwave_decode.py
git commit -m "feat: add --format tui as default, fix diag field names"
```

---

## Chunk 5: Wiring + Integration

### Task 12: Wire Serial Reader to TUI

**Files:**
- Modify: `hardware/tools/mmwave_monitor.py`

This task connects the async serial reader to the Textual widgets via Textual's worker pattern.

- [ ] **Step 1: Add serial reader worker to MmwaveMonitorApp**

```python
from textual.worker import Worker

async def _serial_worker(self) -> None:
    """Background worker reading serial events."""
    from healthy_heartrate_breathing.sensor_models import event_from_dict, read_events
    async for event in read_events(self._port, self._baud):
        self.call_from_thread(self._handle_event, event)
```

- [ ] **Step 2: Implement _handle_event to dispatch to widgets**

Routes each DecodedEvent to the appropriate panel's update method:
- state → StatePanel.update_state()
- bio → VitalsPanel.update_bio()
- light → StatePanel.update_lux()
- targets → RadarPanel.update_targets()
- diag → DiagPanel.update_counters()
- All events → LogPanel (respecting filter and notable mode)

- [ ] **Step 3: Add file playback support**

For `--input-file` mode, read all frames from file, feed through same pipeline, then set header to "Replay complete".

- [ ] **Step 4: Test end-to-end with a captured binary file**

If a capture file exists at `hardware/tests/` or can be created, test the full pipeline.

- [ ] **Step 5: Commit**

```bash
git add hardware/tools/mmwave_monitor.py hardware/tools/widgets/
git commit -m "feat: wire serial reader to TUI widgets"
```

### Task 13: Main Tab Layout Assembly

**Files:**
- Modify: `hardware/tools/mmwave_monitor.py`

- [ ] **Step 1: Replace main tab placeholder with grid layout**

Compose the main tab with: StatePanel + RadarPanel (top row), VitalsPanel (full width), LogPanel (collapsed).

- [ ] **Step 2: Add responsive CSS for <80 col terminals**

```css
@media (max-width: 80) {
    #radar-panel { display: none; }
    #main-grid { grid-size: 1; }
}
```

- [ ] **Step 3: Verify layout at various terminal sizes**
- [ ] **Step 4: Commit**

```bash
git commit -m "feat: assemble main tab layout with responsive grid"
```

### Task 14: Port Auto-Detection for TUI Mode

**Files:**
- Modify: `hardware/tools/mmwave_monitor.py`

When `--port` is omitted in TUI mode, auto-detect using the VID/PID strategy from `mmWave.py` (`0x303A:0x1001` → glob fallback → HELLO probe). Reference: `mmWave.py:_resolve_serial_port()`.

- [ ] **Step 1: Extract auto-detection logic into sensor_models.py**

```python
def detect_serial_port() -> str | None:
    """Auto-detect mmWave sensor serial port. Returns port path or None."""
    try:
        import serial.tools.list_ports
    except ImportError:
        return None
    # VID/PID match first
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x303A and p.pid == 0x1001:
            return p.device
    # Glob fallback (macOS/Linux)
    import glob
    candidates = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*")
    return candidates[0] if candidates else None
```

- [ ] **Step 2: Show "No sensor found" in TUI header when detection fails**

In `MmwaveMonitorApp.on_mount()`, if port is None and auto-detection returns None, set header subtitle to "No sensor — connect device and press r to retry".

- [ ] **Step 3: Add `r` keybinding for retry**
- [ ] **Step 4: Test auto-detection with mock serial.tools.list_ports**
- [ ] **Step 5: Commit**

```bash
git commit -m "feat: add serial port auto-detection for TUI mode"
```

### Task 15: Documentation Update

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`

Per CLAUDE.md rules, all four doc surfaces must be updated.

- [ ] **Step 1: Add TUI monitor to CLAUDE.md architecture section**
- [ ] **Step 2: Add `sensor_models.py` to project structure in CLAUDE.md**
- [ ] **Step 3: Add `textual`, `textual-plotext` to dependencies section in CLAUDE.md**
- [ ] **Step 4: Update common commands with TUI usage examples**
- [ ] **Step 5: Update README.md project structure and configuration**
- [ ] **Step 6: Update docs/TODO.md — move TUI task to Done**
- [ ] **Step 7: Update docs/20260223_roadmap.md if TUI was listed there**
- [ ] **Step 8: Commit**

```bash
git commit -m "docs: add TUI monitor to all documentation surfaces"
```
