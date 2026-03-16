"""Shared data models and event processing for mmWave sensor data.

Consumed by both the TUI monitor (hardware/tools/mmwave_monitor.py)
and the Gradio web dashboard. Field names match the protocol dict keys
from mmwave_protocol.decode_event().
"""

from __future__ import annotations
from typing import Iterator
from collections import deque
from dataclasses import dataclass


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


# ---------------------------------------------------------------------------
# Event processing classes
# ---------------------------------------------------------------------------


class EventBuffer:
    """Ring buffer of recent DecodedEvents."""

    def __init__(self, max_size: int = 1000) -> None:
        """Initialize buffer with maximum capacity."""
        self._buf: deque[DecodedEvent] = deque(maxlen=max_size)

    def append(self, event: DecodedEvent) -> None:
        """Add an event to the buffer."""
        self._buf.append(event)

    def clear(self) -> None:
        """Remove all events."""
        self._buf.clear()

    def __len__(self) -> int:
        """Return number of buffered events."""
        return len(self._buf)

    def __iter__(self) -> Iterator[DecodedEvent]:
        """Iterate over buffered events oldest-first."""
        return iter(self._buf)


class NotableFilter:
    """Determines if an event is notable vs repetitive."""

    def __init__(self, lux_delta_threshold: float = 5.0) -> None:
        """Initialize filter with lux change threshold."""
        self._last_state: str | None = None
        self._last_lux: float | None = None
        self._lux_threshold = lux_delta_threshold

    def is_notable(self, event: DecodedEvent) -> bool:
        """Return True if the event is worth logging."""
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


class BioAcceptanceTracker:
    """Rolling bio acceptance rate."""

    def __init__(self) -> None:
        """Initialize with zero counts."""
        self._total = 0
        self._accepted = 0

    def record(self, allowed: int, valid: int) -> None:
        """Record a bio event's acceptance gate result."""
        self._total += 1
        if allowed and valid:
            self._accepted += 1

    def acceptance_rate(self) -> float:
        """Return fraction of bio events where allowed=1 AND valid=1."""
        return self._accepted / self._total if self._total else 0.0


class StateTransitionLog:
    """Compact log of state changes with durations."""

    def __init__(self) -> None:
        """Initialize empty transition log."""
        self._entries: list[tuple[str, float]] = []  # (state, start_ts)

    def record(self, state: str, timestamp: float) -> None:
        """Record a state observation. Duplicates are ignored."""
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


class TimeSeriesBuffer:
    """Fixed-size ring buffer for sparkline chart data."""

    def __init__(self, max_size: int = 300) -> None:
        """Initialize buffer with maximum number of data points."""
        self._timestamps: deque[float] = deque(maxlen=max_size)
        self._values: deque[float] = deque(maxlen=max_size)
        self._states: deque[str] = deque(maxlen=max_size)

    def append(self, timestamp: float, value: float, state: str) -> None:
        """Add a data point."""
        self._timestamps.append(timestamp)
        self._values.append(value)
        self._states.append(state)

    def get_series(self) -> tuple[list[float], list[float], list[str]]:
        """Return (timestamps, values, states) as plain lists."""
        return list(self._timestamps), list(self._values), list(self._states)

    def __len__(self) -> int:
        """Return number of data points in the buffer."""
        return len(self._values)


class EventRates:
    """Per-event-type rates via sliding window."""

    def __init__(self, window_s: float = 10.0) -> None:
        """Initialize with sliding window duration in seconds."""
        self._window_s = window_s
        self._events: dict[str, deque[float]] = {}

    def record(self, event_type: str, now: float) -> None:
        """Record an event occurrence."""
        if event_type not in self._events:
            self._events[event_type] = deque()
        self._events[event_type].append(now)

    def rate(self, event_type: str, now: float) -> float:
        """Return events/sec for the given type within the sliding window."""
        q = self._events.get(event_type)
        if not q:
            return 0.0
        cutoff = now - self._window_s
        while q and q[0] < cutoff:
            q.popleft()
        return len(q) / self._window_s
