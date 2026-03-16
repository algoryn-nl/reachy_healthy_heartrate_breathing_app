"""Shared data models and event processing for mmWave sensor data.

Consumed by both the TUI monitor (hardware/tools/mmwave_monitor.py)
and the Gradio web dashboard. Field names match the protocol dict keys
from mmwave_protocol.decode_event().
"""

from __future__ import annotations
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
