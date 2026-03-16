"""Tests for sensor_models data models and event processing."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import time

from healthy_heartrate_breathing.sensor_models import (
    BioReading,
    FrameStats,
    TargetInfo,
    DecodedEvent,
    DiagCounters,
    LightReading,
    TargetsEvent,
    ConnectionInfo,
    SensorSnapshot,
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
            t_ms=0,
            state="NO_TARGET",
            pose="UNKNOWN",
            human=0,
            n_targets=0,
            dist_cm=None,
            head_moving=0,
            dist_new=0,
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
            t_ms=100,
            state="MOVING",
            pose="STANDING",
            human=1,
            n_targets=1,
            dist_cm=80.0,
            head_moving=0,
            dist_new=1,
        )
        ev = DecodedEvent(event_type="state", host_ts=time.monotonic(), seq=42, data=snap)
        assert ev.event_type == "state"
        assert isinstance(ev.data, SensorSnapshot)
