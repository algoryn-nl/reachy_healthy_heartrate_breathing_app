"""Tests for sensor_models data models and event processing."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import time

import pytest

from healthy_heartrate_breathing.sensor_models import (
    BioReading,
    EventRates,
    FrameStats,
    TargetInfo,
    EventBuffer,
    DecodedEvent,
    DiagCounters,
    LightReading,
    TargetsEvent,
    NotableFilter,
    ConnectionInfo,
    SensorSnapshot,
    TargetSmoother,
    TimeSeriesBuffer,
    StateTransitionLog,
    BioAcceptanceTracker,
    read_events,
    event_from_dict,
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


class TestNotableFilter:
    def test_state_transition_is_notable(self) -> None:
        f = NotableFilter()
        snap1 = SensorSnapshot(0, "STILL_NEAR", "SITTING", 1, 1, 40.0, 0, 1)
        snap2 = SensorSnapshot(0, "MOVING", "STANDING", 1, 1, 50.0, 0, 1)
        # First event sets baseline -- not notable (no transition)
        assert not f.is_notable(DecodedEvent("state", 0.0, 1, snap1))
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


class TestTargetSmoother:
    def test_first_value_passes_through(self) -> None:
        s = TargetSmoother(alpha=0.35)
        sx, sy = s.smooth(0, 1.0, 2.0, 100.0)
        assert sx == 1.0
        assert sy == 2.0

    def test_ema_smoothing(self) -> None:
        s = TargetSmoother(alpha=0.5)
        s.smooth(0, 0.0, 1.0, 100.0)
        sx, sy = s.smooth(0, 1.0, 1.0, 100.1)
        assert sx == pytest.approx(0.5)
        assert sy == pytest.approx(1.0)

    def test_stale_cluster_resets(self) -> None:
        s = TargetSmoother(alpha=0.5, stale_s=1.0)
        s.smooth(0, 0.0, 1.0, 100.0)
        sx, sy = s.smooth(0, 2.0, 3.0, 102.0)  # >1s gap
        assert sx == 2.0  # reset, not smoothed
        assert sy == 3.0

    def test_prune_removes_stale(self) -> None:
        s = TargetSmoother(alpha=0.5, stale_s=1.0)
        s.smooth(0, 0.0, 1.0, 100.0)
        s.smooth(1, 1.0, 1.0, 101.5)
        s.prune(101.5)
        # Cluster 0 (last seen at 100.0) should be pruned, cluster 1 kept
        assert 0 not in s._state
        assert 1 in s._state

    def test_independent_clusters(self) -> None:
        s = TargetSmoother(alpha=0.5)
        s.smooth(0, 0.0, 0.0, 100.0)
        s.smooth(1, 10.0, 10.0, 100.0)
        sx0, sy0 = s.smooth(0, 1.0, 1.0, 100.1)
        sx1, sy1 = s.smooth(1, 10.0, 10.0, 100.1)
        assert sx0 == pytest.approx(0.5)
        assert sx1 == pytest.approx(10.0)


class TestEventFromDict:
    def test_state_event(self) -> None:
        d = {
            "type": "state",
            "t_ms": 100,
            "state": "MOVING",
            "pose": "STANDING",
            "human": 1,
            "n_targets": 1,
            "dist_cm": 80.0,
            "head_moving": 0,
            "dist_new": 1,
        }
        ev = event_from_dict(d, seq=5)
        assert ev.event_type == "state"
        assert isinstance(ev.data, SensorSnapshot)
        assert ev.data.state == "MOVING"

    def test_bio_event(self) -> None:
        d = {
            "type": "bio",
            "t_ms": 200,
            "hr": 83.0,
            "br": 6.0,
            "allowed": 1,
            "valid": 1,
            "hr_new": 1,
            "br_new": 1,
        }
        ev = event_from_dict(d, seq=6)
        assert isinstance(ev.data, BioReading)
        assert ev.data.hr == 83.0

    def test_light_event(self) -> None:
        d = {"type": "light", "t_ms": 300, "lux": 21.7, "valid": 1}
        ev = event_from_dict(d, seq=7)
        assert isinstance(ev.data, LightReading)

    def test_targets_event(self) -> None:
        d = {
            "type": "targets",
            "t_ms": 400,
            "n": 1,
            "n_targets": 1,
            "forced_focus": 0,
            "focus": {"cluster": 1, "x": 0.1, "y": 0.4, "r": 0.41, "bearing": 14.0, "v": 0.0},
            "targets": [{"cluster": 1, "x": 0.1, "y": 0.4, "r": 0.41, "bearing": 14.0, "v": 0.0}],
            "targets_truncated": False,
        }
        ev = event_from_dict(d, seq=8)
        assert isinstance(ev.data, TargetsEvent)
        assert ev.data.focus is not None
        assert ev.data.focus.velocity == 0.0  # mapped from "v"

    def test_targets_no_focus(self) -> None:
        d = {
            "type": "targets",
            "t_ms": 400,
            "n": 0,
            "n_targets": 0,
            "forced_focus": 0,
            "focus": None,
            "targets": [],
            "targets_truncated": False,
        }
        ev = event_from_dict(d, seq=9)
        assert isinstance(ev.data, TargetsEvent)
        assert ev.data.focus is None

    def test_diag_event(self) -> None:
        d = {
            "type": "diag",
            "t_ms": 500,
            "mmwave_fail_count": 3,
            "mmwave_consecutive_fails": 1,
            "tx_drop_count": 0,
        }
        ev = event_from_dict(d, seq=10)
        assert isinstance(ev.data, DiagCounters)

    def test_unknown_event_passes_through(self) -> None:
        d = {"type": "pong", "t_ms": 600}
        ev = event_from_dict(d, seq=11)
        assert ev.event_type == "pong"
        assert isinstance(ev.data, dict)


class TestReadEvents:
    @pytest.mark.asyncio
    async def test_read_events_yields_decoded_events(self) -> None:
        """read_events wraps serial decode loop into async DecodedEvent stream."""
        fake_events: list[dict] = [  # type: ignore[type-arg]
            {
                "type": "state",
                "t_ms": 100,
                "state": "MOVING",
                "pose": "STANDING",
                "human": 1,
                "n_targets": 1,
                "dist_cm": 80.0,
                "head_moving": 0,
                "dist_new": 1,
            },
            {
                "type": "bio",
                "t_ms": 200,
                "hr": 83.0,
                "br": 6.0,
                "allowed": 1,
                "valid": 1,
                "hr_new": 1,
                "br_new": 1,
            },
        ]
        results = []
        async for ev in read_events(port=None, baud=115200, _test_events=fake_events):
            results.append(ev)
        assert len(results) == 2
        assert results[0].event_type == "state"
        assert results[1].event_type == "bio"
