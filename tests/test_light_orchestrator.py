"""Tests for LightOrchestrator baseline tracking and dispatch."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
from pathlib import Path
from datetime import datetime, timezone, timedelta
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
        "baseline_save_interval_s": 0,  # flush immediately in tests
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
        # With 1 sample, min_samples=5, so exact hour falls back to avg
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


class TestBaselineThrottling:
    def test_save_throttled_within_interval(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=300)
        o.update_baseline(lux=100.0, local_hour=12)
        # First save goes through (last_save_time starts at 0)
        assert o.baseline_path.exists()
        first_content = o.baseline_path.read_text()

        # Second update within interval: save_baseline is a no-op on disk
        o.update_baseline(lux=200.0, local_hour=12)
        assert o.baseline_path.read_text() == first_content  # unchanged on disk
        assert o._baseline_dirty is True  # but marked dirty in memory

    def test_flush_writes_dirty_state(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=300)
        o.update_baseline(lux=100.0, local_hour=12)
        first_content = o.baseline_path.read_text()

        o.update_baseline(lux=200.0, local_hour=12)
        assert o._baseline_dirty is True
        o.flush()
        assert o._baseline_dirty is False
        assert o.baseline_path.read_text() != first_content  # updated

    def test_flush_noop_when_clean(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=0)
        o.update_baseline(lux=100.0, local_hour=12)
        assert o._baseline_dirty is False  # interval=0 means save_baseline wrote immediately
        o.baseline_path.unlink()  # remove file
        o.flush()  # should not recreate it
        assert not o.baseline_path.exists()


class TestBaselinePruning:
    def test_stale_user_pruned_on_load(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=100)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "old-user": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 10}},
                    "updated_at": stale_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "old-user" not in o._baseline_state["users"]

    def test_fresh_user_kept_on_load(self, tmp_path: Path) -> None:
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=10)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "recent-user": {
                    "hours": {"12": {"ema_lux": 200.0, "samples": 5}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "recent-user" in o._baseline_state["users"]

    def test_mixed_stale_and_fresh(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=200)).isoformat()
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=5)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "stale": {
                    "hours": {"10": {"ema_lux": 50.0, "samples": 3}},
                    "updated_at": stale_ts,
                },
                "fresh": {
                    "hours": {"14": {"ema_lux": 300.0, "samples": 8}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "stale" not in o._baseline_state["users"]
        assert "fresh" in o._baseline_state["users"]

    def test_malformed_timestamp_pruned(self, tmp_path: Path) -> None:
        state = {
            "schema_version": 1,
            "users": {
                "bad-ts": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 1}},
                    "updated_at": "not-a-date",
                },
                "no-ts": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 1}},
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "bad-ts" not in o._baseline_state["users"]
        assert "no-ts" not in o._baseline_state["users"]

    def test_pruning_persists_to_disk(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=100)).isoformat()
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=1)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "stale": {
                    "hours": {"12": {"ema_lux": 50.0, "samples": 3}},
                    "updated_at": stale_ts,
                },
                "fresh": {
                    "hours": {"14": {"ema_lux": 300.0, "samples": 8}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        _orchestrator(tmp_path, baseline_max_age_days=90)
        # Pruned state should be saved to disk
        reloaded = json.loads(baseline_path.read_text())
        assert "stale" not in reloaded["users"]
        assert "fresh" in reloaded["users"]

    def test_no_pruning_when_all_fresh(self, tmp_path: Path) -> None:
        fresh_ts = datetime.now(timezone.utc).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "user-a": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 5}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90, baseline_save_interval_s=300)
        assert "user-a" in o._baseline_state["users"]
        # No pruning happened, so dirty flag should not be set
        assert o._baseline_dirty is False


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
