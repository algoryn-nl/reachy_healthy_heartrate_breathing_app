"""Tests for LightOrchestrator baseline tracking and dispatch."""
# ruff: noqa: D101, D102, D103

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
