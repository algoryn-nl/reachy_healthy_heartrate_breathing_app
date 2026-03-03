"""Tests for the light_context proximity/occlusion classification logic."""
# ruff: noqa: D103

from __future__ import annotations
from types import SimpleNamespace

import pytest

from healthy_heartrate_breathing.tools.core_tools import ToolDependencies
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.light_context import LightContext


def _deps() -> ToolDependencies:
    return ToolDependencies(
        reachy_mini=SimpleNamespace(),
        movement_manager=None,
    )


# ---------------------------------------------------------------------------
# Tool enabled/disabled
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_disabled_returns_neutral(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("HEALTHY_LIGHT_CONTEXT_ENABLED", "false")
    tool = LightContext()
    result = await tool(_deps(), lux=10.0)

    assert result["enabled"] is False
    assert result["context_state"] == "neutral"
    assert "tool_disabled" in result["reason_codes"]


@pytest.mark.asyncio
async def test_enabled_by_default(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(_deps(), lux=100.0)

    assert result["enabled"] is True


# ---------------------------------------------------------------------------
# No lux data
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_missing_lux_returns_lux_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(_deps())

    assert result["context_state"] == "neutral"
    assert "lux_missing" in result["reason_codes"]


# ---------------------------------------------------------------------------
# close_presence: low lux + presence
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_low_lux_with_presence_returns_close_presence(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,  # below default threshold of 40
        presence_detected=True,
    )

    assert result["context_state"] == "close_presence"
    assert result["recommended_mode"] == "engaged"
    assert "low_lux_with_presence" in result["reason_codes"]


@pytest.mark.asyncio
async def test_low_lux_without_presence_returns_clear_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,  # low lux but no presence
        presence_detected=False,
    )

    assert result["context_state"] == "clear_path"
    assert "no_presence" in result["reason_codes"]


# ---------------------------------------------------------------------------
# sudden_occlusion: sharp lux drop + presence
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_sharp_drop_with_presence_returns_sudden_occlusion(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=30.0,
        lux_delta_60s=-80.0,  # exceeds default sharp_drop_lux=60
        presence_detected=True,
    )

    assert result["context_state"] == "sudden_occlusion"
    assert result["recommended_mode"] == "engaged"
    assert "sharp_lux_drop" in result["reason_codes"]


# ---------------------------------------------------------------------------
# partial_occlusion: moderate lux + presence
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_moderate_lux_with_presence_returns_partial_occlusion(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=80.0,  # between 40 (low) and 120 (moderate)
        presence_detected=True,
    )

    assert result["context_state"] == "partial_occlusion"
    assert result["recommended_mode"] == "balanced"
    assert "moderate_lux_with_presence" in result["reason_codes"]


# ---------------------------------------------------------------------------
# clear_path: high lux or no presence
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_high_lux_returns_clear_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=300.0,  # well above moderate threshold
        presence_detected=True,
    )

    assert result["context_state"] == "clear_path"
    assert result["recommended_mode"] == "balanced"
    assert "high_lux" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Lux extraction from mmwave_result kwarg
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_lux_extracted_from_mmwave_result(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        mmwave_result={"measure": {"latest_light": {"lux": 300.0}}},
        presence_detected=True,
    )

    # Should have extracted lux=300 and classified as clear_path (high lux)
    assert result["context_state"] == "clear_path"
    assert result["observations"]["lux"] == 300.0


# ---------------------------------------------------------------------------
# Falsy-zero regression (PY-CRIT-3)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_zero_low_lux_threshold_respected(monkeypatch: pytest.MonkeyPatch) -> None:
    """Threshold of 0.0 must not silently become the default (40.0)."""
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=10.0,
        low_lux_threshold=0.0,
        presence_detected=True,
    )
    # With threshold=0.0, lux=10 is NOT low (10 > 0), so partial_occlusion or clear_path
    assert result["context_state"] != "close_presence"


@pytest.mark.asyncio
async def test_zero_sharp_drop_threshold_respected(monkeypatch: pytest.MonkeyPatch) -> None:
    """Threshold of 0.0 means any negative delta triggers sharp drop."""
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=100.0,
        lux_delta_60s=-1.0,
        sharp_drop_lux=0.0,
        presence_detected=True,
    )
    # With threshold=0.0, any negative delta triggers sudden_occlusion
    assert result["context_state"] == "sudden_occlusion"


# ---------------------------------------------------------------------------
# Observations include new fields
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_observations_include_target_distance(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,
        presence_detected=True,
        target_distance_cm=45.0,
    )

    assert result["observations"]["target_distance_cm"] == 45.0
    assert result["policy_version"] == "proximity_context_v1"
