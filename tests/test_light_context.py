"""Tests for the light_context classification logic."""
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
    result = await tool(_deps(), lux=100.0, local_hour=12)

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
# Night wind-down
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_night_low_lux_returns_night_wind_down(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,  # below default threshold of 40
        local_hour=22,  # after default night_start_hour=20
        presence_detected=True,
    )

    assert result["context_state"] == "night_wind_down"
    assert result["recommended_mode"] == "quiet"
    assert "night_low_lux" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Unexpected darkening (sharp drop)
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_sharp_lux_drop_returns_unexpected_darkening(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=30.0,
        lux_delta_60s=-100.0,  # exceeds default sharp_drop_lux=80
        local_hour=14,
        presence_detected=True,
        active_interaction=True,
    )

    assert result["context_state"] == "unexpected_darkening"
    assert result["recommended_mode"] == "quiet"
    assert "sharp_lux_drop" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Intentional dim work
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_prefers_dim_returns_intentional_dim(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,
        local_hour=12,  # daytime
        prefers_dim=True,
    )

    assert result["context_state"] == "intentional_dim_work"
    assert "prefers_dim" in result["reason_codes"]
    assert "intentional_dim" in result["reason_codes"]


@pytest.mark.asyncio
async def test_light_sensitive_returns_intentional_dim_quiet(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,
        local_hour=12,
        light_sensitive=True,
    )

    assert result["context_state"] == "intentional_dim_work"
    assert result["recommended_mode"] == "quiet"
    assert "light_sensitive" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Prolonged low light strain risk
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_prolonged_low_light_returns_strain_risk(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=20.0,
        local_hour=12,
        allow_wellness_nudges=True,
        low_light_duration_min=50.0,  # above default 45 min
    )

    assert result["context_state"] == "prolonged_low_light_strain_risk"
    assert "prolonged_daytime_low_lux" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Bright active
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_bright_daytime_returns_bright_active(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=300.0,  # above default bright threshold of 250
        local_hour=10,
        presence_detected=True,
    )

    assert result["context_state"] == "bright_active"
    assert result["recommended_mode"] == "active"
    assert "bright_daytime" in result["reason_codes"]


# ---------------------------------------------------------------------------
# Neutral conditions
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_neutral_moderate_lux_returns_neutral(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("HEALTHY_LIGHT_CONTEXT_ENABLED", raising=False)
    tool = LightContext()
    result = await tool(
        _deps(),
        lux=100.0,  # between 40 (low) and 250 (bright)
        local_hour=12,
    )

    assert result["context_state"] == "neutral"
    assert "neutral_conditions" in result["reason_codes"]


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
        local_hour=10,
        presence_detected=True,
    )

    # Should have extracted lux=300 and classified as bright_active
    assert result["context_state"] == "bright_active"
    assert result["observations"]["lux"] == 300.0
