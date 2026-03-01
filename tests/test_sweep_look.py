"""Tests for SweepLook tool error handling and named constants."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
from unittest.mock import MagicMock

import pytest

from healthy_heartrate_breathing.tools.core_tools import ToolDependencies


def _deps() -> ToolDependencies:
    mm = MagicMock()
    return ToolDependencies(reachy_mini=MagicMock(), movement_manager=mm)


class TestSweepLookConstants:
    def test_named_constants_exist(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.sweep_look import (
            SWEEP_HOLD_S,
            SWEEP_TRANSITION_S,
            SWEEP_MAX_ANGLE_RAD,
        )

        assert SWEEP_MAX_ANGLE_RAD > 0
        assert SWEEP_TRANSITION_S > 0
        assert SWEEP_HOLD_S > 0


class TestSweepLookErrorHandling:
    @pytest.mark.asyncio
    async def test_robot_error_returns_error_dict(self) -> None:
        """Robot operation raises RuntimeError: returns error dict, doesn't crash."""
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.sweep_look import (
            SweepLook,
        )

        tool = SweepLook()
        deps = _deps()
        deps.reachy_mini.get_current_head_pose.side_effect = RuntimeError("USB disconnected")

        result = await tool(deps)
        assert "error" in result
        assert "USB disconnected" in result["error"]

    @pytest.mark.asyncio
    async def test_normal_execution_returns_success(self) -> None:
        """Normal execution returns success dict with duration."""
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.sweep_look import (
            SweepLook,
        )

        tool = SweepLook()
        deps = _deps()
        deps.reachy_mini.get_current_head_pose.return_value = MagicMock()
        deps.reachy_mini.get_current_joint_positions.return_value = (
            [0.0] * 7,
            [0.0, 0.0],
        )

        result = await tool(deps)
        assert "status" in result
        assert "sweep" in result["status"].lower()
