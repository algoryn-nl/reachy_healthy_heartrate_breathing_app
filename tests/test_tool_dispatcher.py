"""Tests for ToolDispatcher tool call pipeline."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
from unittest.mock import AsyncMock, MagicMock

import pytest

from healthy_heartrate_breathing.idle_policy import IdlePolicy
from healthy_heartrate_breathing.tool_dispatcher import ToolDispatcher
from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator


def _idle_policy(**overrides: object) -> IdlePolicy:
    defaults = {
        "interval_s": 15.0,
        "probe_interval_s": 40.0,
        "probe_duration_s": 5.0,
        "misses_before_sweep": 3,
        "sweep_cooldown_s": 150.0,
        "post_focus_quiet_s": 45.0,
    }
    defaults.update(overrides)
    return IdlePolicy(**defaults)


def _light_orchestrator(tmp_path) -> LightOrchestrator:
    return LightOrchestrator(
        enabled=False,
        analytics_enabled=False,
        user_id="test",
        baseline_path=tmp_path / "baseline.json",
        analytics_path=tmp_path / "analytics.jsonl",
    )


def _dispatcher(tmp_path, **overrides: object) -> ToolDispatcher:
    defaults = {
        "idle_policy": _idle_policy(),
        "light_orchestrator": _light_orchestrator(tmp_path),
        "has_tool": lambda name: name == "mmWave",
        "dispatch_tool": AsyncMock(return_value={"status": "ok"}),
        "send_tool_result": AsyncMock(),
        "create_response": AsyncMock(),
        "create_message": AsyncMock(),
        "enqueue_output": AsyncMock(),
        "get_camera_frame": lambda: None,
        "head_wobbler_reset": None,
    }
    defaults.update(overrides)
    return ToolDispatcher(**defaults)


class TestNonIdleDispatch:
    @pytest.mark.asyncio
    async def test_dispatches_tool_and_sends_result(self, tmp_path) -> None:
        dispatch = AsyncMock(return_value={"answer": 42})
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        consumed = await d.on_tool_call_done(
            tool_name="mmWave",
            args_json='{"mode":"scan"}',
            call_id="call-1",
            is_idle=False,
        )
        dispatch.assert_called_once()
        send_result.assert_called_once_with("call-1", json.dumps({"answer": 42}))
        assert consumed is False  # not idle, so flag not consumed

    @pytest.mark.asyncio
    async def test_creates_response_for_non_idle(self, tmp_path) -> None:
        create_resp = AsyncMock()
        d = _dispatcher(tmp_path, create_response=create_resp)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json='{"mode":"scan"}',
            call_id="call-1",
            is_idle=False,
        )
        create_resp.assert_called_once()

    @pytest.mark.asyncio
    async def test_tool_failure_returns_error(self, tmp_path) -> None:
        dispatch = AsyncMock(side_effect=RuntimeError("boom"))
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=False,
        )
        call_args = send_result.call_args
        result = json.loads(call_args[0][1])
        assert "error" in result


class TestIdleDispatch:
    @pytest.mark.asyncio
    async def test_idle_suppresses_non_mmwave_tool(self, tmp_path) -> None:
        dispatch = AsyncMock()
        enqueue = AsyncMock()
        d = _dispatcher(
            tmp_path,
            dispatch_tool=dispatch,
            enqueue_output=enqueue,
            has_tool=lambda name: name == "mmWave",
        )

        consumed = await d.on_tool_call_done(
            tool_name="dance",
            args_json="{}",
            call_id="call-1",
            is_idle=True,
        )
        dispatch.assert_not_called()
        assert consumed is True

    @pytest.mark.asyncio
    async def test_idle_mmwave_overrides_args(self, tmp_path) -> None:
        dispatch = AsyncMock(return_value={"scan": {"latest_target": None}})
        policy = _idle_policy(probe_duration_s=5.0)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json='{"mode":"scan"}',
            call_id="call-1",
            is_idle=True,
        )
        actual_args = json.loads(dispatch.call_args[0][1])
        assert actual_args["mode"] == "locate_and_measure"
        assert actual_args["duration_s"] == 5.0

    @pytest.mark.asyncio
    async def test_idle_does_not_create_response(self, tmp_path) -> None:
        create_resp = AsyncMock()
        d = _dispatcher(tmp_path, create_response=create_resp)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=True,
        )
        create_resp.assert_not_called()

    @pytest.mark.asyncio
    async def test_idle_consumed_flag(self, tmp_path) -> None:
        d = _dispatcher(tmp_path)
        consumed = await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=True,
        )
        assert consumed is True


class TestIdlePolicyIntegration:
    @pytest.mark.asyncio
    async def test_target_found_resets_misses(self, tmp_path) -> None:
        policy = _idle_policy()
        policy.consecutive_misses = 5
        result = {"scan": {"latest_target": {"x": 1.0, "y": 2.0}}}
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=True,
        )
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_no_target_increments_misses(self, tmp_path) -> None:
        policy = _idle_policy()
        policy.consecutive_misses = 0
        result = {"scan": {"latest_target": None, "targets_seen": 0}}
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=True,
        )
        assert policy.consecutive_misses == 1


class TestHeadWobblerReset:
    @pytest.mark.asyncio
    async def test_resets_wobbler_after_tool_call(self, tmp_path) -> None:
        reset = MagicMock()
        d = _dispatcher(tmp_path, head_wobbler_reset=reset)

        await d.on_tool_call_done(
            tool_name="mmWave",
            args_json="{}",
            call_id="call-1",
            is_idle=False,
        )
        reset.assert_called_once()
