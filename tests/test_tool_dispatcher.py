"""Tests for ToolDispatcher tool call pipeline."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
import time
import asyncio
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
        "timeout_s": 5.0,
    }
    defaults.update(overrides)
    return ToolDispatcher(**defaults)


async def _dispatch_and_wait(
    d: ToolDispatcher,
    *,
    tool_name: str,
    args_json: str,
    call_id: str | None,
    is_idle: bool,
) -> None:
    """Dispatch a tool and wait for it to complete."""
    d.dispatch(tool_name=tool_name, args_json=args_json, call_id=call_id, is_idle=is_idle)
    await asyncio.sleep(0.05)


class TestNonIdleDispatch:
    @pytest.mark.asyncio
    async def test_dispatches_tool_and_sends_result(self, tmp_path) -> None:
        dispatch = AsyncMock(return_value={"answer": 42})
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=False)
        dispatch.assert_called_once()
        send_result.assert_called_once_with("call-1", json.dumps({"answer": 42}))

    @pytest.mark.asyncio
    async def test_creates_response_for_non_idle(self, tmp_path) -> None:
        create_resp = AsyncMock()
        d = _dispatcher(tmp_path, create_response=create_resp)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=False)
        create_resp.assert_called_once()

    @pytest.mark.asyncio
    async def test_tool_failure_returns_error(self, tmp_path) -> None:
        dispatch = AsyncMock(side_effect=RuntimeError("boom"))
        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
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

        await _dispatch_and_wait(d, tool_name="dance", args_json="{}", call_id="call-1", is_idle=True)
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_idle_mmwave_overrides_args(self, tmp_path) -> None:
        dispatch = AsyncMock(return_value={"scan": {"latest_target": None}})
        policy = _idle_policy(probe_duration_s=5.0)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=True)
        actual_args = json.loads(dispatch.call_args[0][1])
        assert actual_args["mode"] == "locate_and_measure"
        assert actual_args["duration_s"] == 5.0

    @pytest.mark.asyncio
    async def test_idle_does_not_create_response(self, tmp_path) -> None:
        create_resp = AsyncMock()
        d = _dispatcher(tmp_path, create_response=create_resp)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        create_resp.assert_not_called()


class TestIdlePolicyIntegration:
    @pytest.mark.asyncio
    async def test_target_found_resets_misses(self, tmp_path) -> None:
        policy = _idle_policy()
        policy.consecutive_misses = 5
        result = {"scan": {"latest_target": {"x": 1.0, "y": 2.0}}}
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_no_target_increments_misses(self, tmp_path) -> None:
        policy = _idle_policy()
        policy.consecutive_misses = 0
        result = {"scan": {"latest_target": None, "targets_seen": 0}}
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        assert policy.consecutive_misses == 1


class TestIdleErrorTracking:
    @pytest.mark.asyncio
    async def test_idle_mmwave_error_calls_record_error(self, tmp_path) -> None:
        """When idle mmWave returns an error, record_error() is called on the policy."""
        policy = _idle_policy()
        result = {"error": "serial error on /dev/ttyUSB0: device disconnected", "status": "disconnected"}
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-e1", is_idle=True)
        assert policy.consecutive_errors == 1

    @pytest.mark.asyncio
    async def test_repeated_errors_suppress_probing(self, tmp_path) -> None:
        """After errors_before_suppression consecutive errors, probing is suppressed."""
        policy = _idle_policy(errors_before_suppression=2, error_backoff_s=60.0)
        error_result = {"error": "device disconnected", "status": "disconnected"}
        dispatch = AsyncMock(return_value=error_result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        # Two consecutive errors
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=True)
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=True)
        assert policy.consecutive_errors == 2
        assert policy.sensor_suppressed is True

    @pytest.mark.asyncio
    async def test_successful_result_after_errors_resets(self, tmp_path) -> None:
        """A successful result after errors resets the error counter."""
        policy = _idle_policy(errors_before_suppression=2)
        dispatch = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        # Two errors
        dispatch.return_value = {"error": "disconnected", "status": "disconnected"}
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=True)
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=True)
        assert policy.consecutive_errors == 2

        # Successful result with target
        dispatch.return_value = {"scan": {"latest_target": {"x": 1.0, "y": 2.0}}}
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c3", is_idle=True)
        assert policy.consecutive_errors == 0
        assert policy.sensor_suppressed is False


def _replace_sensor_state(target: dict) -> object:
    """Create a callback that replaces (clear+update) like the real handler."""

    def _cb(state: dict) -> None:
        target.clear()
        target.update(state)

    return _cb


class TestSensorStateOnError:
    @pytest.mark.asyncio
    async def test_error_result_updates_sensor_state(self, tmp_path) -> None:
        """MmWave error results should propagate to sensor_state for dashboard."""
        sensor_state: dict = {}
        error_result = {"error": "serial error on /dev/ttyUSB0", "status": "disconnected"}
        dispatch = AsyncMock(return_value=error_result)
        d = _dispatcher(
            tmp_path,
            dispatch_tool=dispatch,
            on_sensor_update=_replace_sensor_state(sensor_state),
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)
        assert sensor_state.get("error") is not None
        assert sensor_state.get("status") == "disconnected"
        assert "updated_at" in sensor_state

    @pytest.mark.asyncio
    async def test_successful_result_clears_error_in_sensor_state(self, tmp_path) -> None:
        """After an error, a successful result should produce state without error."""
        sensor_state: dict = {}
        dispatch = AsyncMock()
        d = _dispatcher(
            tmp_path,
            dispatch_tool=dispatch,
            on_sensor_update=_replace_sensor_state(sensor_state),
        )

        # First: error
        dispatch.return_value = {"error": "disconnected", "status": "disconnected"}
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)
        assert sensor_state.get("status") == "disconnected"

        # Second: success
        dispatch.return_value = {"scan": {"latest_target": None, "targets_seen": 0}, "status": "scan_done"}
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=False)
        assert sensor_state.get("error") is None
        assert sensor_state.get("status") == "scan_done"


class TestHeadWobblerReset:
    @pytest.mark.asyncio
    async def test_resets_wobbler_after_tool_call(self, tmp_path) -> None:
        reset = MagicMock()
        d = _dispatcher(tmp_path, head_wobbler_reset=reset)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        reset.assert_called_once()


class TestTimeout:
    @pytest.mark.asyncio
    async def test_tool_timeout_returns_error(self, tmp_path) -> None:
        """A tool that exceeds timeout_s produces an error result."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(10)
            return {"status": "ok"}

        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=slow_tool, send_tool_result=send_result, timeout_s=0.1)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-t", is_idle=False)
        await asyncio.sleep(0.3)

        call_args = send_result.call_args
        result = json.loads(call_args[0][1])
        assert "error" in result
        assert "timed out" in result["error"]

    @pytest.mark.asyncio
    async def test_cancel_stops_running_tool(self, tmp_path) -> None:
        """cancel() terminates an in-flight tool task."""
        started = asyncio.Event()

        async def blocking_tool(name: str, args: str) -> dict:
            started.set()
            await asyncio.sleep(60)
            return {"status": "ok"}

        send_result = AsyncMock()
        d = _dispatcher(tmp_path, dispatch_tool=blocking_tool, send_tool_result=send_result, timeout_s=60)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-c", is_idle=False)
        await started.wait()

        await d.cancel()

        assert d._active_task is None
        send_result.assert_not_called()


class TestSerialization:
    @pytest.mark.asyncio
    async def test_tools_run_one_at_a_time(self, tmp_path) -> None:
        """Semaphore ensures only one tool runs at a time."""
        concurrency = 0
        max_concurrency = 0

        async def tracking_tool(name: str, args: str) -> dict:
            nonlocal concurrency, max_concurrency
            concurrency += 1
            max_concurrency = max(max_concurrency, concurrency)
            await asyncio.sleep(0.05)
            concurrency -= 1
            return {"status": "ok"}

        d = _dispatcher(tmp_path, dispatch_tool=tracking_tool, timeout_s=5)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-2", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-3", is_idle=False)
        await asyncio.sleep(0.3)

        assert max_concurrency == 1


class TestNonBlocking:
    @pytest.mark.asyncio
    async def test_dispatch_returns_immediately(self, tmp_path) -> None:
        """dispatch() must not block — returns before tool completes."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(5)
            return {"status": "ok"}

        d = _dispatcher(tmp_path, dispatch_tool=slow_tool, timeout_s=10)

        t0 = time.monotonic()
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-nb", is_idle=False)
        elapsed = time.monotonic() - t0

        assert elapsed < 0.05, f"dispatch() blocked for {elapsed:.3f}s"
        await d.cancel()


class TestMultiTargetRouting:
    @pytest.mark.asyncio
    async def test_multi_target_calls_record_multi_target(self, tmp_path) -> None:
        """When max_target_count > 1, record_multi_target is called instead of record_target_found."""
        policy = _idle_policy()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 3,
                "targets_seen": 2,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-mt", is_idle=True)
        assert policy.last_multi_target_time is not None
        assert policy.last_focus_time is None  # NOT set for multi-target
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_single_target_still_calls_record_target_found(self, tmp_path) -> None:
        """When max_target_count == 1, record_target_found is still called."""
        policy = _idle_policy()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 1,
                "targets_seen": 1,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = _dispatcher(tmp_path, dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-st", is_idle=True)
        assert policy.last_focus_time is not None
        assert policy.last_multi_target_time is None
