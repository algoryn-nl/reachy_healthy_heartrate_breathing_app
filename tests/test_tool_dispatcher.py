"""Tests for ToolDispatcher tool call pipeline."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
import time
import asyncio
from typing import Any
from pathlib import Path
from unittest.mock import Mock, AsyncMock, MagicMock
from collections.abc import Callable

import pytest

from healthy_heartrate_breathing.idle_policy import IdlePolicy
from healthy_heartrate_breathing.tool_dispatcher import ToolDispatcher, build_device_context, extract_sensor_state
from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator


@pytest.fixture
def idle_policy_factory() -> Callable[..., IdlePolicy]:
    """Return a factory that builds an IdlePolicy with sane defaults."""

    def _make(**overrides: Any) -> IdlePolicy:
        defaults: dict[str, Any] = {
            "interval_s": 15.0,
            "probe_interval_s": 40.0,
            "probe_duration_s": 5.0,
            "misses_before_sweep": 3,
            "sweep_cooldown_s": 150.0,
            "post_focus_quiet_s": 45.0,
        }
        defaults.update(overrides)
        return IdlePolicy(**defaults)

    return _make


@pytest.fixture
def light_orchestrator_factory(tmp_path: Path) -> Callable[..., LightOrchestrator]:
    """Return a factory that builds a LightOrchestrator with sane defaults."""

    def _make(**overrides: Any) -> LightOrchestrator:
        defaults: dict[str, Any] = {
            "enabled": False,
            "analytics_enabled": False,
            "user_id": "test",
            "analytics_path": tmp_path / "analytics.db",
        }
        defaults.update(overrides)
        return LightOrchestrator(**defaults)

    return _make


@pytest.fixture
def dispatcher_factory(
    tmp_path: Path,
    idle_policy_factory: Callable[..., IdlePolicy],
    light_orchestrator_factory: Callable[..., LightOrchestrator],
) -> Callable[..., ToolDispatcher]:
    """Return a factory that builds a ToolDispatcher with sane defaults."""

    def _make(**overrides: Any) -> ToolDispatcher:
        defaults: dict[str, Any] = {
            "idle_policy": idle_policy_factory(),
            "light_orchestrator": light_orchestrator_factory(),
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

    return _make


async def _dispatch_and_wait(
    d: ToolDispatcher,
    *,
    tool_name: str,
    args_json: str,
    call_id: str | None,
    is_idle: bool,
) -> None:
    """Dispatch a tool and yield until the background task completes.

    The mock tools used in tests complete instantly, so repeated event-loop
    yields are sufficient for the fire-and-forget task to finish.  This avoids
    wall-clock sleeps that make tests flaky on slow CI.
    """
    d.dispatch(tool_name=tool_name, args_json=args_json, call_id=call_id, is_idle=is_idle)
    for _ in range(50):
        await asyncio.sleep(0)


class TestNonIdleDispatch:
    @pytest.mark.asyncio
    async def test_dispatches_tool_and_sends_result(self, dispatcher_factory) -> None:
        dispatch = AsyncMock(return_value={"answer": 42})
        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=False)
        dispatch.assert_called_once()
        send_result.assert_called_once_with("call-1", json.dumps({"answer": 42}))

    @pytest.mark.asyncio
    async def test_creates_response_for_non_idle(self, dispatcher_factory) -> None:
        create_resp = AsyncMock()
        d = dispatcher_factory(create_response=create_resp)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=False)
        create_resp.assert_called_once()

    @pytest.mark.asyncio
    async def test_tool_failure_returns_error(self, dispatcher_factory) -> None:
        dispatch = AsyncMock(side_effect=RuntimeError("boom"))
        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        call_args = send_result.call_args
        result = json.loads(call_args[0][1])
        assert "error" in result


class TestIdleDispatch:
    @pytest.mark.asyncio
    async def test_idle_suppresses_non_mmwave_tool(self, dispatcher_factory) -> None:
        dispatch = AsyncMock()
        enqueue = AsyncMock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            enqueue_output=enqueue,
            has_tool=lambda name: name == "mmWave",
        )

        await _dispatch_and_wait(d, tool_name="dance", args_json="{}", call_id="call-1", is_idle=True)
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_idle_mmwave_overrides_args(self, dispatcher_factory, idle_policy_factory) -> None:
        dispatch = AsyncMock(return_value={"scan": {"latest_target": None}})
        policy = idle_policy_factory(probe_duration_s=5.0)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-1", is_idle=True)
        actual_args = json.loads(dispatch.call_args[0][1])
        assert actual_args["mode"] == "locate_and_measure"
        assert actual_args["duration_s"] == 5.0

    @pytest.mark.asyncio
    async def test_idle_silent_when_unremarkable(self, dispatcher_factory) -> None:
        create_resp = AsyncMock()
        # No vitals, no state change → should stay silent
        dispatch = AsyncMock(return_value={"status": "measure_inconclusive", "scan": {"latest_target": None}})
        d = dispatcher_factory(create_response=create_resp, dispatch_tool=dispatch)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        create_resp.assert_not_called()

    @pytest.mark.asyncio
    async def test_idle_speaks_when_noteworthy(self, dispatcher_factory) -> None:
        create_resp = AsyncMock()
        # status="ok" means vitals were captured → noteworthy
        dispatch = AsyncMock(return_value={"status": "ok", "scan": {"latest_target": {"x": 1.0}}})
        d = dispatcher_factory(create_response=create_resp, dispatch_tool=dispatch)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        create_resp.assert_called_once()


class TestIdlePolicyIntegration:
    @pytest.mark.asyncio
    async def test_target_found_resets_misses(self, dispatcher_factory, idle_policy_factory) -> None:
        policy = idle_policy_factory()
        policy.consecutive_misses = 5
        result = {"scan": {"latest_target": {"x": 1.0, "y": 2.0}}}
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_no_target_increments_misses(self, dispatcher_factory, idle_policy_factory) -> None:
        policy = idle_policy_factory()
        policy.consecutive_misses = 0
        result = {"scan": {"latest_target": None, "targets_seen": 0}}
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=True)
        assert policy.consecutive_misses == 1


class TestIdleErrorTracking:
    @pytest.mark.asyncio
    async def test_idle_mmwave_error_calls_record_error(self, dispatcher_factory, idle_policy_factory) -> None:
        """When idle mmWave returns an error, record_error() is called on the policy."""
        policy = idle_policy_factory()
        result = {"error": "serial error on /dev/ttyUSB0: device disconnected", "status": "disconnected"}
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-e1", is_idle=True)
        assert policy.consecutive_errors == 1

    @pytest.mark.asyncio
    async def test_repeated_errors_suppress_probing(self, dispatcher_factory, idle_policy_factory) -> None:
        """After errors_before_suppression consecutive errors, probing is suppressed."""
        policy = idle_policy_factory(errors_before_suppression=2, error_backoff_s=60.0)
        error_result = {"error": "device disconnected", "status": "disconnected"}
        dispatch = AsyncMock(return_value=error_result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        # Two consecutive errors
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=True)
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=True)
        assert policy.consecutive_errors == 2
        assert policy.sensor_suppressed is True

    @pytest.mark.asyncio
    async def test_successful_result_after_errors_resets(self, dispatcher_factory, idle_policy_factory) -> None:
        """A successful result after errors resets the error counter."""
        policy = idle_policy_factory(errors_before_suppression=2)
        dispatch = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

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
    async def test_error_result_updates_sensor_state(self, dispatcher_factory) -> None:
        """MmWave error results should propagate to sensor_state for dashboard."""
        sensor_state: dict = {}
        error_result = {"error": "serial error on /dev/ttyUSB0", "status": "disconnected"}
        dispatch = AsyncMock(return_value=error_result)
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            on_sensor_update=_replace_sensor_state(sensor_state),
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)
        assert sensor_state.get("error") is not None
        assert sensor_state.get("status") == "disconnected"
        assert "updated_at" in sensor_state

    @pytest.mark.asyncio
    async def test_successful_result_clears_error_in_sensor_state(self, dispatcher_factory) -> None:
        """After an error, a successful result should produce state without error."""
        sensor_state: dict = {}
        dispatch = AsyncMock()
        d = dispatcher_factory(
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
    async def test_resets_wobbler_after_tool_call(self, dispatcher_factory) -> None:
        reset = MagicMock()
        d = dispatcher_factory(head_wobbler_reset=reset)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        reset.assert_called_once()


class TestTimeout:
    @pytest.mark.asyncio
    async def test_tool_timeout_returns_error(self, dispatcher_factory) -> None:
        """A tool that exceeds timeout_s produces an error result."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(10)
            return {"status": "ok"}

        result_sent = asyncio.Event()
        send_result = AsyncMock(side_effect=lambda *a, **kw: result_sent.set())
        d = dispatcher_factory(dispatch_tool=slow_tool, send_tool_result=send_result, timeout_s=0.1)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-t", is_idle=False)
        # Wait for the timeout to fire and send the error result.
        await asyncio.wait_for(result_sent.wait(), timeout=2.0)

        call_args = send_result.call_args
        result = json.loads(call_args[0][1])
        assert "error" in result
        assert "timed out" in result["error"]

    @pytest.mark.asyncio
    async def test_cancel_stops_running_tool(self, dispatcher_factory) -> None:
        """cancel() terminates an in-flight tool task."""
        started = asyncio.Event()

        async def blocking_tool(name: str, args: str) -> dict:
            started.set()
            await asyncio.sleep(60)
            return {"status": "ok"}

        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=blocking_tool, send_tool_result=send_result, timeout_s=60)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-c", is_idle=False)
        await started.wait()

        await d.cancel()

        assert d._active_task is None
        send_result.assert_not_called()


class TestSerialization:
    @pytest.mark.asyncio
    async def test_tools_run_one_at_a_time(self, dispatcher_factory) -> None:
        """Semaphore ensures only one tool runs at a time."""
        concurrency = 0
        max_concurrency = 0
        gate = asyncio.Event()

        async def tracking_tool(name: str, args: str) -> dict:
            nonlocal concurrency, max_concurrency
            concurrency += 1
            max_concurrency = max(max_concurrency, concurrency)
            await gate.wait()  # deterministic: all three tools block until released
            concurrency -= 1
            return {"status": "ok"}

        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=tracking_tool, send_tool_result=send_result, timeout_s=5)

        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-1", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-2", is_idle=False)
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-3", is_idle=False)
        # Let the first tool acquire the semaphore
        await asyncio.sleep(0)
        # Release all tools
        gate.set()
        # Wait for all three dispatches to complete
        for _ in range(200):
            if send_result.call_count >= 3:
                break
            await asyncio.sleep(0)

        assert max_concurrency == 1


class TestNonBlocking:
    @pytest.mark.asyncio
    async def test_dispatch_returns_immediately(self, dispatcher_factory) -> None:
        """dispatch() must not block — returns before tool completes."""

        async def slow_tool(name: str, args: str) -> dict:
            await asyncio.sleep(5)
            return {"status": "ok"}

        d = dispatcher_factory(dispatch_tool=slow_tool, timeout_s=10)

        t0 = time.monotonic()
        d.dispatch(tool_name="mmWave", args_json="{}", call_id="call-nb", is_idle=False)
        elapsed = time.monotonic() - t0

        assert elapsed < 0.05, f"dispatch() blocked for {elapsed:.3f}s"
        await d.cancel()


class TestMultiTargetRouting:
    @pytest.mark.asyncio
    async def test_multi_target_calls_record_multi_target(self, dispatcher_factory, idle_policy_factory) -> None:
        """When max_target_count > 1, record_multi_target is called instead of record_target_found."""
        policy = idle_policy_factory()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 3,
                "targets_seen": 2,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-mt", is_idle=True)
        assert policy.last_multi_target_time is not None
        assert policy.last_focus_time is None  # NOT set for multi-target
        assert policy.consecutive_misses == 0

    @pytest.mark.asyncio
    async def test_single_target_still_calls_record_target_found(
        self, dispatcher_factory, idle_policy_factory
    ) -> None:
        """When max_target_count == 1, record_target_found is still called."""
        policy = idle_policy_factory()
        result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 1,
                "targets_seen": 1,
            },
        }
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, idle_policy=policy)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-st", is_idle=True)
        assert policy.last_focus_time is not None
        assert policy.last_multi_target_time is None


class TestBuildDeviceContext:
    def test_resting_vitals_returns_high_reliability(self) -> None:
        state = {"device_state": "RESTING_VITALS", "previous_device_state": "RESTING_VITALS"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["state"] == "RESTING_VITALS"
        assert ctx["vitals_reliability"] == "high"
        assert ctx["changed"] is False
        assert ctx["transition"] is None

    def test_still_near_returns_moderate_reliability(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": "STILL_NEAR"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "moderate"

    def test_moving_returns_low_reliability(self) -> None:
        state = {"device_state": "MOVING", "previous_device_state": "MOVING"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "low"

    def test_present_far_returns_unavailable(self) -> None:
        state = {"device_state": "PRESENT_FAR", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_multi_target_returns_unavailable(self) -> None:
        state = {"device_state": "MULTI_TARGET", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_no_target_returns_unavailable(self) -> None:
        state = {"device_state": "NO_TARGET", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unavailable"

    def test_unknown_state_returns_unknown_reliability(self) -> None:
        state = {"device_state": "UNKNOWN_7", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["vitals_reliability"] == "unknown"
        assert "UNKNOWN_7" in ctx["note"]

    def test_missing_device_state_returns_none(self) -> None:
        state = {"previous_device_state": None}
        assert build_device_context(state) is None

    def test_none_device_state_returns_none(self) -> None:
        state = {"device_state": None, "previous_device_state": None}
        assert build_device_context(state) is None

    def test_changed_true_when_state_differs(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": "MOVING"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert ctx["previous_state"] == "MOVING"
        assert ctx["transition"] is not None
        assert "MOVING" in ctx["transition"]
        assert "STILL_NEAR" in ctx["transition"]

    def test_first_call_previous_none_changed_false(self) -> None:
        state = {"device_state": "STILL_NEAR", "previous_device_state": None}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is False
        assert ctx["previous_state"] is None
        assert ctx["transition"] is None

    def test_transition_no_target_to_present(self) -> None:
        state = {"device_state": "PRESENT_FAR", "previous_device_state": "NO_TARGET"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert "arrived" in ctx["transition"].lower() or "just" in ctx["transition"].lower()

    def test_transition_to_no_target(self) -> None:
        state = {"device_state": "NO_TARGET", "previous_device_state": "STILL_NEAR"}
        ctx = build_device_context(state)
        assert ctx is not None
        assert ctx["changed"] is True
        assert "left" in ctx["transition"].lower()


class TestDeviceContextInjection:
    @pytest.mark.asyncio
    async def test_device_context_injected_into_mmwave_result(self, dispatcher_factory) -> None:
        """MmWave result should have device_context at top level after dispatch."""
        result = {
            "scan": {"device_state": "RESTING_VITALS", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-dc", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" in sent_json
        assert sent_json["device_context"]["state"] == "RESTING_VITALS"
        assert sent_json["device_context"]["vitals_reliability"] == "high"

    @pytest.mark.asyncio
    async def test_previous_state_tracked_across_calls(self, dispatcher_factory) -> None:
        """Second mmWave call should see previous_device_state from first call."""
        sensor_state: dict = {}
        dispatch = AsyncMock()
        send_result = AsyncMock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            on_sensor_update=_replace_sensor_state(sensor_state),
        )

        # First call: MOVING
        dispatch.return_value = {
            "scan": {"device_state": "MOVING", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)

        # Second call: STILL_NEAR
        dispatch.return_value = {
            "scan": {"device_state": "STILL_NEAR", "latest_target": {"x": 1.0}},
            "status": "scan_done",
        }
        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c2", is_idle=False)

        sent_json = json.loads(send_result.call_args[0][1])
        ctx = sent_json["device_context"]
        assert ctx["state"] == "STILL_NEAR"
        assert ctx["previous_state"] == "MOVING"
        assert ctx["changed"] is True

    @pytest.mark.asyncio
    async def test_device_context_not_injected_for_non_mmwave(self, dispatcher_factory) -> None:
        """Non-mmWave tools should not get device_context."""
        dispatch = AsyncMock(return_value={"status": "ok"})
        send_result = AsyncMock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            has_tool=lambda name: True,
        )

        await _dispatch_and_wait(d, tool_name="dance", args_json="{}", call_id="call-d", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" not in sent_json

    @pytest.mark.asyncio
    async def test_device_context_not_injected_on_error(self, dispatcher_factory) -> None:
        """When mmWave returns an error, device_context should not be injected."""
        result = {"error": "serial error", "status": "disconnected"}
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, send_tool_result=send_result)

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="call-e", is_idle=False)
        sent_json = json.loads(send_result.call_args[0][1])
        assert "device_context" not in sent_json


class TestAutoLightContextTimeout:
    @pytest.mark.asyncio
    async def test_light_context_timeout_does_not_block_result(
        self, dispatcher_factory, idle_policy_factory, light_orchestrator_factory
    ) -> None:
        """When auto light_context hangs, it times out and the mmWave result is still sent."""
        mmwave_result = {
            "scan": {
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.5},
                "light_summary": {"latest_lux": 100.0, "valid_samples": 1},
            },
            "status": "scan_done",
        }

        call_count = 0

        async def slow_dispatch(name: str, args: str) -> dict:
            nonlocal call_count
            call_count += 1
            if name == "light_context":
                await asyncio.sleep(60)  # hangs
                return {"context_state": "normal"}
            return mmwave_result

        result_sent = asyncio.Event()
        send_result = AsyncMock(side_effect=lambda *a, **kw: result_sent.set())
        orchestrator = light_orchestrator_factory(enabled=True)
        d = dispatcher_factory(
            dispatch_tool=slow_dispatch,
            send_tool_result=send_result,
            light_orchestrator=orchestrator,
            has_tool=lambda name: name in {"mmWave", "light_context"},
            timeout_s=0.1,
        )

        d.dispatch(tool_name="mmWave", args_json='{"mode":"scan"}', call_id="call-lc", is_idle=False)
        # Wait for the result to be sent (light_context times out internally).
        await asyncio.wait_for(result_sent.wait(), timeout=2.0)

        # The result should still have been sent despite light_context timeout
        assert send_result.called
        sent_json = json.loads(send_result.call_args[0][1])
        assert sent_json["status"] == "scan_done"
        # light_context should NOT be in the result (it timed out)
        assert "light_context" not in sent_json


class TestExtractSensorStateMalformed:
    """Test extract_sensor_state with malformed, missing, or unexpected inputs."""

    def test_empty_dict(self) -> None:
        state = extract_sensor_state({})
        assert "updated_at" in state
        assert state.get("mode") is None
        assert state.get("status") is None
        assert "error" not in state

    def test_missing_scan_and_measure(self) -> None:
        state = extract_sensor_state({"status": "ok", "mode": "scan"})
        assert state["status"] == "ok"
        assert state["mode"] == "scan"
        assert "device_state" not in state

    def test_scan_is_none(self) -> None:
        state = extract_sensor_state({"scan": None, "status": "ok"})
        assert "device_state" not in state
        assert state["status"] == "ok"

    def test_scan_is_string(self) -> None:
        state = extract_sensor_state({"scan": "not_a_dict"})
        assert "device_state" not in state
        assert "target_count" not in state

    def test_scan_is_list(self) -> None:
        state = extract_sensor_state({"scan": [1, 2, 3]})
        assert "device_state" not in state

    def test_scan_is_int(self) -> None:
        state = extract_sensor_state({"scan": 42})
        assert "device_state" not in state

    def test_measure_is_none(self) -> None:
        state = extract_sensor_state({"measure": None, "status": "ok"})
        assert "heart_rate_bpm" not in state

    def test_measure_is_string(self) -> None:
        state = extract_sensor_state({"measure": "invalid"})
        assert "heart_rate_bpm" not in state

    def test_scan_latest_target_is_none(self) -> None:
        state = extract_sensor_state({"scan": {"device_state": "NO_TARGET", "latest_target": None}})
        assert state["device_state"] == "NO_TARGET"
        assert "closest_target_r" not in state

    def test_scan_latest_target_is_string(self) -> None:
        state = extract_sensor_state({"scan": {"device_state": "MOVING", "latest_target": "bad"}})
        assert state["device_state"] == "MOVING"
        assert "closest_target_r" not in state

    def test_scan_latest_target_is_empty_dict(self) -> None:
        state = extract_sensor_state({"scan": {"latest_target": {}}})
        assert state.get("closest_target_r") is None
        assert state.get("closest_target_bearing") is None

    def test_scan_light_summary_is_none(self) -> None:
        state = extract_sensor_state({"scan": {"light_summary": None}})
        assert "lux" not in state

    def test_scan_light_summary_is_string(self) -> None:
        state = extract_sensor_state({"scan": {"light_summary": "bright"}})
        assert "lux" not in state

    def test_measure_valid_bio_is_none(self) -> None:
        state = extract_sensor_state({"measure": {"valid_bio": None, "device_state": "MOVING"}})
        assert state["device_state"] == "MOVING"
        assert "heart_rate_bpm" not in state

    def test_measure_valid_bio_is_string(self) -> None:
        state = extract_sensor_state({"measure": {"valid_bio": "invalid"}})
        assert "heart_rate_bpm" not in state

    def test_measure_valid_bio_empty_dict(self) -> None:
        state = extract_sensor_state({"measure": {"valid_bio": {}}})
        assert state.get("heart_rate_bpm") is None
        assert state.get("breath_rate_bpm") is None

    def test_measure_light_summary_with_none_lux(self) -> None:
        state = extract_sensor_state({"measure": {"light_summary": {"latest_lux": None}}})
        assert "lux" not in state

    def test_measure_light_summary_is_string(self) -> None:
        state = extract_sensor_state({"measure": {"light_summary": "bad"}})
        assert "lux" not in state

    def test_light_context_is_none(self) -> None:
        state = extract_sensor_state({"light_context": None})
        assert "light_context_state" not in state

    def test_light_context_is_string(self) -> None:
        state = extract_sensor_state({"light_context": "invalid"})
        assert "light_context_state" not in state

    def test_light_context_empty_dict(self) -> None:
        state = extract_sensor_state({"light_context": {}})
        assert state.get("light_context_state") is None

    def test_error_is_empty_string(self) -> None:
        """Empty string is falsy, so should not trigger error path."""
        state = extract_sensor_state({"error": "", "scan": {"device_state": "MOVING"}})
        assert "error" not in state
        assert state["device_state"] == "MOVING"

    def test_error_is_zero(self) -> None:
        """Zero is falsy, should not trigger error path."""
        state = extract_sensor_state({"error": 0, "scan": {"device_state": "MOVING"}})
        assert "error" not in state

    def test_error_is_dict(self) -> None:
        """Error as a dict (truthy) triggers error path and gets stringified."""
        state = extract_sensor_state({"error": {"code": 500}, "status": "failed"})
        assert "error" in state
        assert state["status"] == "failed"
        assert isinstance(state["error"], str)

    def test_extra_unexpected_keys_ignored(self) -> None:
        state = extract_sensor_state(
            {
                "scan": {"device_state": "MOVING"},
                "unexpected_key": [1, 2, 3],
                "another": {"nested": True},
                "status": "ok",
            }
        )
        assert state["device_state"] == "MOVING"
        assert state["status"] == "ok"
        assert "unexpected_key" not in state

    def test_nested_nulls_in_scan(self) -> None:
        state = extract_sensor_state(
            {
                "scan": {
                    "device_state": None,
                    "max_target_count": None,
                    "targets_truncated": None,
                    "latest_target": None,
                    "light_summary": None,
                },
            }
        )
        assert state.get("device_state") is None
        assert state.get("target_count") is None
        assert state.get("targets_truncated") is False

    def test_measure_device_state_none_does_not_override_scan(self) -> None:
        state = extract_sensor_state(
            {
                "scan": {"device_state": "MOVING"},
                "measure": {"device_state": None},
            }
        )
        assert state["device_state"] == "MOVING"

    def test_measure_device_state_overrides_scan(self) -> None:
        state = extract_sensor_state(
            {
                "scan": {"device_state": "MOVING"},
                "measure": {"device_state": "RESTING_VITALS"},
            }
        )
        assert state["device_state"] == "RESTING_VITALS"

    def test_measure_lux_overrides_scan_lux(self) -> None:
        state = extract_sensor_state(
            {
                "scan": {"light_summary": {"latest_lux": 100.0}},
                "measure": {"light_summary": {"latest_lux": 50.0}},
            }
        )
        assert state["lux"] == 50.0

    def test_scan_empty_dict(self) -> None:
        state = extract_sensor_state({"scan": {}})
        assert state.get("device_state") is None
        assert state.get("target_count", 0) == 0


class TestVitalsStoreIntegration:
    @pytest.mark.asyncio
    async def test_mmwave_result_appended_to_vitals_store(self, dispatcher_factory) -> None:
        """When mmWave returns valid data, it should be appended to vitals_store."""
        appended: list[dict[str, Any]] = []

        def mock_append(**kwargs: Any) -> None:
            appended.append(kwargs)

        result = {
            "scan": {
                "device_state": "STILL_NEAR",
                "latest_target": {"x": 1.0, "y": 2.0, "r": 0.8},
                "max_target_count": 1,
                "light_summary": {"latest_lux": 55.0},
            },
            "measure": {
                "device_state": "STILL_NEAR",
                "valid_bio": {"heart_rate_bpm": 72.0, "breath_rate_bpm": 16.0},
                "success": True,
            },
            "status": "ok",
        }
        dispatch = AsyncMock(return_value=result)
        d = dispatcher_factory(dispatch_tool=dispatch, on_sensor_update=lambda s: None)
        d._vitals_append = mock_append

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)

        assert len(appended) == 1
        assert appended[0]["device_state"] == "STILL_NEAR"
        assert appended[0]["heart_rate_bpm"] == 72.0
        assert appended[0]["breath_rate_bpm"] == 16.0
        assert appended[0]["target_count"] == 1
        assert appended[0]["lux"] == 55.0

    @pytest.mark.asyncio
    async def test_vitals_not_appended_for_non_mmwave(self, dispatcher_factory) -> None:
        """Non-mmWave tools should not trigger vitals append."""
        appended: list[dict[str, Any]] = []

        def mock_append(**kwargs: Any) -> None:
            appended.append(kwargs)

        dispatch = AsyncMock(return_value={"status": "ok"})
        d = dispatcher_factory(dispatch_tool=dispatch, has_tool=lambda name: True)
        d._vitals_append = mock_append

        await _dispatch_and_wait(d, tool_name="dance", args_json="{}", call_id="c1", is_idle=False)
        assert len(appended) == 0

    @pytest.mark.asyncio
    async def test_vitals_append_failure_does_not_crash(self, dispatcher_factory) -> None:
        """If vitals_append raises, the tool dispatch should still complete."""

        def bad_append(**kwargs: Any) -> None:
            raise RuntimeError("db write failed")

        result = {"scan": {"device_state": "MOVING", "latest_target": {"x": 1.0}}, "status": "ok"}
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = dispatcher_factory(dispatch_tool=dispatch, send_tool_result=send_result)
        d._vitals_append = bad_append

        await _dispatch_and_wait(d, tool_name="mmWave", args_json="{}", call_id="c1", is_idle=False)
        # Result should still be sent despite vitals_append failure
        assert send_result.called


class TestExtractSensorStateTargets:
    def test_recent_targets_included_in_state(self) -> None:
        result = {
            "status": "ok",
            "mode": "scan",
            "scan": {
                "device_state": "MULTI_TARGET",
                "max_target_count": 2,
                "targets_truncated": False,
                "latest_target": {"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0},
                "recent_targets": [
                    {"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0},
                    {"cluster": 1, "x": -0.3, "y": 0.6, "r": 0.9, "bearing": -0.4, "v": 0.02},
                ],
            },
        }
        state = extract_sensor_state(result)
        assert "targets" in state
        assert len(state["targets"]) == 2
        assert state["targets"][0]["x"] == 0.5


class TestTrendInsightInjection:
    @pytest.mark.asyncio
    async def test_trend_insight_injected_when_notable(self, dispatcher_factory) -> None:
        """When trend_analyze returns an insight, it appears in the tool result sent back."""
        insight_dict = {
            "category": "elevated_hr",
            "message": "Your heart rate is notably high",
            "severity": "attention",
            "current_value": 105.0,
            "baseline_mean": 72.0,
            "baseline_stddev": 3.5,
        }

        result = {
            "scan": {
                "device_state": "RESTING_VITALS",
                "latest_target": {"x": 1.0, "y": 0.5, "r": 0.8},
                "max_target_count": 1,
            },
            "measure": {
                "device_state": "RESTING_VITALS",
                "valid_bio": {"heart_rate_bpm": 105.0, "breath_rate_bpm": 16.0},
                "success": True,
            },
            "status": "ok",
        }

        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            on_sensor_update=lambda s: None,
            trend_analyze=lambda hr, br: insight_dict,
            trend_rollup=Mock(),
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)

        # The sent tool result should include trend_insight
        sent_json = send_result.call_args[0][1]
        sent = json.loads(sent_json)
        assert "trend_insight" in sent
        assert sent["trend_insight"]["category"] == "elevated_hr"

    @pytest.mark.asyncio
    async def test_no_trend_insight_when_none(self, dispatcher_factory) -> None:
        result = {
            "scan": {
                "device_state": "RESTING_VITALS",
                "latest_target": {"x": 1.0, "y": 0.5, "r": 0.8},
                "max_target_count": 1,
            },
            "measure": {
                "device_state": "RESTING_VITALS",
                "valid_bio": {"heart_rate_bpm": 72.0, "breath_rate_bpm": 16.0},
                "success": True,
            },
            "status": "ok",
        }

        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            on_sensor_update=lambda s: None,
            trend_analyze=lambda hr, br: None,
            trend_rollup=Mock(),
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)

        sent_json = send_result.call_args[0][1]
        sent = json.loads(sent_json)
        assert "trend_insight" not in sent

    @pytest.mark.asyncio
    async def test_rollup_called_after_mmwave(self, dispatcher_factory) -> None:
        result = {
            "scan": {"device_state": "NO_TARGET", "max_target_count": 0},
            "status": "ok",
        }
        dispatch = AsyncMock(return_value=result)
        mock_rollup = Mock()
        d = dispatcher_factory(
            dispatch_tool=dispatch,
            on_sensor_update=lambda s: None,
            trend_rollup=mock_rollup,
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)

        mock_rollup.assert_called_once()

    @pytest.mark.asyncio
    async def test_trend_analyze_exception_handled(self, dispatcher_factory) -> None:
        """If trend_analyze raises, it should not crash the dispatch."""
        result = {
            "scan": {
                "device_state": "RESTING_VITALS",
                "latest_target": {"x": 1.0, "y": 0.5, "r": 0.8},
                "max_target_count": 1,
            },
            "measure": {
                "device_state": "RESTING_VITALS",
                "valid_bio": {"heart_rate_bpm": 72.0, "breath_rate_bpm": 16.0},
                "success": True,
            },
            "status": "ok",
        }
        dispatch = AsyncMock(return_value=result)
        send_result = AsyncMock()

        def bad_analyze(hr: float, br: float) -> None:
            raise RuntimeError("boom")

        d = dispatcher_factory(
            dispatch_tool=dispatch,
            send_tool_result=send_result,
            on_sensor_update=lambda s: None,
            trend_analyze=bad_analyze,
        )

        await _dispatch_and_wait(d, tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)

        # Should still send result (no crash)
        send_result.assert_called_once()
