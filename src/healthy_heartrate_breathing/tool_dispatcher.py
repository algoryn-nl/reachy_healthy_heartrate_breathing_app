"""Tool dispatch pipeline: idle policy, dispatch, result handling."""

from __future__ import annotations
import json
import time
import asyncio
import logging
from typing import Any, Callable, Optional, Awaitable

import numpy as np

from healthy_heartrate_breathing.idle_policy import IdlePolicy
from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator


logger = logging.getLogger(__name__)


def extract_sensor_state(mmwave_result: dict[str, Any]) -> dict[str, Any]:
    """Extract a flat sensor-state snapshot from an mmWave tool result.

    When the result contains an error (e.g. serial disconnect), the returned
    state includes ``"error"`` and ``"status": "disconnected"`` so that the
    dashboard can display a meaningful disconnected indicator instead of
    silently hiding the panel.
    """
    state: dict[str, Any] = {"updated_at": time.time()}

    # Propagate error information for dashboard display
    error = mmwave_result.get("error")
    if error:
        state["error"] = str(error)
        state["status"] = mmwave_result.get("status", "error")
        state["mode"] = mmwave_result.get("mode")
        return state

    scan = mmwave_result.get("scan")
    measure = mmwave_result.get("measure")

    # Targets info from scan
    if isinstance(scan, dict):
        state["device_state"] = scan.get("device_state")
        state["target_count"] = scan.get("max_target_count", 0)
        state["targets_truncated"] = bool(scan.get("targets_truncated", False))
        latest = scan.get("latest_target")
        if isinstance(latest, dict):
            state["closest_target_r"] = latest.get("r")
            state["closest_target_bearing"] = latest.get("bearing")
        light_summary = scan.get("light_summary")
        if isinstance(light_summary, dict):
            state["lux"] = light_summary.get("latest_lux")

    # Bio info from measure
    if isinstance(measure, dict):
        if measure.get("device_state") is not None:
            state["device_state"] = measure.get("device_state")
        valid_bio = measure.get("valid_bio")
        if isinstance(valid_bio, dict):
            state["heart_rate_bpm"] = valid_bio.get("heart_rate_bpm")
            state["breath_rate_bpm"] = valid_bio.get("breath_rate_bpm")
        light_summary = measure.get("light_summary")
        if isinstance(light_summary, dict) and light_summary.get("latest_lux") is not None:
            state["lux"] = light_summary.get("latest_lux")

    # Light context (if auto-invoked)
    lc = mmwave_result.get("light_context")
    if isinstance(lc, dict):
        state["light_context_state"] = lc.get("context_state")

    state["mode"] = mmwave_result.get("mode")
    state["status"] = mmwave_result.get("status")

    return state


def _short_text(value: Any, limit: int = 220) -> str:
    """Render value as a compact single-line string for logs."""
    text = str(value).replace("\n", " ")
    if len(text) <= limit:
        return text
    return text[:limit] + "\u2026"


def _safe_parse_args(args_json: str) -> dict[str, Any]:
    """Parse tool args json and always return an object."""
    try:
        parsed = json.loads(args_json or "{}")
    except Exception:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _mmwave_has_target(result: Any) -> bool:
    """Return True when mmWave output indicates a detected target/person."""
    if not isinstance(result, dict):
        return False

    scan = result.get("scan")
    if isinstance(scan, dict):
        latest = scan.get("latest_target")
        if isinstance(latest, dict):
            return True
        recent = scan.get("recent_targets")
        if isinstance(recent, list) and len(recent) > 0:
            return True

    measure = result.get("measure")
    if isinstance(measure, dict) and bool(measure.get("success")):
        return True

    return False


def _mmwave_is_no_target(result: Any) -> bool:
    """Best-effort classification of a clean no-target mmWave result."""
    if not isinstance(result, dict):
        return False
    if result.get("error"):
        return False
    if _mmwave_has_target(result):
        return False

    scan = result.get("scan")
    if not isinstance(scan, dict):
        return False

    latest = scan.get("latest_target")
    if latest is None:
        return True

    targets_seen = scan.get("targets_seen")
    return isinstance(targets_seen, int) and targets_seen <= 0


def _mmwave_is_multi_target(result: Any) -> bool:
    """Return True when mmWave output indicates multiple targets."""
    if not isinstance(result, dict):
        return False
    scan = result.get("scan")
    if not isinstance(scan, dict):
        return False
    max_count = scan.get("max_target_count", 0)
    return isinstance(max_count, int) and max_count > 1


# -- Device state context enrichment ------------------------------------------

_STATE_INFO: dict[str, tuple[str, str]] = {
    "RESTING_VITALS": (
        "high",
        "Person is still and close — ideal for heart rate and breathing measurement.",
    ),
    "STILL_NEAR": (
        "moderate",
        "Person is close and settling. Wait for RESTING_VITALS for most reliable readings.",
    ),
    "MOVING": (
        "low",
        "Person is moving — vitals readings may be inaccurate.",
    ),
    "PRESENT_FAR": (
        "unavailable",
        "Person detected but too far for vitals measurement.",
    ),
    "MULTI_TARGET": (
        "unavailable",
        "Multiple people detected — cannot isolate vitals.",
    ),
    "NO_TARGET": (
        "unavailable",
        "No person detected.",
    ),
}

_TRANSITION_SUFFIXES: dict[str, str] = {
    "RESTING_VITALS": "now in ideal position for vitals.",
    "STILL_NEAR": "vitals measurement should improve soon.",
    "MOVING": "movement may affect vitals accuracy.",
    "PRESENT_FAR": "person moved out of vitals range.",
    "MULTI_TARGET": "multiple people now detected.",
    "NO_TARGET": "person left the area.",
}


def build_device_context(sensor_state: dict[str, Any]) -> dict[str, Any] | None:
    """Build a device-context dict from sensor state for LLM consumption.

    Returns ``None`` when no device state is available (error/disconnect).
    """
    current = sensor_state.get("device_state")
    if current is None:
        return None

    previous: str | None = sensor_state.get("previous_device_state")
    changed = previous is not None and previous != current

    info = _STATE_INFO.get(current)
    if info is not None:
        reliability, note = info
    else:
        reliability = "unknown"
        note = f"Unrecognised device state: {current}."

    transition: str | None = None
    if changed:
        suffix = _TRANSITION_SUFFIXES.get(current, f"state is now {current}.")
        if previous == "NO_TARGET":
            transition = f"{previous} → {current} — someone just arrived; {suffix}"
        else:
            transition = f"{previous} → {current} — {suffix}"

    return {
        "state": current,
        "previous_state": previous,
        "changed": changed,
        "vitals_reliability": reliability,
        "transition": transition,
        "note": note,
    }


class ToolDispatcher:
    """Dispatches tool calls, integrating idle policy and light orchestration.

    Testable without OpenAI connections -- all I/O is via injected callbacks.
    """

    def __init__(  # noqa: D107
        self,
        *,
        idle_policy: IdlePolicy,
        light_orchestrator: LightOrchestrator,
        has_tool: Callable[[str], bool],
        dispatch_tool: Callable[[str, str], Awaitable[dict[str, Any]]],
        send_tool_result: Callable[[str, str], Awaitable[None]],
        create_response: Callable[[str], Awaitable[None]],
        create_message: Callable[[dict[str, Any]], Awaitable[None]],
        enqueue_output: Callable[[dict[str, Any]], Awaitable[None]],
        get_camera_frame: Callable[[], np.ndarray | None],
        head_wobbler_reset: Callable[[], None] | None,
        timeout_s: float = 30.0,
        on_sensor_update: Optional[Callable[[dict[str, Any]], None]] = None,
    ) -> None:
        self._idle_policy = idle_policy
        self._light_orchestrator = light_orchestrator
        self._has_tool = has_tool
        self._dispatch_tool = dispatch_tool
        self._send_tool_result = send_tool_result
        self._create_response = create_response
        self._create_message = create_message
        self._enqueue_output = enqueue_output
        self._get_camera_frame = get_camera_frame
        self._head_wobbler_reset = head_wobbler_reset
        self._timeout_s = timeout_s
        self._on_sensor_update = on_sensor_update
        self._semaphore = asyncio.Semaphore(1)
        self._active_task: asyncio.Task[None] | None = None
        self._last_device_state: str | None = None

    def dispatch(
        self,
        *,
        tool_name: str,
        args_json: str,
        call_id: str | None,
        is_idle: bool,
    ) -> None:
        """Fire-and-forget tool dispatch. Returns immediately."""
        asyncio.create_task(
            self._guarded_run(
                tool_name=tool_name,
                args_json=args_json,
                call_id=call_id,
                is_idle=is_idle,
            ),
            name=f"tool-dispatch-{tool_name}",
        )

    async def cancel(self) -> None:
        """Cancel any in-flight tool task (for session teardown)."""
        task = self._active_task
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._active_task = None

    async def _guarded_run(
        self,
        *,
        tool_name: str,
        args_json: str,
        call_id: str | None,
        is_idle: bool,
    ) -> None:
        """Acquire semaphore, track active task, then run tool."""
        try:
            async with self._semaphore:
                self._active_task = asyncio.current_task()
                try:
                    await self._run_tool(
                        tool_name=tool_name,
                        args_json=args_json,
                        call_id=call_id,
                        is_idle=is_idle,
                    )
                finally:
                    self._active_task = None
        except asyncio.CancelledError:
            logger.info("Tool dispatch cancelled for '%s'", tool_name)
        except Exception:
            logger.exception("Unhandled error in tool dispatch for '%s'", tool_name)

    async def _run_tool(
        self,
        *,
        tool_name: str,
        args_json: str,
        call_id: str | None,
        is_idle: bool,
    ) -> None:
        """Execute a tool call with timeout, idle policy, and result handling."""
        logger.info("Tool requested: %s args=%s", tool_name, _short_text(args_json))

        effective_args_json = args_json
        idle_mmwave_sweep_used = False
        tool_result: dict[str, Any]

        # Idle policy: suppress non-mmWave tools when mmWave is available
        if is_idle and self._has_tool("mmWave") and tool_name != "mmWave":
            tool_result = {
                "status": "skipped",
                "reason": (f"idle tool '{tool_name}' suppressed; using passive mmWave idle policy"),
            }
            logger.info("Idle policy suppressed tool: %s", tool_name)
        else:
            # Override mmWave args during idle
            if is_idle and tool_name == "mmWave":
                now = asyncio.get_event_loop().time()
                idle_args = _safe_parse_args(args_json)
                idle_mmwave_sweep_used = self._idle_policy.sweep_allowed(now)
                idle_args["mode"] = "scan" if self._idle_policy.suggest_scan_only else "locate_and_measure"
                idle_args["duration_s"] = self._idle_policy.probe_duration_s
                idle_args["sweep_if_unseen"] = idle_mmwave_sweep_used
                effective_args_json = json.dumps(idle_args)
                if idle_mmwave_sweep_used:
                    self._idle_policy.record_sweep_used(now)
                logger.info(
                    "Idle mmWave policy: misses=%d/%d, sweep_if_unseen=%s, args=%s",
                    self._idle_policy.consecutive_misses,
                    self._idle_policy.misses_before_sweep,
                    idle_mmwave_sweep_used,
                    _short_text(effective_args_json),
                )

            try:
                tool_result = await asyncio.wait_for(
                    self._dispatch_tool(tool_name, effective_args_json),
                    timeout=self._timeout_s,
                )
                logger.debug("Tool '%s' executed successfully", tool_name)
                logger.debug("Tool result: %s", tool_result)
            except asyncio.TimeoutError:
                logger.error("Tool '%s' timed out after %.1fs", tool_name, self._timeout_s)
                tool_result = {"error": f"tool '{tool_name}' timed out after {self._timeout_s:.0f}s"}
            except asyncio.CancelledError:
                logger.info("Tool '%s' cancelled", tool_name)
                return
            except Exception as e:
                logger.error("Tool '%s' failed", tool_name)
                tool_result = {"error": str(e)}

        # Idle mmWave result tracking
        if is_idle and tool_name == "mmWave":
            now = asyncio.get_event_loop().time()
            if isinstance(tool_result, dict) and tool_result.get("error"):
                logger.warning("Idle mmWave failed: %s", _short_text(tool_result.get("error")))
                self._idle_policy.record_error(now)
            elif _mmwave_is_multi_target(tool_result):
                self._idle_policy.record_multi_target(now)
            elif _mmwave_has_target(tool_result):
                self._idle_policy.record_target_found(now)
            elif _mmwave_is_no_target(tool_result):
                self._idle_policy.record_no_target(sweep_was_used=idle_mmwave_sweep_used)
            else:
                self._idle_policy.record_inconclusive()

        # Auto light_context after mmWave (independent timeout)
        if tool_name == "mmWave" and isinstance(tool_result, dict):
            try:

                async def _dispatch_light(name: str, args_json_str: str) -> dict[str, Any]:
                    return await self._dispatch_tool(name, args_json_str)

                auto_light_context = await asyncio.wait_for(
                    self._light_orchestrator.run_from_mmwave(
                        tool_result,
                        is_idle=is_idle,
                        has_tool=self._has_tool("light_context"),
                        dispatch_fn=_dispatch_light,
                    ),
                    timeout=self._timeout_s,
                )
                if auto_light_context is not None:
                    tool_result["light_context"] = auto_light_context
                    logger.info(
                        "Auto light_context state=%s mode=%s",
                        auto_light_context.get("context_state"),
                        auto_light_context.get("recommended_mode"),
                    )
            except asyncio.TimeoutError:
                logger.warning("Auto light_context timed out after %.1fs", self._timeout_s)
            except Exception as e:
                logger.warning("Auto light_context failed after mmWave: %s", e)

        # Update sensor state for dashboard + device context enrichment
        if tool_name == "mmWave" and isinstance(tool_result, dict):
            try:
                new_sensor = extract_sensor_state(tool_result)
                new_sensor["previous_device_state"] = self._last_device_state
                self._last_device_state = new_sensor.get("device_state")
                if self._on_sensor_update is not None:
                    self._on_sensor_update(new_sensor)
                device_ctx = build_device_context(new_sensor)
                if device_ctx is not None:
                    tool_result["device_context"] = device_ctx
            except Exception:
                logger.debug("Sensor state update failed", exc_info=True)

        # Send tool result back to connection
        if isinstance(call_id, str):
            await self._send_tool_result(call_id, json.dumps(tool_result))

        # Enqueue UI output
        await self._enqueue_output(
            {
                "role": "assistant",
                "content": json.dumps(tool_result),
                "metadata": {"title": f"\U0001f6e0\ufe0f Used tool {tool_name}", "status": "done"},
            },
        )

        # Camera image handling
        if tool_name == "camera" and "b64_im" in tool_result:
            await self._handle_camera_result(tool_result)

        # Create response: always speak for user-initiated calls;
        # for idle probes, speak when something noteworthy happened.
        if is_idle:
            idle_noteworthy = False
            if tool_name == "mmWave" and isinstance(tool_result, dict):
                ctx = tool_result.get("device_context")
                has_vitals = tool_result.get("status") == "ok"
                state_changed = isinstance(ctx, dict) and ctx.get("changed", False)
                idle_noteworthy = has_vitals or state_changed
            if idle_noteworthy:
                await self._create_response(
                    "The idle scan just returned noteworthy results. "
                    "Briefly comment on what you observed — state changes, "
                    "vitals, or who arrived/left. Keep it to one or two sentences.",
                )
        else:
            await self._create_response(
                "Use the tool result just returned and answer concisely in speech.",
            )

        # Reset head wobbler after tool call
        if self._head_wobbler_reset is not None:
            self._head_wobbler_reset()

    async def _handle_camera_result(self, tool_result: dict[str, Any]) -> None:
        """Handle camera tool result: send image to conversation and UI."""
        import cv2
        import gradio as gr

        b64_im = tool_result["b64_im"]
        if not isinstance(b64_im, str):
            logger.warning("Unexpected type for b64_im: %s", type(b64_im))
            b64_im = str(b64_im)

        await self._create_message(
            {
                "type": "message",
                "role": "user",
                "content": [
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{b64_im}",
                    },
                ],
            },
        )
        logger.info("Added camera image to conversation")

        np_img = self._get_camera_frame()
        if np_img is not None:
            rgb_frame = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        else:
            rgb_frame = None
        img = gr.Image(value=rgb_frame)

        await self._enqueue_output(
            {
                "role": "assistant",
                "content": img,
            },
        )
