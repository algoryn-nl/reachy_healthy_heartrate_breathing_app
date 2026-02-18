"""Tool to interact with the Reachy mmWave firmware over serial."""

from __future__ import annotations
import os
import re
import glob
import json
import time
import asyncio
import logging
from typing import Any, Dict, Optional

from reachy_mini.utils import create_head_pose
from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies
from healthy_heartrate_breathing.dance_emotion_moves import GotoQueueMove


logger = logging.getLogger(__name__)


def _to_ms(value: Any, default: int) -> int:
    """Coerce a positive int and clamp to a minimum of 1."""
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return max(1, parsed)


def _to_int_or_default(value: Any, default: int) -> int:
    """Coerce to int, or return default."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value: Any, default: float) -> float:
    """Coerce a float and clamp to a minimum of 0."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return max(0.0, parsed)


class MmWave(Tool):
    """Read mmWave telemetry and request heart/breath values."""

    name = "mmWave"
    description = (
        "Locate people with mmWave targets telemetry and measure heart/breath rates "
        "when a single, stationary person is in range."
    )
    parameters_schema = {
        "type": "object",
        "properties": {
            "mode": {
                "type": "string",
                "enum": ["scan", "measure", "locate_and_measure"],
                "description": "scan for people, measure heart/breath now, or do both",
            },
            "serial_port": {"type": "string", "description": "USB serial port for mmWave"},
            "duration_s": {
                "type": "number",
                "minimum": 1,
                "description": "How long to collect telemetry in scan/measurement mode",
            },
            "focus_cluster": {
                "type": "integer",
                "minimum": -1,
                "description": "Use a specific target cluster for measurement. -1 for auto",
            },
            "sweep_if_unseen": {
                "type": "boolean",
                "description": "Run a short scan sweep if no target is found",
            },
            "targets_ms": {"type": "integer", "minimum": 50},
            "bio_ms": {"type": "integer", "minimum": 50},
        },
        "required": [],
    }

    DEFAULT_SCAN_SECONDS = 8
    DEFAULT_MEASURE_SECONDS = 15
    DEFAULT_TARGETS_MS = 250
    DEFAULT_BIO_MS = 1000

    def _resolve_serial_port(self, explicit_port: Optional[str]) -> str:
        if explicit_port:
            return explicit_port

        env_port = os.getenv("MMWAVE_SERIAL_PORT")
        if env_port:
            return env_port

        candidates = sorted(
            glob.glob("/dev/cu.usbmodem*")
            + glob.glob("/dev/tty.usbmodem*")
            + glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
        )
        if not candidates:
            raise RuntimeError("No mmWave serial port found. Set MMWAVE_SERIAL_PORT.")
        return candidates[0]

    def _send_line(self, ser: Any, line: str) -> None:
        payload = f"{line}\n".encode("utf-8")
        ser.write(payload)
        ser.flush()

    def _parse_payload(self, raw: bytes) -> Optional[Dict[str, Any]]:
        text = raw.strip().decode("utf-8", errors="ignore").strip()
        if not text:
            return None

        # Serial monitor output can include timestamp prefixes such as:
        # "20:09:52.663 -> { ... }". Strip that noise first.
        if "->" in text:
            suffix = text.split("->", 1)[1].strip()
            if suffix:
                text = suffix

        # Keep the first JSON object on the line if extra text is present.
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end == -1 or end < start:
            return None
        text = text[start : end + 1].strip()

        # Handle malformed monitor copy patterns like:
        # {"type":"state","t_ms":34792PRESENT_FAR","pose":"SITTING",...}
        # where the `state` key got glued to the t_ms value.
        text = re.sub(r'"t_ms":(\d+)([A-Z_]+)",', r'"t_ms":\1,"state":"\2",', text, count=1)

        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            logger.debug("Ignoring non-json payload: %s", text)
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    def _pick_target_from_message(self, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if msg.get("type") != "targets":
            return None

        n_raw = msg.get("n", msg.get("n_targets", 0))
        try:
            n = int(n_raw)
        except (TypeError, ValueError):
            targets = msg.get("targets")
            if isinstance(targets, list):
                n = len(targets)
            else:
                n = 0

        if n <= 0:
            return None

        targets = msg.get("targets")
        if not isinstance(targets, list):
            return None

        valid: list[dict[str, Any]] = []
        for item in targets:
            if not isinstance(item, dict):
                continue
            cluster = item.get("cluster")
            x = item.get("x")
            y = item.get("y")
            r = item.get("r")
            if not all(isinstance(v, (int, float)) for v in (x, y, r, cluster)):
                continue
            valid.append(
                {
                    "cluster": int(cluster),
                    "x": float(x),
                    "y": float(y),
                    "r": float(r),
                    "bearing": float(item.get("bearing", 0.0)),
                    "v": float(item.get("v", 0.0)),
                },
            )

        if not valid:
            return None

        return min(valid, key=lambda item: item["r"])

    def _queue_short_sweep(self, deps: ToolDependencies) -> None:
        if deps.movement_manager is None:
            return
        try:
            head_pose = deps.reachy_mini.get_current_head_pose()
            head_joints, antenna_joints = deps.reachy_mini.get_current_joint_positions()
            current_body_yaw = head_joints[0]
            current_ant1 = antenna_joints[0]
            current_ant2 = antenna_joints[1]
            max_yaw = 0.8
            move_s = 1.2
            hold_s = 0.8

            left_pose = create_head_pose(0, 0, 0, 0, 0, max_yaw, degrees=False)
            center_pose = create_head_pose(0, 0, 0, 0, 0, 0, degrees=False)
            right_pose = create_head_pose(0, 0, 0, 0, 0, -max_yaw, degrees=False)

            move_to_left = GotoQueueMove(
                target_head_pose=left_pose,
                start_head_pose=head_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw + max_yaw,
                start_body_yaw=current_body_yaw,
                duration=move_s,
            )
            hold_left = GotoQueueMove(
                target_head_pose=left_pose,
                start_head_pose=left_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw + max_yaw,
                start_body_yaw=current_body_yaw + max_yaw,
                duration=hold_s,
            )
            center_from_left = GotoQueueMove(
                target_head_pose=center_pose,
                start_head_pose=left_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw,
                start_body_yaw=current_body_yaw + max_yaw,
                duration=move_s,
            )
            move_to_right = GotoQueueMove(
                target_head_pose=right_pose,
                start_head_pose=center_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw - max_yaw,
                start_body_yaw=current_body_yaw,
                duration=move_s,
            )
            hold_right = GotoQueueMove(
                target_head_pose=right_pose,
                start_head_pose=right_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw - max_yaw,
                start_body_yaw=current_body_yaw - max_yaw,
                duration=hold_s,
            )
            return_center = GotoQueueMove(
                target_head_pose=center_pose,
                start_head_pose=right_pose,
                target_antennas=(current_ant1, current_ant2),
                start_antennas=(current_ant1, current_ant2),
                target_body_yaw=current_body_yaw,
                start_body_yaw=current_body_yaw - max_yaw,
                duration=move_s,
            )

            deps.movement_manager.clear_move_queue()
            deps.movement_manager.queue_move(move_to_left)
            deps.movement_manager.queue_move(hold_left)
            deps.movement_manager.queue_move(center_from_left)
            deps.movement_manager.queue_move(move_to_right)
            deps.movement_manager.queue_move(hold_right)
            deps.movement_manager.queue_move(return_center)
            deps.movement_manager.set_moving_state(4 * move_s + 2 * hold_s)
        except Exception:
            logger.warning("Could not queue sweep moves")

    def _scan_sync(
        self,
        ser: Any,
        *,
        deps: ToolDependencies,
        duration_s: float,
        do_sweep: bool,
        focus_cluster: int,
        targets_ms: int,
        stop_when_target_found: bool = False,
    ) -> Dict[str, Any]:
        state = {
            "targets_seen": 0,
            "latest_target": None,
            "recent_targets": [],
            "state": None,
            "telemetry": [],
        }
        timeout = time.monotonic() + max(0.5, duration_s)
        if do_sweep:
            self._queue_short_sweep(deps)

        self._send_line(ser, "HM=1")
        self._send_line(ser, f"TARGETS_MS={targets_ms}")
        if focus_cluster >= 0:
            self._send_line(ser, f"FOCUS={focus_cluster}")

        while time.monotonic() < timeout:
            raw = ser.readline()
            if not raw:
                continue
            msg = self._parse_payload(raw)
            if not msg:
                continue

            msg_type = msg.get("type")
            if msg_type == "targets":
                state["telemetry"].append(msg)
                state["targets_seen"] += 1
                best = self._pick_target_from_message(msg)
                if best is not None:
                    state["latest_target"] = best
                    if do_sweep:
                        deps.movement_manager.clear_move_queue()
                    if isinstance(msg.get("n"), int) and msg.get("n") == 1:
                        # Prefer the single target once seen as "high confidence"
                        state["recent_targets"].append(best)
                    if stop_when_target_found and state["latest_target"] is not None:
                        break
            if msg_type == "state":
                state["state"] = msg.get("state")

        if state["recent_targets"]:
            unique = []
            for target in state["recent_targets"]:
                if target not in unique:
                    unique.append(target)
            state["recent_targets"] = unique
        return state

    def _measure_sync(
        self,
        ser: Any,
        *,
        focus_cluster: int,
        timeout_s: float,
        bio_ms: int,
    ) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "attempts": 0,
            "latest_bio": None,
            "state": None,
            "bio_messages": [],
            "valid_bio": None,
            "success": False,
        }

        if focus_cluster >= 0:
            self._send_line(ser, f"FOCUS={focus_cluster}")
        self._send_line(ser, "HM=0")
        self._send_line(ser, f"BIO_MS={bio_ms}")

        timeout = time.monotonic() + timeout_s
        while time.monotonic() < timeout:
            raw = ser.readline()
            if not raw:
                continue
            msg = self._parse_payload(raw)
            if not msg:
                continue

            msg_type = msg.get("type")
            if msg_type == "state":
                result["state"] = msg
            if msg_type == "bio":
                result["attempts"] += 1
                result["latest_bio"] = msg
                result["bio_messages"].append(msg)

                allowed = bool(msg.get("allowed", 0))
                valid = bool(msg.get("valid", 0))
                if allowed and valid and msg.get("br") is not None and msg.get("hr") is not None:
                    result["success"] = True
                    result["valid_bio"] = {
                        "heart_rate_bpm": msg.get("hr"),
                        "breath_rate_bpm": msg.get("br"),
                        "state": msg.get("state"),
                        "hr_new": msg.get("hr_new"),
                        "br_new": msg.get("br_new"),
                    }
                    break
        return result

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Execute mmWave scan/measure orchestration."""
        mode = kwargs.get("mode", "locate_and_measure")
        if mode not in {"scan", "measure", "locate_and_measure"}:
            return {"error": f"invalid mode '{mode}', use scan, measure, or locate_and_measure"}

        serial_port = self._resolve_serial_port(kwargs.get("serial_port"))
        try:
            import serial
            import serial.serialutil
        except ModuleNotFoundError as exc:
            return {"error": f"missing dependency: {exc.name or 'pyserial'}"}

        duration_s = float(_to_float(kwargs.get("duration_s", 0), 0.0))
        if mode == "locate_and_measure":
            duration_s = _to_float(kwargs.get("duration_s", self.DEFAULT_SCAN_SECONDS), self.DEFAULT_SCAN_SECONDS)
            measure_duration = _to_float(
                kwargs.get("measure_duration_s", self.DEFAULT_MEASURE_SECONDS), self.DEFAULT_MEASURE_SECONDS
            )
            do_sweep = bool(kwargs.get("sweep_if_unseen", True))
            targets_ms = _to_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = _to_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)
        elif mode == "scan":
            duration_s = _to_float(kwargs.get("duration_s", self.DEFAULT_SCAN_SECONDS), self.DEFAULT_SCAN_SECONDS)
            measure_duration = 0.0
            do_sweep = bool(kwargs.get("sweep_if_unseen", True))
            targets_ms = _to_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = _to_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)
        else:
            # measure
            measure_duration = _to_float(
                kwargs.get("duration_s", self.DEFAULT_MEASURE_SECONDS), self.DEFAULT_MEASURE_SECONDS
            )
            do_sweep = False
            targets_ms = _to_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = _to_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)

        focus_cluster = _to_int_or_default(kwargs.get("focus_cluster", -1), -1)

        def run_session() -> Dict[str, Any]:
            effective_focus_cluster = focus_cluster
            with serial.Serial(serial_port, 115200, timeout=0.2) as ser:
                response: Dict[str, Any] = {
                    "serial_port": serial_port,
                    "mode": mode,
                    "status": "started",
                }

                if mode in {"scan", "locate_and_measure"}:
                    scan_result = self._scan_sync(
                        ser,
                        deps=deps,
                        duration_s=max(0.5, duration_s),
                        do_sweep=do_sweep,
                        focus_cluster=effective_focus_cluster,
                        targets_ms=targets_ms,
                        stop_when_target_found=(mode == "locate_and_measure"),
                    )
                    response["scan"] = scan_result
                    response["mode"] = mode

                if mode == "scan":
                    response["status"] = "scan_done"
                    return response

                if mode in {"measure", "locate_and_measure"}:
                    if mode == "locate_and_measure":
                        latest = response["scan"].get("latest_target") if "scan" in response else None
                        if latest is not None:
                            effective_focus_cluster = int(latest.get("cluster", -1))

                    measure_result = self._measure_sync(
                        ser,
                        focus_cluster=effective_focus_cluster,
                        timeout_s=max(0.5, measure_duration),
                        bio_ms=bio_ms,
                    )
                    response["measure"] = measure_result
                    if measure_result["success"]:
                        response["status"] = "ok"
                    else:
                        response["status"] = "measure_inconclusive"

                return response

        try:
            return await asyncio.to_thread(run_session)
        except serial.serialutil.SerialException as e:
            return {"error": f"serial error on {serial_port}: {e}"}
        except Exception as e:
            logger.exception("mmWave tool failed")
            return {"error": str(e)}
