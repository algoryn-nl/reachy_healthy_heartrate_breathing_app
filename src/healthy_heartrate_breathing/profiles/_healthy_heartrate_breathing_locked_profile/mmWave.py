"""Tool to interact with the Reachy mmWave firmware over serial."""

from __future__ import annotations
import os
import glob
import time
import asyncio
import logging
from typing import Any, Dict, Optional

from reachy_mini.utils import create_head_pose
from healthy_heartrate_breathing.env_utils import coerce_ms, coerce_int, coerce_float_nonneg
from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies
from healthy_heartrate_breathing.dance_emotion_moves import GotoQueueMove
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    CMD_PING,
    CMD_SET_HM,
    CMD_SET_FOCUS,
    PROTO_VERSION,
    CMD_SET_BIO_MS,
    CMD_SET_TARGETS_MS,
    ProtocolError,
    decode_event,
    decode_frame,
    encode_frame,
    pack_cmd_set_hm,
    pack_cmd_set_focus,
    pack_cmd_set_bio_ms,
    extract_encoded_frames,
    pack_cmd_set_targets_ms,
)


logger = logging.getLogger(__name__)

# Device signature for serial port auto-detection (Seeed XIAO ESP32)
EXPECTED_VID = 0x303A
EXPECTED_PID = 0x1001
PROBE_PING_TIMEOUT_S = 0.5
PROBE_HELLO_TIMEOUT_S = 2.0

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:
    serial = None  # type: ignore[assignment]
    comports = None  # type: ignore[assignment]


class MmWave(Tool):
    """Read mmWave telemetry and request heart/breath values."""

    name = "mmWave"

    def __init__(self) -> None:  # noqa: D107
        self._warned_version_mismatch = False

    description = (
        "Locate people with mmWave targets telemetry and measure heart/breath rates "
        "when a single, stationary person is in range. Includes ambient light lux telemetry when available."
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

    def _summarize_light(self, samples: list[Dict[str, Any]]) -> Dict[str, Any]:
        """Build a compact summary of lux samples collected during a call."""
        valid_lux_values: list[float] = []
        latest_lux: float | None = None
        latest_t_ms: int | None = None

        for sample in samples:
            if not isinstance(sample, dict):
                continue
            lux = sample.get("lux")
            if not isinstance(lux, (int, float)):
                continue
            lux_f = float(lux)
            valid_lux_values.append(lux_f)
            latest_lux = lux_f
            t_ms = sample.get("t_ms")
            latest_t_ms = int(t_ms) if isinstance(t_ms, int) else latest_t_ms

        avg_lux = sum(valid_lux_values) / len(valid_lux_values) if valid_lux_values else None
        min_lux = min(valid_lux_values) if valid_lux_values else None
        max_lux = max(valid_lux_values) if valid_lux_values else None

        return {
            "samples": len(samples),
            "valid_samples": len(valid_lux_values),
            "latest_lux": latest_lux,
            "latest_t_ms": latest_t_ms,
            "avg_lux": avg_lux,
            "min_lux": min_lux,
            "max_lux": max_lux,
        }

    def _resolve_serial_port(self, explicit_port: str | None = None) -> str:
        """Resolve serial port: explicit → env → VID/PID → glob → probe."""
        if explicit_port:
            return explicit_port

        env_port = os.getenv("MMWAVE_SERIAL_PORT")
        if env_port:
            return env_port

        # Tier 1: VID/PID match
        candidates: list[str] = []
        if comports is not None:
            for port_info in comports():
                if port_info.vid == EXPECTED_VID and port_info.pid == EXPECTED_PID:
                    candidates.append(port_info.device)

        if len(candidates) == 1:
            logger.info("Auto-detected mmWave port by VID/PID: %s", candidates[0])
            return candidates[0]

        if len(candidates) > 1:
            logger.info("Multiple VID/PID matches (%s), probing...", candidates)
            for candidate in candidates:
                if self._probe_port(candidate):
                    return candidate
            logger.warning("No VID/PID candidate responded to probe, using first: %s", candidates[0])
            return candidates[0]

        # Tier 2: Glob fallback
        logger.warning("No VID/PID match found, falling back to glob-based detection")
        glob_candidates = sorted(
            glob.glob("/dev/cu.usbmodem*")
            + glob.glob("/dev/tty.usbmodem*")
            + glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
        )

        if not glob_candidates:
            raise RuntimeError("No mmWave serial port found. Set MMWAVE_SERIAL_PORT.")

        if len(glob_candidates) == 1:
            logger.info("Auto-detected mmWave port by glob: %s", glob_candidates[0])
            return glob_candidates[0]

        # Tier 3: Probe glob candidates
        logger.info("Multiple glob candidates (%s), probing...", glob_candidates)
        for candidate in glob_candidates:
            if self._probe_port(candidate):
                return candidate

        logger.warning("No glob candidate responded to probe, using first: %s", glob_candidates[0])
        return glob_candidates[0]

    def _probe_port(self, port: str) -> bool:
        """Probe a serial port for our firmware: PING→PONG, then DTR reset→HELLO."""
        try:
            ser = serial.Serial(port, 115200, timeout=PROBE_PING_TIMEOUT_S, dsrdtr=False)
        except (OSError, serial.serialutil.SerialException) as exc:
            logger.warning("Cannot open %s for probe: %s", port, exc)
            return False

        try:
            # Phase 1: send PING, wait for PONG
            tx_state = {"seq": 0}
            try:
                self._send_command(ser, tx_state, CMD_PING)
            except OSError:
                pass
            else:
                deadline = time.monotonic() + PROBE_PING_TIMEOUT_S
                rx_buffer = bytearray()
                while time.monotonic() < deadline:
                    events = self._poll_events(ser, rx_buffer)
                    for event in events:
                        if event.get("type") == "pong":
                            logger.info("Probe: %s responded to PING", port)
                            return True

            # Phase 2: toggle DTR to reset, wait for HELLO
            ser.dtr = True
            time.sleep(0.05)
            ser.dtr = False

            deadline = time.monotonic() + PROBE_HELLO_TIMEOUT_S
            rx_buffer = bytearray()
            while time.monotonic() < deadline:
                events = self._poll_events(ser, rx_buffer)
                for event in events:
                    if event.get("type") == "hello" and event.get("proto_version") == PROTO_VERSION:
                        logger.info("Probe: %s responded with HELLO after reset", port)
                        return True
        except OSError as exc:
            logger.warning("Probe error on %s: %s", port, exc)
        finally:
            ser.close()

        logger.debug("Probe: %s did not respond", port)
        return False

    def _next_tx_seq(self, tx_state: Dict[str, int]) -> int:
        seq = tx_state["seq"]
        tx_state["seq"] = (seq + 1) & 0xFFFF
        return seq

    def _send_command(self, ser: Any, tx_state: Dict[str, int], msg_type: int, payload: bytes = b"") -> None:
        frame = encode_frame(
            msg_type=msg_type, payload=payload, seq=self._next_tx_seq(tx_state), version=PROTO_VERSION
        )
        try:
            ser.write(frame)
            ser.flush()
        except OSError as exc:
            logger.warning("Serial write failed (msg_type=0x%02X): %s", msg_type, exc)
            raise

    def _poll_events(self, ser: Any, rx_buffer: bytearray) -> list[Dict[str, Any]]:
        try:
            in_waiting = getattr(ser, "in_waiting", 0)
            read_size = in_waiting if isinstance(in_waiting, int) and in_waiting > 0 else 1
            chunk = ser.read(read_size)
        except OSError as exc:
            logger.warning("Serial read failed: %s", exc)
            raise
        if not chunk:
            return []

        rx_buffer.extend(chunk)
        events: list[Dict[str, Any]] = []
        for encoded_frame in extract_encoded_frames(rx_buffer):
            try:
                version, msg_type, _seq, payload = decode_frame(encoded_frame)
            except ProtocolError:
                logger.debug("Ignoring invalid frame", exc_info=True)
                continue

            if version != PROTO_VERSION:
                if not self._warned_version_mismatch:
                    logger.warning(
                        "Protocol version mismatch: device sent v%s, host expects v%s. "
                        "Firmware/host may be out of sync.",
                        version,
                        PROTO_VERSION,
                    )
                    self._warned_version_mismatch = True
                continue

            try:
                event = decode_event(msg_type, payload)
            except ProtocolError:
                logger.debug("Ignoring frame with invalid payload", exc_info=True)
                continue

            events.append(event)

        return events

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
        tx_state: Dict[str, int],
        rx_buffer: bytearray,
        duration_s: float,
        do_sweep: bool,
        focus_cluster: int,
        targets_ms: int,
        stop_when_target_found: bool = False,
    ) -> Dict[str, Any]:
        state = {
            "targets_seen": 0,
            "max_target_count": 0,
            "targets_truncated": False,
            "latest_target": None,
            "recent_targets": [],
            "latest_light": None,
            "light_samples": [],
            "state": None,
            "device_state": None,
            "telemetry": [],
        }
        timeout = time.monotonic() + max(0.5, duration_s)
        if do_sweep:
            self._queue_short_sweep(deps)

        try:
            self._send_command(ser, tx_state, CMD_SET_HM, pack_cmd_set_hm(1))
            self._send_command(ser, tx_state, CMD_SET_TARGETS_MS, pack_cmd_set_targets_ms(targets_ms))
            if focus_cluster >= 0:
                self._send_command(ser, tx_state, CMD_SET_FOCUS, pack_cmd_set_focus(focus_cluster))
        except OSError:
            state["light_summary"] = self._summarize_light(state["light_samples"])
            return state

        while time.monotonic() < timeout:
            try:
                events = self._poll_events(ser, rx_buffer)
            except OSError:
                break
            if not events:
                continue

            for msg in events:
                msg_type = msg.get("type")
                if msg_type == "targets":
                    state["telemetry"].append(msg)
                    state["targets_seen"] += 1
                    n = msg.get("n", 0)
                    if isinstance(n, int) and n > state["max_target_count"]:
                        state["max_target_count"] = n
                    if msg.get("targets_truncated"):
                        state["targets_truncated"] = True
                    best = self._pick_target_from_message(msg)
                    if best is not None:
                        state["latest_target"] = best
                        if do_sweep and deps.movement_manager is not None:
                            deps.movement_manager.clear_move_queue()
                        if isinstance(msg.get("n"), int) and msg.get("n") == 1:
                            # Prefer the single target once seen as "high confidence"
                            state["recent_targets"].append(best)
                        if stop_when_target_found:
                            break
                if msg_type == "state":
                    state["state"] = msg.get("state")
                    state["device_state"] = msg.get("state")
                if msg_type == "light":
                    state["light_samples"].append(msg)
                    lux = msg.get("lux")
                    if isinstance(lux, (int, float)):
                        state["latest_light"] = {
                            "t_ms": msg.get("t_ms"),
                            "valid": msg.get("valid"),
                            "lux": float(lux),
                        }

            if stop_when_target_found and state["latest_target"] is not None:
                break

        if state["recent_targets"]:
            unique = []
            for target in state["recent_targets"]:
                if target not in unique:
                    unique.append(target)
            state["recent_targets"] = unique
        state["light_summary"] = self._summarize_light(state["light_samples"])
        return state

    def _measure_sync(
        self,
        ser: Any,
        *,
        tx_state: Dict[str, int],
        rx_buffer: bytearray,
        focus_cluster: int,
        timeout_s: float,
        bio_ms: int,
    ) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "attempts": 0,
            "latest_bio": None,
            "latest_light": None,
            "light_samples": [],
            "state": None,
            "device_state": None,
            "bio_messages": [],
            "valid_bio": None,
            "success": False,
        }

        try:
            if focus_cluster >= 0:
                self._send_command(ser, tx_state, CMD_SET_FOCUS, pack_cmd_set_focus(focus_cluster))
            self._send_command(ser, tx_state, CMD_SET_HM, pack_cmd_set_hm(0))
            self._send_command(ser, tx_state, CMD_SET_BIO_MS, pack_cmd_set_bio_ms(bio_ms))
        except OSError:
            result["light_summary"] = self._summarize_light(result["light_samples"])
            return result

        timeout = time.monotonic() + timeout_s
        while time.monotonic() < timeout:
            try:
                events = self._poll_events(ser, rx_buffer)
            except OSError:
                break
            if not events:
                continue

            for msg in events:
                msg_type = msg.get("type")
                if msg_type == "state":
                    result["state"] = msg
                    result["device_state"] = msg.get("state")
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
                        result["light_summary"] = self._summarize_light(result["light_samples"])
                        return result
                if msg_type == "light":
                    result["light_samples"].append(msg)
                    lux = msg.get("lux")
                    if isinstance(lux, (int, float)):
                        result["latest_light"] = {
                            "t_ms": msg.get("t_ms"),
                            "valid": msg.get("valid"),
                            "lux": float(lux),
                        }
        result["light_summary"] = self._summarize_light(result["light_samples"])
        return result

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Execute mmWave scan/measure orchestration."""
        mode = kwargs.get("mode", "locate_and_measure")
        if mode not in {"scan", "measure", "locate_and_measure"}:
            return {"error": f"invalid mode '{mode}', use scan, measure, or locate_and_measure"}

        try:
            serial_port = self._resolve_serial_port(kwargs.get("serial_port"))
        except RuntimeError as e:
            logger.warning("mmWave port resolution failed: %s", e)
            return {"error": str(e), "status": "disconnected"}
        try:
            import serial
            import serial.serialutil
        except ModuleNotFoundError as exc:
            return {"error": f"missing dependency: {exc.name or 'pyserial'}"}

        duration_s = float(coerce_float_nonneg(kwargs.get("duration_s", 0), 0.0))
        if mode == "locate_and_measure":
            duration_s = coerce_float_nonneg(
                kwargs.get("duration_s", self.DEFAULT_SCAN_SECONDS), self.DEFAULT_SCAN_SECONDS
            )
            measure_duration = coerce_float_nonneg(
                kwargs.get("measure_duration_s", self.DEFAULT_MEASURE_SECONDS), self.DEFAULT_MEASURE_SECONDS
            )
            do_sweep = bool(kwargs.get("sweep_if_unseen", True))
            targets_ms = coerce_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = coerce_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)
        elif mode == "scan":
            duration_s = coerce_float_nonneg(
                kwargs.get("duration_s", self.DEFAULT_SCAN_SECONDS), self.DEFAULT_SCAN_SECONDS
            )
            measure_duration = 0.0
            do_sweep = bool(kwargs.get("sweep_if_unseen", True))
            targets_ms = coerce_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = coerce_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)
        else:
            # measure
            measure_duration = coerce_float_nonneg(
                kwargs.get("duration_s", self.DEFAULT_MEASURE_SECONDS), self.DEFAULT_MEASURE_SECONDS
            )
            do_sweep = False
            targets_ms = coerce_ms(kwargs.get("targets_ms", self.DEFAULT_TARGETS_MS), self.DEFAULT_TARGETS_MS)
            bio_ms = coerce_ms(kwargs.get("bio_ms", self.DEFAULT_BIO_MS), self.DEFAULT_BIO_MS)

        focus_cluster = coerce_int(kwargs.get("focus_cluster", -1), -1)

        def run_session() -> Dict[str, Any]:
            effective_focus_cluster = focus_cluster
            tx_state = {"seq": 0}
            rx_buffer = bytearray()
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
                        tx_state=tx_state,
                        rx_buffer=rx_buffer,
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
                        tx_state=tx_state,
                        rx_buffer=rx_buffer,
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
            logger.warning("mmWave serial exception on %s: %s", serial_port, e)
            return {"error": f"serial error on {serial_port}: {e}", "status": "disconnected"}
        except OSError as e:
            logger.warning("mmWave OS error on %s: %s", serial_port, e)
            return {"error": f"device I/O error on {serial_port}: {e}", "status": "disconnected"}
        except Exception as e:
            logger.exception("mmWave tool failed")
            return {"error": str(e)}
