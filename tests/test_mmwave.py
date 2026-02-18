"""Tests for the mmWave tool."""

from __future__ import annotations
import sys
import json
import types
from types import SimpleNamespace

import pytest

from healthy_heartrate_breathing.tools.core_tools import ToolDependencies
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave import MmWave


def _json_line(payload: dict[str, object]) -> bytes:
    """Serialize one telemetry payload line."""
    return (json.dumps(payload) + "\n").encode("utf-8")


def _raw_line(payload: str) -> bytes:
    """Serialize raw serial text line."""
    return (payload + "\n").encode("utf-8")


class FakeSerial:
    """Small fake serial port for deterministic mmWave tests."""

    class SerialException(Exception):
        """Fake serial exception type."""

    response_batches: list[list[bytes]] = []
    instances: list["FakeSerial"] = []

    def __init__(self, _port: str, _baud: int, timeout: float = 0.2, responses: list[bytes] | None = None) -> None:
        """Create a fake serial connection with optional deterministic responses."""
        self.port = _port
        self.baudrate = _baud
        self.timeout = timeout
        if responses is not None:
            self._responses = list(responses)
        elif FakeSerial.response_batches:
            self._responses = FakeSerial.response_batches.pop(0)
        else:
            self._responses = []
        self.writes: list[str] = []
        FakeSerial.instances.append(self)

    def write(self, data: bytes) -> None:
        """Capture serial payload writes."""
        self.writes.append(data.decode("utf-8").strip())

    def flush(self) -> None:
        """Flush is a no-op in the fake serial."""
        return None

    def readline(self) -> bytes:
        """Read next queued line or return empty bytes when exhausted."""
        if not self._responses:
            return b""
        return self._responses.pop(0)

    def close(self) -> None:
        """Close hook for consistency with real serial objects."""
        return None

    def __enter__(self) -> "FakeSerial":
        """Context-manager entry."""
        return self

    def __exit__(self, *_exc_info: object) -> bool:
        """Context-manager exit."""
        return False


def _patch_serial_modules(monkeypatch: pytest.MonkeyPatch, responses_batches: list[list[bytes]]) -> None:
    """Inject fake serial modules for mmWave.__call__."""
    FakeSerial.response_batches = [list(lines) for lines in responses_batches]
    FakeSerial.instances = []

    serial_module = types.ModuleType("serial")
    serialutil_module = types.ModuleType("serial.serialutil")
    serialutil_module.SerialException = FakeSerial.SerialException

    serial_module.Serial = FakeSerial
    serial_module.serialutil = serialutil_module

    monkeypatch.setitem(sys.modules, "serial", serial_module)
    monkeypatch.setitem(sys.modules, "serial.serialutil", serialutil_module)


def _deps() -> ToolDependencies:
    """Build minimal tool dependencies for mmWave tests."""
    return ToolDependencies(
        reachy_mini=SimpleNamespace(),
        movement_manager=None,
    )


def test_resolve_serial_port_prefers_explicit() -> None:
    """Explicit serial port always wins over env fallback."""
    tool = MmWave()
    assert tool._resolve_serial_port("/dev/custom") == "/dev/custom"


def test_resolve_serial_port_uses_env_var(monkeypatch: pytest.MonkeyPatch) -> None:
    """Environment variable is used when no explicit port is set."""
    monkeypatch.setenv("MMWAVE_SERIAL_PORT", "/env/port")
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/env/port"


def test_pick_target_selects_closest() -> None:
    """Pick the nearest target among valid telemetry entries."""
    tool = MmWave()

    message = {
        "type": "targets",
        "n": 2,
        "targets": [
            {"cluster": 3, "x": 0.2, "y": 2.0, "r": 2.02, "bearing": 0, "v": 0},
            {"cluster": 5, "x": 0.1, "y": 0.5, "r": 0.51, "bearing": 0, "v": 0},
        ],
    }

    picked = tool._pick_target_from_message(message)
    assert picked is not None
    assert picked["cluster"] == 5


def test_parse_payload_strips_serial_prefix_and_repairs_state() -> None:
    """Monitor-prefixed and malformed state JSON is repaired and parsed."""
    tool = MmWave()

    payload = tool._parse_payload(
        b'20:09:52.663 -> {"type":"state","t_ms":34792PRESENT_FAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":34.44,"dist_new":1}'
    )

    assert payload is not None
    assert payload["type"] == "state"
    assert payload["state"] == "PRESENT_FAR"
    assert payload["t_ms"] == 34792


def test_pick_target_uses_n_targets_when_n_missing() -> None:
    """Targets payloads using n_targets should still be accepted."""
    tool = MmWave()

    message = {
        "type": "targets",
        "t_ms": 35996,
        "n_targets": 2,
        "targets": [
            {"cluster": 9, "x": 0.2, "y": 0.8, "r": 0.83, "bearing": 1.0, "v": 0},
            {"cluster": 11, "x": 0.1, "y": 0.3, "r": 0.32, "bearing": 1.0, "v": 0},
        ],
    }

    picked = tool._pick_target_from_message(message)
    assert picked is not None
    assert picked["cluster"] == 11


@pytest.mark.asyncio
async def test_locate_and_measure_with_full_serial_snippet(monkeypatch: pytest.MonkeyPatch) -> None:
    """End-to-end mixed serial stream should still resolve to a usable vitals result."""
    mixed_stream = [
        _raw_line(
            '20:09:52.663 -> {"type":"state","t_ms":34792PRESENT_FAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":34.44,"dist_new":1}'
        ),
        _raw_line(
            '20:09:52.729 -> {"type":"bio","t_ms":34993,"allowed":1,"valid":0,"br":14.00,"br_new":1,"hr":84.00,"hr_new":0}'
        ),
        _raw_line(
            '20:09:52.827 -> {"type":"targets","t_ms":35093,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.080,"y":0.349,"r":0.358,"bearing":-12.8,"v":0.00},"targets":[{"cluster":0,"x":-0.080,"y":0.349,"r":0.358,"bearing":-12.8,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:53.025 -> {"type":"state","t_ms":35294,"state":"PRESENT_FAR","pose":"SITTING","head_moving":0,"human":0,"n_targets":0,"dist_cm":34.44,"dist_new":0}'
        ),
        _raw_line(
            '20:09:53.125 -> {"type":"targets","t_ms":35394,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.102,"y":0.357,"r":0.372,"bearing":-15.9,"v":0.00},"targets":[{"cluster":0,"x":-0.102,"y":0.357,"r":0.372,"bearing":-15.9,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:53.125 -> {"type":"state","t_ms":35394,"state":"PRESENT_FAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":34.44,"dist_new":1}'
        ),
        _raw_line(
            '20:09:53.421 -> {"type":"targets","t_ms":35695,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.154,"y":0.465,"r":0.490,"bearing":-18.3,"v":0.00},"targets":[{"cluster":0,"x":-0.154,"y":0.465,"r":0.490,"bearing":-18.3,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:53.517 -> {"type":"state","t_ms":35796,"state":"PRESENT_FAR","pose":"SITTING","head_moving":0,"human":0,"n_targets":0,"dist_cm":34.44,"dist_new":0}'
        ),
        _raw_line(
            '20:09:53.615 -> {"type":"state","t_ms":35896,"state":"PRESENT_FAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":34.44,"dist_new":1}'
        ),
        _raw_line(
            '20:09:53.747 -> {"type":"targets","t_ms":35996,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.154,"y":0.465,"r":0.490,"bearing":-18.3,"v":0.00},"targets":[{"cluster":0,"x":-0.154,"y":0.465,"r":0.490,"bearing":-18.3,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:53.747 -> {"type":"bio","t_ms":35996,"allowed":1,"valid":1,"br":14.00,"br_new":1,"hr":79.00,"hr_new":1}'
        ),
        _raw_line(
            '20:09:54.044 -> {"type":"state","t_ms":36302,"state":"PRESENT_FAR","pose":"SITTING","head_moving":0,"human":0,"n_targets":0,"dist_cm":34.44,"dist_new":0}'
        ),
        _raw_line(
            '20:09:54.143 -> {"type":"targets","t_ms":36402,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00},"targets":[{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:54.143 -> {"type":"state","t_ms":36402,"state":"STILL_NEAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":40.18,"dist_new":1}'
        ),
        _raw_line(
            '20:09:54.441 -> {"type":"targets","t_ms":36703,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00},"targets":[{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:54.637 -> {"type":"state","t_ms":36904,"state":"STILL_NEAR","pose":"SITTING","head_moving":0,"human":0,"n_targets":0,"dist_cm":40.18,"dist_new":0}'
        ),
        _raw_line(
            '20:09:54.736 -> {"type":"targets","t_ms":37004,"n":1,"forced_focus":-1,"focus":{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00},"targets":[{"cluster":0,"x":-0.183,"y":0.525,"r":0.556,"bearing":-19.2,"v":0.00}]}'
        ),
        _raw_line(
            '20:09:54.736 -> {"type":"state","t_ms":37004,"state":"STILL_NEAR","pose":"SITTING","head_moving":0,"human":1,"n_targets":1,"dist_cm":40.18,"dist_new":1}'
        ),
        _raw_line(
            '20:09:54.736 -> {"type":"bio","t_ms":37004,"allowed":1,"valid":0,"br":10.00,"br_new":1,"hr":76.00,"hr_new":0}'
        ),
    ]

    _patch_serial_modules(monkeypatch, [mixed_stream])

    tool = MmWave()
    response = await tool(
        _deps(),
        mode="locate_and_measure",
        duration_s=0.2,
        measure_duration_s=0.2,
        sweep_if_unseen=False,
    )

    assert response["status"] == "ok"
    assert response["scan"]["latest_target"] is not None
    assert response["scan"]["latest_target"]["cluster"] == 0
    assert response["measure"]["success"]
    assert response["measure"]["valid_bio"]["heart_rate_bpm"] == 79.0


@pytest.mark.asyncio
async def test_measure_mode_requests_focus_and_returns_bio(monkeypatch: pytest.MonkeyPatch) -> None:
    """Measure mode issues HM/BIO commands and returns parsed bio."""
    responses = [
        _json_line(
            {
                "type": "bio",
                "t_ms": 100,
                "allowed": 1,
                "valid": 1,
                "br": 12.4,
                "br_new": 1,
                "hr": 69.0,
                "hr_new": 1,
            }
        ),
    ]

    _patch_serial_modules(monkeypatch, [responses])

    tool = MmWave()
    response = await tool(_deps(), mode="measure", focus_cluster=4, duration_s=0.2)
    assert response["status"] == "ok"

    serial = FakeSerial.instances[0]
    assert any(cmd == "FOCUS=4" for cmd in serial.writes)
    assert "HM=0" in serial.writes
    assert response["measure"]["success"]


@pytest.mark.asyncio
async def test_locate_and_measure_uses_scan_cluster_for_focus(monkeypatch: pytest.MonkeyPatch) -> None:
    """locate_and_measure should lock onto last scanned cluster before measuring."""
    scan_message = _json_line(
        {
            "type": "targets",
            "n": 1,
            "targets": [{"cluster": 7, "x": 0.1, "y": 1.0, "r": 1.01, "bearing": 1.0, "v": 0}],
        }
    )
    bio_message = _json_line(
        {
            "type": "bio",
            "t_ms": 150,
            "allowed": 1,
            "valid": 1,
            "br": 13.1,
            "br_new": 1,
            "hr": 70.0,
            "hr_new": 1,
        }
    )

    _patch_serial_modules(monkeypatch, [[scan_message, bio_message]])

    tool = MmWave()
    response = await tool(
        _deps(),
        mode="locate_and_measure",
        duration_s=0.1,
        measure_duration_s=0.1,
        sweep_if_unseen=False,
    )
    assert response["status"] == "ok"
    assert response["scan"]["latest_target"]["cluster"] == 7

    serial = FakeSerial.instances[0]
    assert any(cmd == "FOCUS=7" for cmd in serial.writes)
    assert any(cmd == "HM=1" for cmd in serial.writes)
    assert any(cmd == "HM=0" for cmd in serial.writes)


def test_scan_mode_updates_latest_target_from_targets() -> None:
    """Low-level _scan_sync extracts the closest target from targets telemetry."""
    tool = MmWave()
    ser = FakeSerial(
        "/dev/test",
        115200,
        responses=[
            _json_line(
                {
                    "type": "targets",
                    "n": 1,
                    "targets": [
                        {"cluster": 9, "x": 0.2, "y": 0.8, "r": 0.83, "bearing": 1.0, "v": 0},
                        {"cluster": 11, "x": 0.1, "y": 0.3, "r": 0.32, "bearing": 1.0, "v": 0},
                    ],
                }
            )
        ],
    )

    scan = tool._scan_sync(
        ser,
        deps=_deps(),
        duration_s=0.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert scan["latest_target"] is not None
    assert scan["latest_target"]["cluster"] == 11


@pytest.mark.asyncio
async def test_invalid_mode_returns_error() -> None:
    """Unsupported modes should return a structured error."""
    tool = MmWave()
    response = await tool(_deps(), mode="dance")

    assert response["error"] == "invalid mode 'dance', use scan, measure, or locate_and_measure"
