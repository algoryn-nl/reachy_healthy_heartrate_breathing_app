"""Tests for the binary mmWave protocol and tool integration."""
# ruff: noqa: D102,D103,D105,D107

from __future__ import annotations
import sys
import types
import struct
from types import SimpleNamespace
from typing import Any

import pytest

from healthy_heartrate_breathing.tools.core_tools import ToolDependencies
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave import MmWave
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    EVT_BIO,
    EVT_PONG,
    EVT_LIGHT,
    EVT_STATE,
    CMD_SET_HM,
    ERR_BAD_LEN,
    EVT_TARGETS,
    CMD_SET_FOCUS,
    PROTO_VERSION,
    CMD_SET_BIO_MS,
    CMD_SET_TARGETS_MS,
    ProtocolError,
    cobs_encode,
    decode_event,
    decode_frame,
    encode_frame,
    crc16_ccitt_false,
    extract_encoded_frames,
)


def _pack_state(
    *,
    t_ms: int,
    state_enum: int,
    pose_enum: int,
    head_moving: int,
    human: int,
    n_targets: int,
    dist_new: int,
    dist_mm: int,
) -> bytes:
    return struct.pack(
        "<IBBBBBBH",
        t_ms,
        state_enum,
        pose_enum,
        head_moving,
        human,
        n_targets,
        dist_new,
        dist_mm,
    )


def _pack_bio(*, t_ms: int, allowed: int, valid: int, br_new: int, hr_new: int, br_centi: int, hr_centi: int) -> bytes:
    return struct.pack("<IBBBBHH", t_ms, allowed, valid, br_new, hr_new, br_centi, hr_centi)


def _pack_targets_single(
    *,
    t_ms: int,
    forced_focus: int,
    cluster: int,
    x_mm: int,
    y_mm: int,
    r_mm: int,
    bearing_cdeg: int,
    v_x10: int,
    truncated: bool = False,
) -> bytes:
    flags = 0x01
    if truncated:
        flags |= 0x02
    header = struct.pack(
        "<IhhhhHhhBB",
        t_ms,
        forced_focus,
        cluster,
        x_mm,
        y_mm,
        r_mm,
        bearing_cdeg,
        v_x10,
        flags,
        1,
    )
    target = struct.pack("<hhhHhh", cluster, x_mm, y_mm, r_mm, bearing_cdeg, v_x10)
    return header + target


def _pack_light(*, t_ms: int, valid: int, lux: float) -> bytes:
    return struct.pack("<IBf", t_ms, valid, lux)


class FakeSerial:
    """Small fake serial port for deterministic binary protocol tests."""

    class SerialException(Exception):
        """Fake serial exception type."""

    response_batches: list[dict[str, Any]] = []
    instances: list["FakeSerial"] = []

    def __init__(
        self,
        _port: str,
        _baud: int,
        timeout: float = 0.2,
        responses: bytes | None = None,
        read_chunk_size: int | None = None,
    ) -> None:
        self.port = _port
        self.baudrate = _baud
        self.timeout = timeout
        if responses is not None:
            self._buffer = bytearray(responses)
            self._read_chunk_size = read_chunk_size
        elif FakeSerial.response_batches:
            config = FakeSerial.response_batches.pop(0)
            self._buffer = bytearray(config.get("bytes", b""))
            self._read_chunk_size = config.get("read_chunk_size")
        else:
            self._buffer = bytearray()
            self._read_chunk_size = None
        self.writes: list[bytes] = []
        FakeSerial.instances.append(self)

    @property
    def in_waiting(self) -> int:
        return len(self._buffer)

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))

    def flush(self) -> None:
        return None

    def read(self, size: int = 1) -> bytes:
        if not self._buffer:
            return b""
        n = min(size, len(self._buffer))
        if self._read_chunk_size is not None:
            n = min(n, self._read_chunk_size)
        out = bytes(self._buffer[:n])
        del self._buffer[:n]
        return out

    def close(self) -> None:
        return None

    def __enter__(self) -> "FakeSerial":
        return self

    def __exit__(self, *_exc_info: object) -> bool:
        return False


def _patch_serial_modules(
    monkeypatch: pytest.MonkeyPatch,
    responses_batches: list[dict[str, Any]],
) -> None:
    FakeSerial.response_batches = list(responses_batches)
    FakeSerial.instances = []

    serial_module = types.ModuleType("serial")
    serialutil_module = types.ModuleType("serial.serialutil")
    serialutil_module.SerialException = FakeSerial.SerialException
    serial_module.Serial = FakeSerial
    serial_module.serialutil = serialutil_module

    monkeypatch.setitem(sys.modules, "serial", serial_module)
    monkeypatch.setitem(sys.modules, "serial.serialutil", serialutil_module)


def _deps() -> ToolDependencies:
    return ToolDependencies(
        reachy_mini=SimpleNamespace(),
        movement_manager=None,
    )


def _decode_written_frames(serial: FakeSerial) -> list[tuple[int, int, int, bytes]]:
    decoded = []
    for write in serial.writes:
        assert write.endswith(b"\x00")
        decoded.append(decode_frame(write[:-1]))
    return decoded


def test_frame_roundtrip_and_cobs_extraction() -> None:
    payload = b"\x00\x01\x02\x00"
    frame_1 = encode_frame(EVT_PONG, payload, seq=7)
    frame_2 = encode_frame(EVT_STATE, _pack_state(t_ms=42, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344), seq=8)

    buffer = bytearray()
    parts = [frame_1[:4], frame_1[4:] + frame_2[:5], frame_2[5:]]
    extracted: list[bytes] = []
    for part in parts:
        buffer.extend(part)
        extracted.extend(extract_encoded_frames(buffer))

    assert len(extracted) == 2

    version_1, msg_type_1, seq_1, payload_1 = decode_frame(extracted[0])
    assert version_1 == PROTO_VERSION
    assert msg_type_1 == EVT_PONG
    assert seq_1 == 7
    assert payload_1 == payload

    version_2, msg_type_2, seq_2, payload_2 = decode_frame(extracted[1])
    assert version_2 == PROTO_VERSION
    assert msg_type_2 == EVT_STATE
    assert seq_2 == 8
    assert decode_event(msg_type_2, payload_2)["state"] == "PRESENT_FAR"


def test_decode_frame_rejects_bad_length() -> None:
    raw = struct.pack("<BBHH", PROTO_VERSION, EVT_BIO, 1, 30) + b"\x01\x02"
    crc = crc16_ccitt_false(raw)
    raw += struct.pack("<H", crc)
    encoded = cobs_encode(raw)

    with pytest.raises(ProtocolError):
        decode_frame(encoded)


def test_unknown_event_is_reported_as_unknown() -> None:
    unknown = decode_event(0xFE, b"\x01\x02")
    assert unknown["type"] == "unknown"
    assert unknown["msg_type"] == 0xFE


def test_bio_null_sentinel_decodes_to_none() -> None:
    payload = _pack_bio(t_ms=123, allowed=0, valid=0, br_new=0, hr_new=0, br_centi=0xFFFF, hr_centi=0xFFFF)
    event = decode_event(EVT_BIO, payload)
    assert event["br"] is None
    assert event["hr"] is None


def test_light_valid_decodes_float_lux() -> None:
    payload = _pack_light(t_ms=456, valid=1, lux=123.45)
    event = decode_event(EVT_LIGHT, payload)
    assert event["type"] == "light"
    assert event["valid"] == 1
    assert event["lux"] == pytest.approx(123.45, rel=1e-6)


def test_light_invalid_nan_decodes_to_none() -> None:
    payload = _pack_light(t_ms=789, valid=0, lux=float("nan"))
    event = decode_event(EVT_LIGHT, payload)
    assert event["type"] == "light"
    assert event["valid"] == 0
    assert event["lux"] is None


def test_light_bad_payload_len_raises_protocol_error() -> None:
    with pytest.raises(ProtocolError):
        decode_event(EVT_LIGHT, b"\x01\x02")


@pytest.mark.asyncio
async def test_measure_mode_sends_binary_commands_and_parses_bio(monkeypatch: pytest.MonkeyPatch) -> None:
    bio_payload = _pack_bio(t_ms=100, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1240, hr_centi=6900)
    responses = encode_frame(EVT_BIO, bio_payload, seq=1)
    _patch_serial_modules(monkeypatch, [{"bytes": responses}])

    tool = MmWave()
    response = await tool(_deps(), mode="measure", focus_cluster=4, duration_s=0.1)

    assert response["status"] == "ok"
    assert response["measure"]["valid_bio"]["heart_rate_bpm"] == 69.0
    assert response["measure"]["valid_bio"]["breath_rate_bpm"] == 12.4

    serial = FakeSerial.instances[0]
    sent = _decode_written_frames(serial)
    msg_types = [msg_type for _version, msg_type, _seq, _payload in sent]
    assert msg_types[:3] == [CMD_SET_FOCUS, CMD_SET_HM, CMD_SET_BIO_MS]


@pytest.mark.asyncio
async def test_locate_and_measure_partial_read_and_focus_lock(monkeypatch: pytest.MonkeyPatch) -> None:
    targets_payload = _pack_targets_single(
        t_ms=35093,
        forced_focus=-1,
        cluster=7,
        x_mm=100,
        y_mm=1000,
        r_mm=1005,
        bearing_cdeg=100,
        v_x10=0,
    )
    bio_payload = _pack_bio(t_ms=35996, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1310, hr_centi=7000)

    stream = encode_frame(EVT_TARGETS, targets_payload, seq=2) + encode_frame(EVT_BIO, bio_payload, seq=3)
    _patch_serial_modules(monkeypatch, [{"bytes": stream, "read_chunk_size": 3}])

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
    sent = _decode_written_frames(serial)
    msg_types = [msg_type for _version, msg_type, _seq, _payload in sent]
    assert msg_types[0:2] == [CMD_SET_HM, CMD_SET_TARGETS_MS]
    assert CMD_SET_FOCUS in msg_types


@pytest.mark.asyncio
async def test_resync_after_corrupted_frame(monkeypatch: pytest.MonkeyPatch) -> None:
    bad_frame = bytearray(encode_frame(EVT_STATE, _pack_state(t_ms=10, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=350), seq=10))
    bad_frame[-3] ^= 0x11  # break crc while keeping delimiter

    good_bio = encode_frame(EVT_BIO, _pack_bio(t_ms=20, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1400, hr_centi=7900), seq=11)
    stream = bytes(bad_frame) + good_bio
    _patch_serial_modules(monkeypatch, [{"bytes": stream}])

    tool = MmWave()
    response = await tool(_deps(), mode="measure", duration_s=0.1)

    assert response["status"] == "ok"
    assert response["measure"]["valid_bio"]["heart_rate_bpm"] == 79.0


@pytest.mark.asyncio
async def test_unsupported_version_frame_is_ignored(monkeypatch: pytest.MonkeyPatch) -> None:
    bad_version = encode_frame(
        EVT_BIO,
        _pack_bio(t_ms=30, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1200, hr_centi=6800),
        seq=12,
        version=2,
    )
    good_version = encode_frame(
        EVT_BIO,
        _pack_bio(t_ms=31, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1210, hr_centi=6810),
        seq=13,
    )
    _patch_serial_modules(monkeypatch, [{"bytes": bad_version + good_version}])

    tool = MmWave()
    response = await tool(_deps(), mode="measure", duration_s=0.1)

    assert response["status"] == "ok"
    assert response["measure"]["valid_bio"]["heart_rate_bpm"] == 68.1


@pytest.mark.asyncio
async def test_locate_and_measure_ignores_light_event_in_mixed_stream(monkeypatch: pytest.MonkeyPatch) -> None:
    targets_payload = _pack_targets_single(
        t_ms=1000,
        forced_focus=-1,
        cluster=3,
        x_mm=20,
        y_mm=800,
        r_mm=801,
        bearing_cdeg=150,
        v_x10=0,
    )
    light_payload = _pack_light(t_ms=1100, valid=1, lux=42.0)
    bio_payload = _pack_bio(t_ms=1200, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1325, hr_centi=7750)
    stream = (
        encode_frame(EVT_TARGETS, targets_payload, seq=100)
        + encode_frame(EVT_LIGHT, light_payload, seq=101)
        + encode_frame(EVT_BIO, bio_payload, seq=102)
    )
    _patch_serial_modules(monkeypatch, [{"bytes": stream, "read_chunk_size": 4}])

    tool = MmWave()
    response = await tool(
        _deps(),
        mode="locate_and_measure",
        duration_s=0.1,
        measure_duration_s=0.1,
        sweep_if_unseen=False,
    )

    assert response["status"] == "ok"
    assert response["scan"]["latest_target"]["cluster"] == 3
    assert response["measure"]["success"] is True
    assert response["measure"]["valid_bio"]["heart_rate_bpm"] == 77.5


@pytest.mark.asyncio
async def test_locate_and_measure_with_full_mixed_stream(monkeypatch: pytest.MonkeyPatch) -> None:
    stream = b"".join(
        [
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=34792,
                    state_enum=2,
                    pose_enum=1,
                    head_moving=0,
                    human=1,
                    n_targets=1,
                    dist_new=1,
                    dist_mm=344,
                ),
                seq=20,
            ),
            encode_frame(EVT_BIO, _pack_bio(t_ms=34993, allowed=1, valid=0, br_new=1, hr_new=0, br_centi=1400, hr_centi=8400), seq=21),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=35093,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-80,
                    y_mm=349,
                    r_mm=358,
                    bearing_cdeg=-1280,
                    v_x10=0,
                ),
                seq=22,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=35294, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344), seq=23),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=35394,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-102,
                    y_mm=357,
                    r_mm=372,
                    bearing_cdeg=-1590,
                    v_x10=0,
                ),
                seq=24,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=35394, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344), seq=25),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=35695,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-154,
                    y_mm=465,
                    r_mm=490,
                    bearing_cdeg=-1830,
                    v_x10=0,
                ),
                seq=26,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=35796, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344), seq=27),
            encode_frame(EVT_STATE, _pack_state(t_ms=35896, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344), seq=28),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=35996,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-154,
                    y_mm=465,
                    r_mm=490,
                    bearing_cdeg=-1830,
                    v_x10=0,
                ),
                seq=29,
            ),
            encode_frame(EVT_BIO, _pack_bio(t_ms=35996, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1400, hr_centi=7900), seq=30),
            encode_frame(EVT_STATE, _pack_state(t_ms=36302, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344), seq=31),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=36402,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-183,
                    y_mm=525,
                    r_mm=556,
                    bearing_cdeg=-1920,
                    v_x10=0,
                ),
                seq=32,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=36402, state_enum=4, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=401), seq=33),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=36703,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-183,
                    y_mm=525,
                    r_mm=556,
                    bearing_cdeg=-1920,
                    v_x10=0,
                ),
                seq=34,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=36904, state_enum=4, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=401), seq=35),
            encode_frame(
                EVT_TARGETS,
                _pack_targets_single(
                    t_ms=37004,
                    forced_focus=-1,
                    cluster=0,
                    x_mm=-183,
                    y_mm=525,
                    r_mm=556,
                    bearing_cdeg=-1920,
                    v_x10=0,
                ),
                seq=36,
            ),
            encode_frame(EVT_STATE, _pack_state(t_ms=37004, state_enum=4, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=401), seq=37),
            encode_frame(EVT_BIO, _pack_bio(t_ms=37004, allowed=1, valid=0, br_new=1, hr_new=0, br_centi=1000, hr_centi=7600), seq=38),
        ]
    )
    _patch_serial_modules(monkeypatch, [{"bytes": stream, "read_chunk_size": 7}])

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
async def test_invalid_mode_returns_error() -> None:
    tool = MmWave()
    response = await tool(_deps(), mode="dance")
    assert response["error"] == "invalid mode 'dance', use scan, measure, or locate_and_measure"


def test_bad_len_error_code_constant_is_stable() -> None:
    assert ERR_BAD_LEN == 2
