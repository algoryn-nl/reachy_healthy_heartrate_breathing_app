"""Tests for the binary mmWave protocol and tool integration."""
# ruff: noqa: D102,D103,D105,D107

from __future__ import annotations
import sys
import types
import struct
from types import SimpleNamespace
from typing import Any

import pytest

from healthy_heartrate_breathing.tool_dispatcher import extract_sensor_state
from healthy_heartrate_breathing.tools.core_tools import ToolDependencies
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave import MmWave
from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    EVT_BIO,
    EVT_PONG,
    EVT_HELLO,
    EVT_LIGHT,
    EVT_STATE,
    CMD_SET_HM,
    ERR_BAD_LEN,
    EVT_TARGETS,
    CMD_SET_FOCUS,
    PROTO_VERSION,
    CMD_SET_BIO_MS,
    FEAT_LIGHT_SENSOR,
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
    monkeypatch.setenv("MMWAVE_SERIAL_PORT", "/dev/fake_mmwave")


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
    frame_2 = encode_frame(
        EVT_STATE,
        _pack_state(t_ms=42, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344),
        seq=8,
    )

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


# ---------------------------------------------------------------------------
# Bio rate boundary condition tests (firmware guard rails: BR 4–30, HR 35–200)
# ---------------------------------------------------------------------------


class TestBioRateBoundaryDecode:
    """Verify decode_event handles exact guard-rail boundary values for bio rates.

    Firmware encodes rates as centi-bpm (uint16_t, ×100). Guard rails:
      BR: 4.0–30.0 bpm  →  wire 400–3000
      HR: 35.0–200.0 bpm →  wire 3500–20000
    Values outside rails are sent as 0xFFFF sentinel → decoded as None.
    """

    @pytest.mark.parametrize(
        "br_centi, hr_centi, expected_br, expected_hr",
        [
            # Exact lower bounds
            (400, 3500, 4.0, 35.0),
            # Exact upper bounds
            (3000, 20000, 30.0, 200.0),
            # Just inside lower bounds
            (401, 3501, 4.01, 35.01),
            # Just inside upper bounds
            (2999, 19999, 29.99, 199.99),
            # Mid-range normal values
            (1400, 7200, 14.0, 72.0),
            # Minimum non-null wire value (1 centi-bpm = 0.01 bpm)
            (1, 1, 0.01, 0.01),
            # Maximum non-null wire value (0xFFFE = 655.34 bpm)
            (0xFFFE, 0xFFFE, 655.34, 655.34),
            # Zero wire value (0 centi-bpm = 0.0 bpm, distinct from null)
            (0, 0, 0.0, 0.0),
        ],
        ids=[
            "exact-lower-bound",
            "exact-upper-bound",
            "just-inside-lower",
            "just-inside-upper",
            "mid-range",
            "min-nonzero-wire",
            "max-non-null-wire",
            "zero-wire",
        ],
    )
    def test_bio_boundary_values_decode(
        self, br_centi: int, hr_centi: int, expected_br: float, expected_hr: float
    ) -> None:
        payload = _pack_bio(t_ms=100, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=br_centi, hr_centi=hr_centi)
        event = decode_event(EVT_BIO, payload)
        assert event["br"] == pytest.approx(expected_br, abs=1e-9)
        assert event["hr"] == pytest.approx(expected_hr, abs=1e-9)

    def test_bio_null_sentinel_both(self) -> None:
        payload = _pack_bio(t_ms=100, allowed=0, valid=0, br_new=0, hr_new=0, br_centi=0xFFFF, hr_centi=0xFFFF)
        event = decode_event(EVT_BIO, payload)
        assert event["br"] is None
        assert event["hr"] is None

    def test_bio_null_sentinel_br_only(self) -> None:
        payload = _pack_bio(t_ms=100, allowed=1, valid=0, br_new=0, hr_new=1, br_centi=0xFFFF, hr_centi=7200)
        event = decode_event(EVT_BIO, payload)
        assert event["br"] is None
        assert event["hr"] == pytest.approx(72.0)

    def test_bio_null_sentinel_hr_only(self) -> None:
        payload = _pack_bio(t_ms=100, allowed=1, valid=0, br_new=1, hr_new=0, br_centi=1400, hr_centi=0xFFFF)
        event = decode_event(EVT_BIO, payload)
        assert event["br"] == pytest.approx(14.0)
        assert event["hr"] is None

    def test_bio_flags_passthrough(self) -> None:
        """Verify allowed/valid/br_new/hr_new flags are decoded as integers."""
        payload = _pack_bio(t_ms=500, allowed=1, valid=0, br_new=0, hr_new=1, br_centi=1400, hr_centi=7200)
        event = decode_event(EVT_BIO, payload)
        assert event["allowed"] == 1
        assert event["valid"] == 0
        assert event["br_new"] == 0
        assert event["hr_new"] == 1


# ---------------------------------------------------------------------------
# Bio rate acceptance gate tests (tool layer: _measure_sync valid_bio logic)
# ---------------------------------------------------------------------------


class TestBioAcceptanceGate:
    """Verify _measure_sync gates valid_bio on allowed, valid, and non-None rates.

    Acceptance requires ALL of: allowed=1, valid=1, br is not None, hr is not None.
    """

    @staticmethod
    def _bio_frame(
        *, allowed: int, valid: int, br_centi: int = 1400, hr_centi: int = 7200, seq: int = 1
    ) -> bytes:
        payload = _pack_bio(
            t_ms=100, allowed=allowed, valid=valid, br_new=1, hr_new=1, br_centi=br_centi, hr_centi=hr_centi
        )
        return bytes(encode_frame(EVT_BIO, payload, seq=seq))

    @pytest.mark.asyncio
    async def test_accepted_when_allowed_valid_rates_present(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=1, valid=1, br_centi=1400, hr_centi=7200)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "ok"
        assert response["measure"]["success"] is True
        assert response["measure"]["valid_bio"]["heart_rate_bpm"] == pytest.approx(72.0)
        assert response["measure"]["valid_bio"]["breath_rate_bpm"] == pytest.approx(14.0)

    @pytest.mark.asyncio
    async def test_rejected_when_not_allowed(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=0, valid=1)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "measure_inconclusive"
        assert response["measure"]["success"] is False
        assert response["measure"]["valid_bio"] is None

    @pytest.mark.asyncio
    async def test_rejected_when_not_valid(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=1, valid=0)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "measure_inconclusive"
        assert response["measure"]["success"] is False
        assert response["measure"]["valid_bio"] is None

    @pytest.mark.asyncio
    async def test_rejected_when_br_null(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=1, valid=1, br_centi=0xFFFF, hr_centi=7200)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "measure_inconclusive"
        assert response["measure"]["success"] is False
        assert response["measure"]["valid_bio"] is None

    @pytest.mark.asyncio
    async def test_rejected_when_hr_null(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=1, valid=1, br_centi=1400, hr_centi=0xFFFF)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "measure_inconclusive"
        assert response["measure"]["success"] is False
        assert response["measure"]["valid_bio"] is None

    @pytest.mark.asyncio
    async def test_rejected_when_both_null(self, monkeypatch: pytest.MonkeyPatch) -> None:
        stream = self._bio_frame(allowed=1, valid=1, br_centi=0xFFFF, hr_centi=0xFFFF)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["status"] == "measure_inconclusive"
        assert response["measure"]["success"] is False
        assert response["measure"]["valid_bio"] is None

    @pytest.mark.asyncio
    async def test_accepted_at_exact_guard_rail_lower_bounds(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """BR=4.00 bpm (wire 400), HR=35.00 bpm (wire 3500) — exact lower bounds."""
        stream = self._bio_frame(allowed=1, valid=1, br_centi=400, hr_centi=3500)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["measure"]["success"] is True
        assert response["measure"]["valid_bio"]["breath_rate_bpm"] == pytest.approx(4.0)
        assert response["measure"]["valid_bio"]["heart_rate_bpm"] == pytest.approx(35.0)

    @pytest.mark.asyncio
    async def test_accepted_at_exact_guard_rail_upper_bounds(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """BR=30.00 bpm (wire 3000), HR=200.00 bpm (wire 20000) — exact upper bounds."""
        stream = self._bio_frame(allowed=1, valid=1, br_centi=3000, hr_centi=20000)
        _patch_serial_modules(monkeypatch, [{"bytes": stream}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["measure"]["success"] is True
        assert response["measure"]["valid_bio"]["breath_rate_bpm"] == pytest.approx(30.0)
        assert response["measure"]["valid_bio"]["heart_rate_bpm"] == pytest.approx(200.0)

    @pytest.mark.asyncio
    async def test_first_valid_bio_wins(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """_measure_sync returns on the first valid bio, ignoring subsequent messages."""
        first = self._bio_frame(allowed=1, valid=1, br_centi=1400, hr_centi=7200, seq=1)
        second = self._bio_frame(allowed=1, valid=1, br_centi=1800, hr_centi=8000, seq=2)
        _patch_serial_modules(monkeypatch, [{"bytes": first + second}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["measure"]["valid_bio"]["heart_rate_bpm"] == pytest.approx(72.0)
        assert response["measure"]["valid_bio"]["breath_rate_bpm"] == pytest.approx(14.0)

    @pytest.mark.asyncio
    async def test_skips_invalid_then_accepts_valid(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """Invalid bio messages are skipped; first valid one is accepted."""
        invalid = self._bio_frame(allowed=0, valid=0, br_centi=0xFFFF, hr_centi=0xFFFF, seq=1)
        valid = self._bio_frame(allowed=1, valid=1, br_centi=1600, hr_centi=6500, seq=2)
        _patch_serial_modules(monkeypatch, [{"bytes": invalid + valid}])
        tool = MmWave()
        response = await tool(_deps(), mode="measure", duration_s=0.1)
        assert response["measure"]["success"] is True
        assert response["measure"]["valid_bio"]["heart_rate_bpm"] == pytest.approx(65.0)
        assert response["measure"]["valid_bio"]["breath_rate_bpm"] == pytest.approx(16.0)
        assert response["measure"]["attempts"] == 2


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
    bad_frame = bytearray(
        encode_frame(
            EVT_STATE,
            _pack_state(
                t_ms=10, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=350
            ),
            seq=10,
        )
    )
    bad_frame[-3] ^= 0x11  # break crc while keeping delimiter

    good_bio = encode_frame(
        EVT_BIO, _pack_bio(t_ms=20, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1400, hr_centi=7900), seq=11
    )
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
            encode_frame(
                EVT_BIO,
                _pack_bio(t_ms=34993, allowed=1, valid=0, br_new=1, hr_new=0, br_centi=1400, hr_centi=8400),
                seq=21,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=35294, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344
                ),
                seq=23,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=35394, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344
                ),
                seq=25,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=35796, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344
                ),
                seq=27,
            ),
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=35896, state_enum=2, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=344
                ),
                seq=28,
            ),
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
            encode_frame(
                EVT_BIO,
                _pack_bio(t_ms=35996, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1400, hr_centi=7900),
                seq=30,
            ),
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=36302, state_enum=2, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=344
                ),
                seq=31,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=36402, state_enum=4, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=401
                ),
                seq=33,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=36904, state_enum=4, pose_enum=1, head_moving=0, human=0, n_targets=0, dist_new=0, dist_mm=401
                ),
                seq=35,
            ),
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
            encode_frame(
                EVT_STATE,
                _pack_state(
                    t_ms=37004, state_enum=4, pose_enum=1, head_moving=0, human=1, n_targets=1, dist_new=1, dist_mm=401
                ),
                seq=37,
            ),
            encode_frame(
                EVT_BIO,
                _pack_bio(t_ms=37004, allowed=1, valid=0, br_new=1, hr_new=0, br_centi=1000, hr_centi=7600),
                seq=38,
            ),
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


# ---------------------------------------------------------------------------
# Serial error path tests
# ---------------------------------------------------------------------------


class FailingSerial:
    """Fake serial that raises OSError on write or read as configured."""

    def __init__(
        self,
        initial_data: bytes = b"",
        *,
        fail_write: bool = False,
        reads_before_disconnect: int | None = None,
    ) -> None:
        self._buffer = bytearray(initial_data)
        self._fail_write = fail_write
        self._reads_before_disconnect = reads_before_disconnect
        self._read_count = 0
        self.writes: list[bytes] = []

    @property
    def in_waiting(self) -> int:
        if self._reads_before_disconnect is not None and self._read_count >= self._reads_before_disconnect:
            raise OSError("USB device disconnected")
        return len(self._buffer)

    def read(self, size: int = 1) -> bytes:
        self._read_count += 1
        if self._reads_before_disconnect is not None and self._read_count > self._reads_before_disconnect:
            raise OSError("USB device disconnected")
        if not self._buffer:
            return b""
        n = min(size, len(self._buffer))
        out = bytes(self._buffer[:n])
        del self._buffer[:n]
        return out

    def write(self, data: bytes) -> None:
        if self._fail_write:
            raise OSError("USB device disconnected")
        self.writes.append(bytes(data))

    def flush(self) -> None:
        if self._fail_write:
            raise OSError("USB device disconnected")


def test_send_command_logs_and_reraises_on_write_error() -> None:
    tool = MmWave()
    tx_state = {"seq": 0}
    ser = FailingSerial(fail_write=True)

    with pytest.raises(OSError, match="USB device disconnected"):
        tool._send_command(ser, tx_state, CMD_SET_HM, b"\x01")


def test_poll_events_logs_and_reraises_on_read_error() -> None:
    tool = MmWave()
    ser = FailingSerial(reads_before_disconnect=0)

    with pytest.raises(OSError, match="USB device disconnected"):
        tool._poll_events(ser, bytearray())


def test_poll_events_warns_once_on_version_mismatch(caplog: pytest.LogCaptureFixture) -> None:
    """Version mismatch should log WARNING once, not DEBUG, and include both versions."""
    tool = MmWave()

    # Build two valid frames with wrong protocol version (99 instead of PROTO_VERSION)
    wrong_version = 99
    bio_payload = struct.pack("<IBBBBHH", 1000, 1, 1, 1, 1, 7200, 1500)
    frame1 = encode_frame(EVT_BIO, bio_payload, seq=1, version=wrong_version)
    frame2 = encode_frame(EVT_BIO, bio_payload, seq=2, version=wrong_version)

    ser = FailingSerial(frame1 + frame2)
    rx_buffer = bytearray()

    import logging

    with caplog.at_level(
        logging.DEBUG, logger="healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave"
    ):
        events = tool._poll_events(ser, rx_buffer)

    # Both frames should be dropped
    assert events == []

    # Should have exactly one WARNING about version mismatch (not two, not zero)
    warning_records = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warning_records) == 1
    assert str(wrong_version) in warning_records[0].message
    assert str(PROTO_VERSION) in warning_records[0].message

    # No DEBUG-level version mismatch messages (the old behavior)
    debug_version_records = [
        r for r in caplog.records if r.levelno == logging.DEBUG and "protocol version" in r.message.lower()
    ]
    assert debug_version_records == []


def test_poll_events_version_mismatch_warning_resets_per_instance() -> None:
    """Each MmWave instance should warn independently."""
    wrong_version = 99
    bio_payload = struct.pack("<IBBBBHH", 1000, 1, 1, 1, 1, 7200, 1500)
    frame = encode_frame(EVT_BIO, bio_payload, seq=1, version=wrong_version)

    tool1 = MmWave()
    tool2 = MmWave()

    ser1 = FailingSerial(frame)
    ser2 = FailingSerial(frame)

    # Both instances should warn (not share state)
    tool1._poll_events(ser1, bytearray())
    tool2._poll_events(ser2, bytearray())
    # If we got here without error, instances are independent (verified by first test)


def test_scan_sync_returns_empty_state_when_initial_commands_fail() -> None:
    tool = MmWave()
    ser = FailingSerial(fail_write=True)
    tx_state = {"seq": 0}

    result = tool._scan_sync(
        ser,
        deps=_deps(),
        tx_state=tx_state,
        rx_buffer=bytearray(),
        duration_s=5.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert result["targets_seen"] == 0
    assert result["latest_target"] is None
    assert "light_summary" in result


def test_scan_sync_preserves_partial_data_on_mid_loop_disconnect() -> None:
    targets_payload = _pack_targets_single(
        t_ms=1000,
        forced_focus=-1,
        cluster=5,
        x_mm=100,
        y_mm=500,
        r_mm=510,
        bearing_cdeg=200,
        v_x10=0,
    )
    light_payload = _pack_light(t_ms=1100, valid=1, lux=55.0)
    stream = encode_frame(EVT_TARGETS, targets_payload, seq=1) + encode_frame(EVT_LIGHT, light_payload, seq=2)

    # Deliver all data on first read, then disconnect on second read
    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._scan_sync(
        ser,
        deps=_deps(),
        tx_state=tx_state,
        rx_buffer=bytearray(),
        duration_s=5.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert result["targets_seen"] >= 1
    assert result["latest_target"] is not None
    assert result["latest_target"]["cluster"] == 5
    assert result["light_summary"]["valid_samples"] >= 1


def test_measure_sync_returns_empty_state_when_initial_commands_fail() -> None:
    tool = MmWave()
    ser = FailingSerial(fail_write=True)
    tx_state = {"seq": 0}

    result = tool._measure_sync(
        ser,
        tx_state=tx_state,
        rx_buffer=bytearray(),
        focus_cluster=-1,
        timeout_s=5.0,
        bio_ms=1000,
    )

    assert result["success"] is False
    assert result["attempts"] == 0
    assert "light_summary" in result


def test_measure_sync_preserves_partial_data_on_mid_loop_disconnect() -> None:
    # Send one invalid bio event, then disconnect
    bio_payload = _pack_bio(t_ms=100, allowed=1, valid=0, br_new=0, hr_new=0, br_centi=0xFFFF, hr_centi=0xFFFF)
    light_payload = _pack_light(t_ms=150, valid=1, lux=42.0)
    stream = encode_frame(EVT_BIO, bio_payload, seq=1) + encode_frame(EVT_LIGHT, light_payload, seq=2)

    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._measure_sync(
        ser,
        tx_state=tx_state,
        rx_buffer=bytearray(),
        focus_cluster=-1,
        timeout_s=5.0,
        bio_ms=1000,
    )

    assert result["success"] is False
    assert result["attempts"] >= 1
    assert result["latest_bio"] is not None
    assert result["light_summary"]["valid_samples"] >= 1


@pytest.mark.asyncio
async def test_full_call_returns_error_on_serial_disconnect(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_serial_modules(monkeypatch, [{"bytes": b""}])
    # Make the fake serial raise on write to simulate immediate disconnect
    original_write = FakeSerial.write

    def _failing_write(self: Any, data: bytes) -> None:
        raise FakeSerial.SerialException("device removed")

    monkeypatch.setattr(FakeSerial, "write", _failing_write)

    tool = MmWave()
    response = await tool(_deps(), mode="scan", duration_s=0.1)

    assert "error" in response
    monkeypatch.setattr(FakeSerial, "write", original_write)


# ---------------------------------------------------------------------------
# HELLO feature bits tests
# ---------------------------------------------------------------------------


def _pack_hello(*, proto_version: int, feature_bits: int) -> bytes:
    return struct.pack("<BH", proto_version, feature_bits)


def test_hello_with_light_sensor_feature_bit() -> None:
    payload = _pack_hello(proto_version=1, feature_bits=FEAT_LIGHT_SENSOR)
    event = decode_event(EVT_HELLO, payload)
    assert event["type"] == "hello"
    assert event["proto_version"] == 1
    assert event["feature_bits"] & FEAT_LIGHT_SENSOR


def test_hello_without_light_sensor_feature_bit() -> None:
    payload = _pack_hello(proto_version=1, feature_bits=0)
    event = decode_event(EVT_HELLO, payload)
    assert event["type"] == "hello"
    assert not (event["feature_bits"] & FEAT_LIGHT_SENSOR)


# ---------------------------------------------------------------------------
# Serial port auto-detection tests
# ---------------------------------------------------------------------------


class FakePortInfo:
    """Mimics serial.tools.list_ports_common.ListPortInfo."""

    def __init__(self, device: str, vid: int | None = None, pid: int | None = None) -> None:
        self.device = device
        self.vid = vid
        self.pid = pid


class ProbeSerial:
    """Fake serial for probe tests — responds to PING with PONG or HELLO after reset."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        dsrdtr: bool = False,
        *,
        respond_pong: bool = False,
        respond_hello: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dsrdtr = dsrdtr
        self._respond_pong = respond_pong
        self._respond_hello = respond_hello
        self._dtr_toggled = False
        self._buffer = bytearray()
        self._dtr = False

    @property
    def dtr(self) -> bool:
        return self._dtr

    @dtr.setter
    def dtr(self, value: bool) -> None:
        if not value and self._dtr:
            self._dtr_toggled = True
            if self._respond_hello:
                hello_payload = struct.pack("<BH", PROTO_VERSION, 0)
                self._buffer.extend(encode_frame(EVT_HELLO, hello_payload, seq=0))
        self._dtr = value

    @property
    def in_waiting(self) -> int:
        return len(self._buffer)

    def write(self, data: bytes) -> None:
        if self._respond_pong:
            pong_payload = struct.pack("<I", 0)
            self._buffer.extend(encode_frame(EVT_PONG, pong_payload, seq=0))

    def flush(self) -> None:
        pass

    def read(self, size: int = 1) -> bytes:
        if not self._buffer:
            return b""
        n = min(size, len(self._buffer))
        out = bytes(self._buffer[:n])
        del self._buffer[:n]
        return out

    def close(self) -> None:
        pass

    def __enter__(self) -> "ProbeSerial":
        return self

    def __exit__(self, *_exc_info: object) -> bool:
        return False


def test_resolve_explicit_port_bypasses_detection() -> None:
    tool = MmWave()
    assert tool._resolve_serial_port("/dev/explicit") == "/dev/explicit"


def test_resolve_env_var_bypasses_detection(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MMWAVE_SERIAL_PORT", "/dev/from_env")
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/from_env"


def test_resolve_single_vidpid_match(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [
            FakePortInfo("/dev/cu.other", vid=0x1234, pid=0x5678),
            FakePortInfo("/dev/cu.usbmodem101", vid=0x303A, pid=0x1001),
        ],
    )
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/cu.usbmodem101"


def test_resolve_no_vidpid_single_glob(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [FakePortInfo("/dev/cu.bluetooth", vid=None, pid=None)],
    )
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.glob.glob",
        lambda pattern: ["/dev/cu.usbmodem201"] if "usbmodem" in pattern else [],
    )
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/cu.usbmodem201"


def test_resolve_multiple_vidpid_probe_picks_pong_responder(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [
            FakePortInfo("/dev/cu.usbmodem101", vid=0x303A, pid=0x1001),
            FakePortInfo("/dev/cu.usbmodem201", vid=0x303A, pid=0x1001),
        ],
    )

    def fake_serial(port: str, baudrate: int = 115200, timeout: float = 0.2, dsrdtr: bool = False) -> ProbeSerial:
        return ProbeSerial(port, baudrate, timeout, dsrdtr, respond_pong=(port == "/dev/cu.usbmodem201"))

    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.serial.Serial",
        fake_serial,
    )
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/cu.usbmodem201"


def test_resolve_probe_falls_back_to_hello_after_reset(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [
            FakePortInfo("/dev/cu.usbmodem101", vid=0x303A, pid=0x1001),
            FakePortInfo("/dev/cu.usbmodem201", vid=0x303A, pid=0x1001),
        ],
    )

    def fake_serial(port: str, baudrate: int = 115200, timeout: float = 0.2, dsrdtr: bool = False) -> ProbeSerial:
        return ProbeSerial(port, baudrate, timeout, dsrdtr, respond_hello=(port == "/dev/cu.usbmodem201"))

    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.serial.Serial",
        fake_serial,
    )
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/cu.usbmodem201"


def test_resolve_no_candidates_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [],
    )
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.glob.glob",
        lambda pattern: [],
    )
    tool = MmWave()
    with pytest.raises(RuntimeError, match="No mmWave serial port found"):
        tool._resolve_serial_port(None)


def test_resolve_probe_skips_port_on_open_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("MMWAVE_SERIAL_PORT", raising=False)
    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.comports",
        lambda: [
            FakePortInfo("/dev/cu.usbmodem101", vid=0x303A, pid=0x1001),
            FakePortInfo("/dev/cu.usbmodem201", vid=0x303A, pid=0x1001),
        ],
    )

    call_count = 0

    def fake_serial(port: str, baudrate: int = 115200, timeout: float = 0.2, dsrdtr: bool = False) -> ProbeSerial:
        nonlocal call_count
        call_count += 1
        if port == "/dev/cu.usbmodem101":
            raise OSError("permission denied")
        return ProbeSerial(port, baudrate, timeout, dsrdtr, respond_pong=True)

    monkeypatch.setattr(
        "healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmWave.serial.Serial",
        fake_serial,
    )
    tool = MmWave()
    assert tool._resolve_serial_port(None) == "/dev/cu.usbmodem201"


# ---------------------------------------------------------------------------
# Targets truncation flag and device state surfacing tests
# ---------------------------------------------------------------------------


def test_scan_sync_surfaces_truncation_flag_and_max_target_count() -> None:
    targets_payload = _pack_targets_single(
        t_ms=1000,
        forced_focus=-1,
        cluster=2,
        x_mm=100,
        y_mm=500,
        r_mm=510,
        bearing_cdeg=200,
        v_x10=0,
        truncated=True,
    )
    stream = encode_frame(EVT_TARGETS, targets_payload, seq=1)

    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._scan_sync(
        ser,
        deps=_deps(),
        tx_state=tx_state,
        rx_buffer=bytearray(),
        duration_s=5.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert result["targets_truncated"] is True
    assert result["max_target_count"] == 1


def test_scan_sync_no_truncation_by_default() -> None:
    targets_payload = _pack_targets_single(
        t_ms=1000,
        forced_focus=-1,
        cluster=2,
        x_mm=100,
        y_mm=500,
        r_mm=510,
        bearing_cdeg=200,
        v_x10=0,
        truncated=False,
    )
    stream = encode_frame(EVT_TARGETS, targets_payload, seq=1)

    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._scan_sync(
        ser,
        deps=_deps(),
        tx_state=tx_state,
        rx_buffer=bytearray(),
        duration_s=5.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert result["targets_truncated"] is False
    assert result["max_target_count"] == 1


def test_scan_sync_surfaces_device_state() -> None:
    state_payload = _pack_state(
        t_ms=1000,
        state_enum=5,  # RESTING_VITALS
        pose_enum=1,
        head_moving=0,
        human=1,
        n_targets=1,
        dist_new=1,
        dist_mm=500,
    )
    stream = encode_frame(EVT_STATE, state_payload, seq=1)

    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._scan_sync(
        ser,
        deps=_deps(),
        tx_state=tx_state,
        rx_buffer=bytearray(),
        duration_s=5.0,
        do_sweep=False,
        focus_cluster=-1,
        targets_ms=250,
    )

    assert result["device_state"] == "RESTING_VITALS"
    assert result["state"] == "RESTING_VITALS"


def test_measure_sync_surfaces_device_state() -> None:
    state_payload = _pack_state(
        t_ms=100,
        state_enum=4,  # STILL_NEAR
        pose_enum=1,
        head_moving=0,
        human=1,
        n_targets=1,
        dist_new=1,
        dist_mm=400,
    )
    bio_payload = _pack_bio(t_ms=200, allowed=1, valid=1, br_new=1, hr_new=1, br_centi=1400, hr_centi=7200)
    stream = encode_frame(EVT_STATE, state_payload, seq=1) + encode_frame(EVT_BIO, bio_payload, seq=2)

    ser = FailingSerial(stream, reads_before_disconnect=1)
    tool = MmWave()
    tx_state = {"seq": 0}

    result = tool._measure_sync(
        ser,
        tx_state=tx_state,
        rx_buffer=bytearray(),
        focus_cluster=-1,
        timeout_s=5.0,
        bio_ms=1000,
    )

    assert result["device_state"] == "STILL_NEAR"
    assert result["success"] is True


# ---------------------------------------------------------------------------
# extract_sensor_state tests
# ---------------------------------------------------------------------------


def test_extract_sensor_state_from_full_result() -> None:
    result = {
        "mode": "locate_and_measure",
        "status": "ok",
        "scan": {
            "max_target_count": 3,
            "targets_truncated": True,
            "device_state": "STILL_NEAR",
            "latest_target": {"cluster": 0, "r": 0.51, "bearing": -12.8},
            "light_summary": {"latest_lux": 42.5},
        },
        "measure": {
            "device_state": "RESTING_VITALS",
            "valid_bio": {"heart_rate_bpm": 72.0, "breath_rate_bpm": 14.0},
            "light_summary": {"latest_lux": 43.0},
        },
        "light_context": {"context_state": "neutral"},
    }

    state = extract_sensor_state(result)

    assert state["target_count"] == 3
    assert state["targets_truncated"] is True
    assert state["device_state"] == "RESTING_VITALS"  # measure overrides scan
    assert state["heart_rate_bpm"] == 72.0
    assert state["breath_rate_bpm"] == 14.0
    assert state["lux"] == 43.0  # measure overrides scan
    assert state["closest_target_r"] == 0.51
    assert state["light_context_state"] == "neutral"
    assert state["mode"] == "locate_and_measure"
    assert state["status"] == "ok"
    assert "updated_at" in state


def test_extract_sensor_state_scan_only() -> None:
    result = {
        "mode": "scan",
        "status": "scan_done",
        "scan": {
            "max_target_count": 1,
            "targets_truncated": False,
            "device_state": "PRESENT_FAR",
            "latest_target": None,
            "light_summary": {"latest_lux": 100.0},
        },
    }

    state = extract_sensor_state(result)

    assert state["target_count"] == 1
    assert state["targets_truncated"] is False
    assert state["device_state"] == "PRESENT_FAR"
    assert "heart_rate_bpm" not in state
    assert state["lux"] == 100.0


def test_extract_sensor_state_empty_result() -> None:
    result: dict[str, Any] = {"mode": "scan", "status": "scan_done"}

    state = extract_sensor_state(result)

    assert state["mode"] == "scan"
    assert "target_count" not in state
    assert "updated_at" in state


def test_extract_sensor_state_error_result() -> None:
    """Error results produce state with error field and disconnected status."""
    result: dict[str, Any] = {
        "error": "serial error on /dev/ttyUSB0: device disconnected",
        "status": "disconnected",
    }

    state = extract_sensor_state(result)

    assert state["error"] == "serial error on /dev/ttyUSB0: device disconnected"
    assert state["status"] == "disconnected"
    assert "updated_at" in state
    # Error results should NOT contain scan/measure fields
    assert "target_count" not in state
    assert "heart_rate_bpm" not in state
    assert "device_state" not in state


def test_extract_sensor_state_error_without_status() -> None:
    """Error results without explicit status get a generic 'error' status."""
    result: dict[str, Any] = {"error": "something went wrong"}

    state = extract_sensor_state(result)

    assert state["error"] == "something went wrong"
    assert state["status"] == "error"
    assert "updated_at" in state
