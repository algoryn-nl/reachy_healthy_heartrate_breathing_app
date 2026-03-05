"""Cross-validate C firmware codec (via ctypes) against Python mmwave_protocol.py."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import math
import ctypes
import struct
import platform
import subprocess
from ctypes import (
    POINTER,
    byref,
    c_int,
    c_float,
    c_int16,
    c_int32,
    c_uint8,
    c_size_t,
    c_uint16,
    c_uint32,
)
from pathlib import Path

import pytest

from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    cobs_encode,
    crc16_ccitt_false,
)


# ---------------------------------------------------------------------------
# Session-scoped fixture: build and load the shared library
# ---------------------------------------------------------------------------

HARDWARE_TESTS_DIR = Path(__file__).resolve().parent.parent / "hardware" / "tests"


@pytest.fixture(scope="session")
def lib():
    """Build libreachy_codec and return the ctypes CDLL handle with argtypes set."""
    # Build
    result = subprocess.run(
        ["make", "-C", str(HARDWARE_TESTS_DIR)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.fail(f"make failed:\nstdout: {result.stdout}\nstderr: {result.stderr}")

    # Load
    ext = "dylib" if platform.system() == "Darwin" else "so"
    lib_path = HARDWARE_TESTS_DIR / f"libreachy_codec.{ext}"
    if not lib_path.exists():
        pytest.fail(f"Shared library not found at {lib_path}")
    cdll = ctypes.CDLL(str(lib_path))

    # --- argtypes / restype setup ---
    cdll.shim_frame_delimiter.argtypes = []
    cdll.shim_frame_delimiter.restype = c_uint8

    cdll.shim_is_finite_positive.argtypes = [c_float]
    cdll.shim_is_finite_positive.restype = c_int

    cdll.shim_crc16_ccitt_false.argtypes = [POINTER(c_uint8), c_size_t]
    cdll.shim_crc16_ccitt_false.restype = c_uint16

    cdll.shim_cobs_encode.argtypes = [
        POINTER(c_uint8),
        c_size_t,
        POINTER(c_uint8),
        c_size_t,
    ]
    cdll.shim_cobs_encode.restype = c_size_t

    cdll.shim_cobs_decode.argtypes = [
        POINTER(c_uint8),
        c_size_t,
        POINTER(c_uint8),
        POINTER(c_size_t),
        c_size_t,
    ]
    cdll.shim_cobs_decode.restype = c_int

    cdll.shim_append_u8.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_uint8]
    cdll.shim_append_u8.restype = None

    cdll.shim_append_u16le.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_uint16]
    cdll.shim_append_u16le.restype = None

    cdll.shim_append_i16le.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_int16]
    cdll.shim_append_i16le.restype = None

    cdll.shim_append_u32le.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_uint32]
    cdll.shim_append_u32le.restype = None

    cdll.shim_append_i32le.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_int32]
    cdll.shim_append_i32le.restype = None

    cdll.shim_append_f32le.argtypes = [POINTER(c_uint8), POINTER(c_size_t), c_float]
    cdll.shim_append_f32le.restype = None

    cdll.shim_read_u16le.argtypes = [POINTER(c_uint8)]
    cdll.shim_read_u16le.restype = c_uint16

    cdll.shim_read_i16le.argtypes = [POINTER(c_uint8)]
    cdll.shim_read_i16le.restype = c_int16

    cdll.shim_to_i16_scaled.argtypes = [c_float, c_float]
    cdll.shim_to_i16_scaled.restype = c_int16

    cdll.shim_to_u16_scaled_or_null.argtypes = [c_float, c_float]
    cdll.shim_to_u16_scaled_or_null.restype = c_uint16

    return cdll


# ---------------------------------------------------------------------------
# Helper: COBS round-trip through C
# ---------------------------------------------------------------------------

def _c_cobs_round_trip(lib, data: bytes) -> bytes:
    """Encode then decode data through the C COBS codec and return the result."""
    in_len = len(data)
    in_buf = (c_uint8 * in_len)(*data) if in_len > 0 else (c_uint8 * 0)()
    enc_cap = in_len + in_len // 254 + 2  # worst-case COBS expansion
    enc_buf = (c_uint8 * enc_cap)()

    enc_len = lib.shim_cobs_encode(
        in_buf if in_len > 0 else None,
        in_len,
        enc_buf,
        enc_cap,
    )
    assert enc_len > 0 or in_len == 0

    dec_buf = (c_uint8 * (enc_len + 1))()
    out_len = c_size_t(0)
    ok = lib.shim_cobs_decode(enc_buf, enc_len, dec_buf, byref(out_len), enc_len + 1)
    assert ok == 1
    return bytes(dec_buf[: out_len.value])


def _c_cobs_encode(lib, data: bytes) -> bytes:
    """Encode data through the C COBS codec and return the encoded bytes."""
    in_len = len(data)
    in_buf = (c_uint8 * in_len)(*data) if in_len > 0 else (c_uint8 * 0)()
    enc_cap = in_len + in_len // 254 + 2
    enc_buf = (c_uint8 * enc_cap)()

    enc_len = lib.shim_cobs_encode(
        in_buf if in_len > 0 else None,
        in_len,
        enc_buf,
        enc_cap,
    )
    return bytes(enc_buf[:enc_len])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestFrameDelimiter:
    def test_delimiter_is_zero(self, lib):
        assert lib.shim_frame_delimiter() == 0x00


class TestIsFinitePositive:
    def test_positive_float(self, lib):
        assert lib.shim_is_finite_positive(c_float(1.0)) == 1
        assert lib.shim_is_finite_positive(c_float(0.001)) == 1

    def test_zero(self, lib):
        assert lib.shim_is_finite_positive(c_float(0.0)) == 0

    def test_negative(self, lib):
        assert lib.shim_is_finite_positive(c_float(-1.0)) == 0

    def test_nan(self, lib):
        assert lib.shim_is_finite_positive(c_float(math.nan)) == 0

    def test_inf(self, lib):
        assert lib.shim_is_finite_positive(c_float(math.inf)) == 0

    def test_neg_inf(self, lib):
        assert lib.shim_is_finite_positive(c_float(-math.inf)) == 0


class TestCrc16:
    def test_known_vector(self, lib):
        data = b"123456789"
        buf = (c_uint8 * len(data))(*data)
        assert lib.shim_crc16_ccitt_false(buf, len(data)) == 0x29B1

    def test_empty(self, lib):
        buf = (c_uint8 * 0)()
        assert lib.shim_crc16_ccitt_false(buf, 0) == 0xFFFF

    def test_cross_validation(self, lib):
        test_vectors = [
            b"",
            b"\x00",
            b"\xFF",
            b"123456789",
            b"Hello, mmWave!",
            bytes(range(256)),
            b"\x00" * 100,
        ]
        for data in test_vectors:
            buf = (c_uint8 * len(data))(*data) if data else (c_uint8 * 0)()
            c_result = lib.shim_crc16_ccitt_false(buf, len(data))
            py_result = crc16_ccitt_false(data)
            assert c_result == py_result, f"CRC mismatch for {data!r}: C={c_result:#06x} Python={py_result:#06x}"


class TestCobsRoundTrip:
    def test_empty_input(self, lib):
        assert _c_cobs_round_trip(lib, b"") == b""

    def test_no_zeros(self, lib):
        assert _c_cobs_round_trip(lib, b"\x01\x02\x03") == b"\x01\x02\x03"

    def test_single_zero(self, lib):
        assert _c_cobs_round_trip(lib, b"\x00") == b"\x00"

    def test_all_zeros(self, lib):
        assert _c_cobs_round_trip(lib, b"\x00\x00\x00") == b"\x00\x00\x00"

    def test_mixed(self, lib):
        assert _c_cobs_round_trip(lib, b"\x01\x00\x02\x00\x03") == b"\x01\x00\x02\x00\x03"

    def test_254_non_zero_bytes(self, lib):
        data = bytes(range(1, 255))
        assert _c_cobs_round_trip(lib, data) == data


class TestCobsCrossValidation:
    def test_encode_matches_python(self, lib):
        test_vectors = [
            b"",
            b"\x00",
            b"\x01\x02\x03",
            b"\x00\x00\x00",
            b"\x01\x00\x02\x00\x03",
            bytes(range(1, 255)),
            bytes(range(256)),
        ]
        for data in test_vectors:
            c_encoded = _c_cobs_encode(lib, data)
            py_encoded = cobs_encode(data)
            assert c_encoded == py_encoded, (
                f"COBS encode mismatch for {data!r}:\n"
                f"  C:      {c_encoded.hex()}\n"
                f"  Python: {py_encoded.hex()}"
            )


class TestSerialization:
    def test_append_u8(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_u8(buf, byref(idx), 0x42)
        assert idx.value == 1
        assert buf[0] == 0x42

    def test_append_u16le(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_u16le(buf, byref(idx), 0x1234)
        assert idx.value == 2
        assert buf[0] == 0x34
        assert buf[1] == 0x12

    def test_append_i16le_negative(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_i16le(buf, byref(idx), c_int16(-1))
        assert idx.value == 2
        assert buf[0] == 0xFF
        assert buf[1] == 0xFF

    def test_append_u32le(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_u32le(buf, byref(idx), 0x12345678)
        assert idx.value == 4
        assert buf[0] == 0x78
        assert buf[1] == 0x56
        assert buf[2] == 0x34
        assert buf[3] == 0x12

    def test_append_i32le_negative(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_i32le(buf, byref(idx), c_int32(-1))
        assert idx.value == 4
        assert all(buf[i] == 0xFF for i in range(4))

    def test_append_f32le(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_f32le(buf, byref(idx), c_float(1.0))
        assert idx.value == 4
        expected = struct.pack("<f", 1.0)
        assert bytes(buf[:4]) == expected

    def test_read_u16le(self, lib):
        buf = (c_uint8 * 2)(0x34, 0x12)
        assert lib.shim_read_u16le(buf) == 0x1234

    def test_read_i16le_negative(self, lib):
        buf = (c_uint8 * 2)(0xFF, 0xFF)
        assert lib.shim_read_i16le(buf) == -1

    def test_u16le_round_trip(self, lib):
        buf = (c_uint8 * 16)()
        idx = c_size_t(0)
        lib.shim_append_u16le(buf, byref(idx), 0xABCD)
        assert lib.shim_read_u16le(buf) == 0xABCD


class TestToI16Scaled:
    def test_normal(self, lib):
        assert lib.shim_to_i16_scaled(c_float(1.5), c_float(1000.0)) == 1500

    def test_clamp_max(self, lib):
        assert lib.shim_to_i16_scaled(c_float(40.0), c_float(1000.0)) == 32767

    def test_clamp_min(self, lib):
        assert lib.shim_to_i16_scaled(c_float(-40.0), c_float(1000.0)) == -32768

    def test_nan(self, lib):
        assert lib.shim_to_i16_scaled(c_float(math.nan), c_float(1000.0)) == 0

    def test_inf(self, lib):
        assert lib.shim_to_i16_scaled(c_float(math.inf), c_float(1000.0)) == 0

    def test_neg_inf(self, lib):
        assert lib.shim_to_i16_scaled(c_float(-math.inf), c_float(1000.0)) == 0

    def test_zero(self, lib):
        assert lib.shim_to_i16_scaled(c_float(0.0), c_float(1000.0)) == 0

    def test_negative(self, lib):
        assert lib.shim_to_i16_scaled(c_float(-1.5), c_float(1000.0)) == -1500


class TestToU16ScaledOrNull:
    def test_normal(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(1.5), c_float(1000.0)) == 1500

    def test_clamp_max(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(70.0), c_float(1000.0)) == 65534

    def test_negative_clamps_to_zero(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(-1.0), c_float(1000.0)) == 0

    def test_nan_sentinel(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(math.nan), c_float(1000.0)) == 0xFFFF

    def test_inf_sentinel(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(math.inf), c_float(1000.0)) == 0xFFFF

    def test_neg_inf_sentinel(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(-math.inf), c_float(1000.0)) == 0xFFFF

    def test_zero(self, lib):
        assert lib.shim_to_u16_scaled_or_null(c_float(0.0), c_float(1000.0)) == 0


class TestCmdResetCodec:
    """Cross-validate CMD_RESET frame encoding between Python and C codec."""

    def test_cmd_reset_frame_cobs_crc_matches_c(self, lib) -> None:
        """Encode CMD_RESET in Python, decode via C COBS, verify CRC matches."""
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            encode_frame,
            pack_cmd_reset,
        )

        frame = encode_frame(CMD_RESET, pack_cmd_reset(), seq=0)
        encoded = frame[:-1]  # strip 0x00 delimiter

        # COBS decode via C
        in_buf = (c_uint8 * len(encoded))(*encoded)
        out_buf = (c_uint8 * 256)()
        out_len = c_size_t(0)
        ok = lib.shim_cobs_decode(in_buf, len(encoded), out_buf, byref(out_len), 256)
        assert ok == 1

        # Verify CRC via C
        decoded_bytes = bytes(out_buf[: out_len.value])
        data_len = out_len.value - 2
        data_buf = (c_uint8 * data_len)(*decoded_bytes[:data_len])
        crc_c = lib.shim_crc16_ccitt_false(data_buf, data_len)

        crc_in_packet = struct.unpack_from("<H", decoded_bytes, data_len)[0]
        assert crc_c == crc_in_packet
