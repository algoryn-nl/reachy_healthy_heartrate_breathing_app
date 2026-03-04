# FW-LOW-2: Firmware Test Harness — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Extract pure C functions from `reachy-sensor.ino` into a testable header, compile to shared library, and cross-validate against the Python codec via pytest/ctypes.

**Architecture:** Header-only `reachy_codec.h` contains 15 pure functions. A thin C shim (`reachy_codec_shim.c`) re-exports them with `extern` linkage. Makefile compiles to `.so`/`.dylib`. Pytest loads the library via ctypes and cross-validates against `mmwave_protocol.py`.

**Tech Stack:** C11, Python 3.12, pytest, ctypes, Make

---

### Task 1: Create the header file `reachy_codec.h`

**Files:**
- Create: `hardware/arduino/reachy-sensor/reachy_codec.h`
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino`

**Step 1: Create `reachy_codec.h`**

This header must compile in both Arduino (C++) and plain C contexts. Use `#ifdef __cplusplus extern "C"` guards and `<stdint.h>`/`<stddef.h>`/`<math.h>` instead of Arduino types.

```c
// reachy_codec.h — Pure protocol functions shared between firmware and host tests.
// Included by reachy-sensor.ino (Arduino) and reachy_codec_shim.c (host test build).
#ifndef REACHY_CODEC_H
#define REACHY_CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

// --- Frame delimiter ---
static const uint8_t FRAME_DELIMITER = 0x00;

// --- Guards ---
static inline int rc_is_finite_positive(float v) {
    return isfinite(v) && v > 0.0f;
}

// --- CRC-16/CCITT-FALSE ---
static inline uint16_t rc_crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

// --- Buffer serialization (little-endian) ---
static inline void rc_append_u8(uint8_t* buf, size_t* idx, uint8_t value) {
    buf[(*idx)++] = value;
}

static inline void rc_append_u16le(uint8_t* buf, size_t* idx, uint16_t value) {
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void rc_append_i16le(uint8_t* buf, size_t* idx, int16_t value) {
    rc_append_u16le(buf, idx, (uint16_t)value);
}

static inline void rc_append_u32le(uint8_t* buf, size_t* idx, uint32_t value) {
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFF);
}

static inline void rc_append_i32le(uint8_t* buf, size_t* idx, int32_t value) {
    rc_append_u32le(buf, idx, (uint32_t)value);
}

static inline void rc_append_f32le(uint8_t* buf, size_t* idx, float value) {
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(raw));
    rc_append_u32le(buf, idx, raw);
}

static inline uint16_t rc_read_u16le(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline int16_t rc_read_i16le(const uint8_t* buf) {
    return (int16_t)rc_read_u16le(buf);
}

// --- COBS codec ---
// Returns encoded length, or 0 on overflow.
static inline size_t rc_cobs_encode(const uint8_t* input, size_t inputLen,
                                     uint8_t* output, size_t outputCap) {
    if (outputCap == 0) return 0;
    size_t readIdx = 0, writeIdx = 1, codeIdx = 0;
    uint8_t code = 1;
    output[0] = 0;

    while (readIdx < inputLen) {
        uint8_t byte = input[readIdx++];
        if (byte == 0) {
            if (codeIdx >= outputCap) return 0;
            output[codeIdx] = code;
            code = 1;
            codeIdx = writeIdx++;
            if (codeIdx >= outputCap) return 0;
            continue;
        }
        if (writeIdx >= outputCap) return 0;
        output[writeIdx++] = byte;
        code++;
        if (code == 0xFF) {
            if (codeIdx >= outputCap) return 0;
            output[codeIdx] = code;
            code = 1;
            codeIdx = writeIdx++;
            if (codeIdx >= outputCap) return 0;
        }
    }
    if (codeIdx >= outputCap) return 0;
    output[codeIdx] = code;
    return writeIdx;
}

// Returns 1 on success, 0 on malformed input or overflow.
static inline int rc_cobs_decode(const uint8_t* input, size_t inputLen,
                                  uint8_t* output, size_t* outLen, size_t outputCap) {
    if (inputLen == 0) return 0;
    size_t readIdx = 0, writeIdx = 0;

    while (readIdx < inputLen) {
        uint8_t code = input[readIdx++];
        if (code == 0) return 0;
        size_t next = readIdx + (size_t)code - 1;
        if (next > inputLen) return 0;
        while (readIdx < next) {
            if (writeIdx >= outputCap) return 0;
            output[writeIdx++] = input[readIdx++];
        }
        if (code < 0xFF && readIdx < inputLen) {
            if (writeIdx >= outputCap) return 0;
            output[writeIdx++] = 0;
        }
    }
    *outLen = writeIdx;
    return 1;
}

// --- Numeric scaling ---
static inline int16_t rc_to_i16_scaled(float value, float scale) {
    if (!isfinite(value)) return 0;
    float scaled = value * scale;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)lroundf(scaled);
}

// NaN/Inf → 0xFFFF (null sentinel).
static inline uint16_t rc_to_u16_scaled_or_null(float value, float scale) {
    if (!isfinite(value)) return 0xFFFF;
    float scaled = value * scale;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 65534.0f) scaled = 65534.0f;
    return (uint16_t)lroundf(scaled);
}

#ifdef __cplusplus
}
#endif

#endif // REACHY_CODEC_H
```

**Step 2: Update `reachy-sensor.ino`**

Add `#include "reachy_codec.h"` near the top (after the existing library includes, before the protocol constants section). Then replace the 15 inline function definitions with thin wrappers that call the `rc_*` versions — OR simply delete the duplicates and use the `rc_*` names directly throughout the .ino. The simpler approach is to keep the old names as `static inline` aliases:

After the `#include "reachy_codec.h"` line, replace each original function body. For example the `FRAME_DELIMITER` constant is now in the header — remove it from the .ino. For functions, replace:
- `isFinitePositive(v)` body → `return rc_is_finite_positive(v);`
- `crc16CcittFalse(data, len)` body → `return rc_crc16_ccitt_false(data, len);`
- `cobsEncode(...)` body → `return rc_cobs_encode(...);`
- `cobsDecode(...)` body → `return rc_cobs_decode(...);`
- All `appendXxx` / `readXxx` bodies → call `rc_append_xxx` / `rc_read_xxx`
- `toI16Scaled` / `toU16ScaledOrNull` bodies → call `rc_to_i16_scaled` / `rc_to_u16_scaled_or_null`

Alternatively (cleaner): use `#define` aliases at the top of the .ino:
```c
#include "reachy_codec.h"
#define isFinitePositive  rc_is_finite_positive
#define crc16CcittFalse   rc_crc16_ccitt_false
#define cobsEncode        rc_cobs_encode
#define cobsDecode        rc_cobs_decode
#define appendU8          rc_append_u8
#define appendU16LE       rc_append_u16le
#define appendI16LE       rc_append_i16le
#define appendU32LE       rc_append_u32le
#define appendI32LE       rc_append_i32le
#define appendF32LE       rc_append_f32le
#define readU16LE         rc_read_u16le
#define readI16LE         rc_read_i16le
#define toI16Scaled       rc_to_i16_scaled
#define toU16ScaledOrNull rc_to_u16_scaled_or_null
```

Then **delete the original function bodies** from the .ino (lines ~393-574, keeping surrounding section comments).

**Step 3: Verify firmware still compiles conceptually**

We can't compile the .ino without the Arduino toolchain, but verify the header compiles on its own:

Run: `cc -std=c11 -fsyntax-only -x c hardware/arduino/reachy-sensor/reachy_codec.h`
Expected: no errors

**Step 4: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy_codec.h hardware/arduino/reachy-sensor/reachy-sensor.ino
git commit -m "refactor: extract pure codec functions into reachy_codec.h (FW-LOW-2)"
```

---

### Task 2: Create the C shim and Makefile

**Files:**
- Create: `hardware/tests/reachy_codec_shim.c`
- Create: `hardware/tests/Makefile`

**Step 1: Create `hardware/tests/reachy_codec_shim.c`**

This file forces the header's `static inline` functions to be emitted as real symbols so ctypes can find them.

```c
// reachy_codec_shim.c — Export reachy_codec.h functions for ctypes testing.
#include "../arduino/reachy-sensor/reachy_codec.h"

// Force emission of all static inline functions as real exported symbols.
int           shim_is_finite_positive(float v)                   { return rc_is_finite_positive(v); }
uint16_t      shim_crc16(const uint8_t* data, size_t len)        { return rc_crc16_ccitt_false(data, len); }
size_t        shim_cobs_encode(const uint8_t* in, size_t inLen,
                               uint8_t* out, size_t outCap)      { return rc_cobs_encode(in, inLen, out, outCap); }
int           shim_cobs_decode(const uint8_t* in, size_t inLen,
                               uint8_t* out, size_t* outLen,
                               size_t outCap)                    { return rc_cobs_decode(in, inLen, out, outLen, outCap); }
void          shim_append_u8(uint8_t* buf, size_t* idx,
                             uint8_t value)                      { rc_append_u8(buf, idx, value); }
void          shim_append_u16le(uint8_t* buf, size_t* idx,
                                uint16_t value)                  { rc_append_u16le(buf, idx, value); }
void          shim_append_i16le(uint8_t* buf, size_t* idx,
                                int16_t value)                   { rc_append_i16le(buf, idx, value); }
void          shim_append_u32le(uint8_t* buf, size_t* idx,
                                uint32_t value)                  { rc_append_u32le(buf, idx, value); }
void          shim_append_i32le(uint8_t* buf, size_t* idx,
                                int32_t value)                   { rc_append_i32le(buf, idx, value); }
void          shim_append_f32le(uint8_t* buf, size_t* idx,
                                float value)                     { rc_append_f32le(buf, idx, value); }
uint16_t      shim_read_u16le(const uint8_t* buf)                { return rc_read_u16le(buf); }
int16_t       shim_read_i16le(const uint8_t* buf)                { return rc_read_i16le(buf); }
int16_t       shim_to_i16_scaled(float value, float scale)       { return rc_to_i16_scaled(value, scale); }
uint16_t      shim_to_u16_scaled_or_null(float value, float scale) { return rc_to_u16_scaled_or_null(value, scale); }
```

**Step 2: Create `hardware/tests/Makefile`**

```makefile
# Build shared library for firmware codec host testing via ctypes.
UNAME := $(shell uname -s)
ifeq ($(UNAME),Darwin)
  LIB = libreachy_codec.dylib
  LDFLAGS = -dynamiclib
else
  LIB = libreachy_codec.so
  LDFLAGS = -shared
endif

CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -fPIC

$(LIB): reachy_codec_shim.c ../arduino/reachy-sensor/reachy_codec.h
	$(CC) $(CFLAGS) $(LDFLAGS) -lm -o $@ reachy_codec_shim.c

clean:
	rm -f libreachy_codec.so libreachy_codec.dylib

.PHONY: clean
```

**Step 3: Build the shared library**

Run: `make -C hardware/tests`
Expected: `libreachy_codec.dylib` (macOS) or `libreachy_codec.so` (Linux) created without errors

**Step 4: Add built library to `.gitignore`**

Append to the project `.gitignore`:
```
hardware/tests/libreachy_codec.*
```

**Step 5: Commit**

```bash
git add hardware/tests/reachy_codec_shim.c hardware/tests/Makefile .gitignore
git commit -m "feat: add C shim and Makefile for firmware codec testing"
```

---

### Task 3: Write CRC-16 cross-validation tests

**Files:**
- Create: `tests/test_firmware_codec.py`

**Step 1: Write the test file with a conftest-style fixture and CRC tests**

```python
"""Cross-validate firmware C codec against Python mmwave_protocol via ctypes."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations

import ctypes
import platform
import struct
import subprocess
from pathlib import Path

import pytest

from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    cobs_decode as py_cobs_decode,
    cobs_encode as py_cobs_encode,
    crc16_ccitt_false as py_crc16,
)

HARDWARE_TESTS_DIR = Path(__file__).resolve().parent.parent / "hardware" / "tests"


@pytest.fixture(scope="session")
def codec() -> ctypes.CDLL:
    """Build and load the firmware codec shared library."""
    subprocess.check_call(["make", "-C", str(HARDWARE_TESTS_DIR)])
    ext = "dylib" if platform.system() == "Darwin" else "so"
    lib_path = HARDWARE_TESTS_DIR / f"libreachy_codec.{ext}"
    lib = ctypes.CDLL(str(lib_path))

    # Declare return types
    lib.shim_crc16.restype = ctypes.c_uint16
    lib.shim_cobs_encode.restype = ctypes.c_size_t
    lib.shim_cobs_decode.restype = ctypes.c_int
    lib.shim_is_finite_positive.restype = ctypes.c_int
    lib.shim_read_u16le.restype = ctypes.c_uint16
    lib.shim_read_i16le.restype = ctypes.c_int16
    lib.shim_to_i16_scaled.restype = ctypes.c_int16
    lib.shim_to_u16_scaled_or_null.restype = ctypes.c_uint16
    return lib


class TestCrc16CrossValidation:
    """C crc16 must match Python crc16 for all inputs."""

    @pytest.mark.parametrize(
        "data",
        [
            b"",
            b"\x00",
            b"\xff",
            b"hello",
            b"\x01\x02\x03\x04",
            bytes(range(256)),
            b"\x00" * 100,
        ],
        ids=["empty", "zero", "0xff", "hello", "1234", "all_bytes", "100_zeros"],
    )
    def test_crc_matches_python(self, codec: ctypes.CDLL, data: bytes) -> None:
        c_buf = (ctypes.c_uint8 * len(data))(*data) if data else (ctypes.c_uint8 * 0)()
        c_result = codec.shim_crc16(c_buf, ctypes.c_size_t(len(data)))
        py_result = py_crc16(data)
        assert c_result == py_result, f"CRC mismatch for {data!r}: C={c_result:#06x} Python={py_result:#06x}"
```

**Step 2: Run tests**

Run: `uv run pytest tests/test_firmware_codec.py -v`
Expected: 7 CRC tests pass

**Step 3: Commit**

```bash
git add tests/test_firmware_codec.py
git commit -m "test: add CRC-16 cross-validation tests (firmware vs Python)"
```

---

### Task 4: Add COBS cross-validation tests

**Files:**
- Modify: `tests/test_firmware_codec.py`

**Step 1: Add COBS test class**

Append to the test file:

```python
class TestCobsCrossValidation:
    """C COBS codec must match Python COBS codec and round-trip correctly."""

    @pytest.mark.parametrize(
        "data",
        [
            b"",
            b"\x00",
            b"\x01",
            b"\x00\x00\x00",
            b"hello",
            b"\x01\x02\x03",
            bytes(range(1, 255)),  # 254 non-zero bytes (triggers code=0xFF block)
            b"\x00" + bytes(range(1, 100)) + b"\x00",
        ],
        ids=["empty", "single_zero", "single_nonzero", "three_zeros", "hello",
             "123", "254_nonzero", "zeros_around_data"],
    )
    def test_encode_matches_python(self, codec: ctypes.CDLL, data: bytes) -> None:
        """C cobsEncode must produce identical bytes to Python cobs_encode."""
        py_encoded = py_cobs_encode(data)

        in_buf = (ctypes.c_uint8 * len(data))(*data) if data else (ctypes.c_uint8 * 0)()
        out_buf = (ctypes.c_uint8 * 512)()
        c_len = codec.shim_cobs_encode(in_buf, ctypes.c_size_t(len(data)),
                                        out_buf, ctypes.c_size_t(512))
        c_encoded = bytes(out_buf[:c_len])
        assert c_encoded == py_encoded, f"COBS encode mismatch for {data!r}"

    @pytest.mark.parametrize(
        "data",
        [
            b"",
            b"\x00",
            b"\x01\x02\x03",
            b"hello world",
            bytes(range(1, 255)),
            b"\x00" * 50,
        ],
        ids=["empty", "single_zero", "123", "hello_world", "254_nonzero", "50_zeros"],
    )
    def test_c_roundtrip(self, codec: ctypes.CDLL, data: bytes) -> None:
        """C encode then decode recovers the original data."""
        in_buf = (ctypes.c_uint8 * len(data))(*data) if data else (ctypes.c_uint8 * 0)()
        enc_buf = (ctypes.c_uint8 * 512)()
        enc_len = codec.shim_cobs_encode(in_buf, ctypes.c_size_t(len(data)),
                                          enc_buf, ctypes.c_size_t(512))
        assert enc_len > 0 or len(data) == 0

        dec_buf = (ctypes.c_uint8 * 512)()
        dec_len = ctypes.c_size_t(0)
        ok = codec.shim_cobs_decode(enc_buf, ctypes.c_size_t(enc_len),
                                     dec_buf, ctypes.byref(dec_len), ctypes.c_size_t(512))
        assert ok == 1, "COBS decode failed"
        assert bytes(dec_buf[:dec_len.value]) == data

    def test_encode_overflow_returns_zero(self, codec: ctypes.CDLL) -> None:
        """Output buffer too small should return 0, not crash."""
        data = b"hello"
        in_buf = (ctypes.c_uint8 * len(data))(*data)
        tiny_buf = (ctypes.c_uint8 * 2)()
        result = codec.shim_cobs_encode(in_buf, ctypes.c_size_t(len(data)),
                                         tiny_buf, ctypes.c_size_t(2))
        assert result == 0
```

**Step 2: Run tests**

Run: `uv run pytest tests/test_firmware_codec.py -v`
Expected: 7 CRC + 15 COBS = 22 tests pass

**Step 3: Commit**

```bash
git add tests/test_firmware_codec.py
git commit -m "test: add COBS cross-validation and round-trip tests"
```

---

### Task 5: Add serialization and numeric scaling tests

**Files:**
- Modify: `tests/test_firmware_codec.py`

**Step 1: Add serialization and scaling test classes**

Append to the test file:

```python
class TestSerialization:
    """Buffer append/read helpers produce correct little-endian bytes."""

    def test_append_u16le(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 8)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_u16le(buf, ctypes.byref(idx), ctypes.c_uint16(0x1234))
        assert bytes(buf[:2]) == b"\x34\x12"
        assert idx.value == 2

    def test_append_i16le_negative(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 8)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_i16le(buf, ctypes.byref(idx), ctypes.c_int16(-1))
        assert bytes(buf[:2]) == b"\xff\xff"

    def test_append_u32le(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 8)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_u32le(buf, ctypes.byref(idx), ctypes.c_uint32(0xDEADBEEF))
        assert bytes(buf[:4]) == b"\xef\xbe\xad\xde"

    def test_append_f32le(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 8)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_f32le(buf, ctypes.byref(idx), ctypes.c_float(1.0))
        expected = struct.pack("<f", 1.0)
        assert bytes(buf[:4]) == expected

    def test_read_u16le_roundtrip(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 8)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_u16le(buf, ctypes.byref(idx), ctypes.c_uint16(0xABCD))
        result = codec.shim_read_u16le(buf)
        assert result == 0xABCD

    def test_read_i16le_negative(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 2)(*b"\x00\x80")  # -32768 in LE
        result = codec.shim_read_i16le(buf)
        assert result == -32768

    def test_append_u8(self, codec: ctypes.CDLL) -> None:
        buf = (ctypes.c_uint8 * 4)()
        idx = ctypes.c_size_t(0)
        codec.shim_append_u8(buf, ctypes.byref(idx), ctypes.c_uint8(0x42))
        assert buf[0] == 0x42
        assert idx.value == 1


class TestNumericScaling:
    """toI16Scaled and toU16ScaledOrNull handle edge cases correctly."""

    def test_i16_normal_scaling(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_i16_scaled(ctypes.c_float(1.5), ctypes.c_float(100.0))
        assert result == 150

    def test_i16_clamp_max(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_i16_scaled(ctypes.c_float(500.0), ctypes.c_float(100.0))
        assert result == 32767

    def test_i16_clamp_min(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_i16_scaled(ctypes.c_float(-500.0), ctypes.c_float(100.0))
        assert result == -32768

    def test_i16_nan_returns_zero(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_i16_scaled(ctypes.c_float(float("nan")), ctypes.c_float(100.0))
        assert result == 0

    def test_i16_inf_returns_zero(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_i16_scaled(ctypes.c_float(float("inf")), ctypes.c_float(100.0))
        assert result == 0

    def test_u16_normal_scaling(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_u16_scaled_or_null(ctypes.c_float(1.5), ctypes.c_float(100.0))
        assert result == 150

    def test_u16_nan_returns_sentinel(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_u16_scaled_or_null(ctypes.c_float(float("nan")), ctypes.c_float(100.0))
        assert result == 0xFFFF

    def test_u16_inf_returns_sentinel(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_u16_scaled_or_null(ctypes.c_float(float("inf")), ctypes.c_float(100.0))
        assert result == 0xFFFF

    def test_u16_negative_clamps_to_zero(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_u16_scaled_or_null(ctypes.c_float(-5.0), ctypes.c_float(100.0))
        assert result == 0

    def test_u16_clamp_max(self, codec: ctypes.CDLL) -> None:
        result = codec.shim_to_u16_scaled_or_null(ctypes.c_float(700.0), ctypes.c_float(100.0))
        assert result == 65534


class TestIsFinitePositive:
    """isFinitePositive edge cases."""

    @pytest.mark.parametrize(
        ("value", "expected"),
        [
            (0.001, 1),
            (100.0, 1),
            (0.0, 0),
            (-1.0, 0),
            (float("nan"), 0),
            (float("inf"), 0),
            (float("-inf"), 0),
        ],
        ids=["small_positive", "large_positive", "zero", "negative", "nan", "inf", "neg_inf"],
    )
    def test_edge_cases(self, codec: ctypes.CDLL, value: float, expected: int) -> None:
        result = codec.shim_is_finite_positive(ctypes.c_float(value))
        assert result == expected
```

**Step 2: Run tests**

Run: `uv run pytest tests/test_firmware_codec.py -v`
Expected: 7 CRC + 15 COBS + 7 serialization + 10 scaling + 7 isFinitePositive = 46 tests pass

**Step 3: Commit**

```bash
git add tests/test_firmware_codec.py
git commit -m "test: add serialization, numeric scaling, and isFinitePositive tests"
```

---

### Task 6: Lint, verify full suite, update docs

**Step 1: Lint and format**

Run: `uv run ruff check src/ tests/ && uv run ruff format --check src/ tests/`

**Step 2: Full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision --ignore=tests/test_config_name_collisions.py --ignore=tests/test_external_loading.py -q`
Expected: all pass (377 existing + ~46 new = ~423 tests)

**Step 3: Update docs**

`docs/TODO.md` — mark FW-LOW-2 done:
```
- [x] **FW-LOW-2**: Add firmware unit test harness for state machine, COBS codec, and CRC (2026-03-04)
```

`docs/20260223_roadmap.md` — strike through FW-LOW-2 in Low Priority and Phase 4:
```
- ~~**FW-LOW-2**: No firmware unit tests (state machine, COBS, CRC)~~ (fixed: reachy_codec.h + ctypes cross-validation; 46 tests; 2026-03-04)
```

`CLAUDE.md` — add to Testing Notes section:
```
- Firmware codec tests (`tests/test_firmware_codec.py`) cross-validate C functions against Python via ctypes; requires `make -C hardware/tests` (auto-built by fixture)
```

Update test count in CLAUDE.md Testing Notes (375 → ~423).

**Step 4: Commit**

```bash
git add docs/TODO.md docs/20260223_roadmap.md CLAUDE.md
git commit -m "docs: mark FW-LOW-2 firmware test harness complete"
```
