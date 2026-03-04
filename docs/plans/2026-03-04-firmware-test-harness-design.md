# FW-LOW-2: Firmware Unit Test Harness — Design

## Problem

The firmware (`reachy-sensor.ino`, 1323 lines) has no unit tests. Pure functions like COBS encode/decode, CRC-16, buffer serialization, and numeric scaling are verified only indirectly via the Python protocol tests. A bug in the C implementation could go undetected if it produces different output than the Python codec.

## Approach: Extract header + shared lib + pytest/ctypes

Extract the ~15 pure functions into a header-only file. Compile a thin C shim into a shared library. Test from pytest via ctypes, cross-validating C output against the Python `mmwave_protocol.py` codec.

## File Structure

```
hardware/
  arduino/reachy-sensor/
    reachy_codec.h          ← extracted pure functions (header-only, included by .ino)
    reachy-sensor.ino       ← #include "reachy_codec.h" (replaces inline definitions)
  tests/
    reachy_codec_shim.c     ← extern "C" wrappers exporting functions for ctypes
    Makefile                ← cc -shared → libreachy_codec.{so,dylib}
tests/
  test_firmware_codec.py    ← pytest + ctypes cross-validation
```

## What Goes Into `reachy_codec.h`

15 pure functions (no globals, no hardware):

| Function | Purpose |
|----------|---------|
| `isFinitePositive` | NaN/Inf/zero/negative guard |
| `crc16CcittFalse` | CRC-16/CCITT-FALSE checksum |
| `cobsEncode` / `cobsDecode` | COBS framing codec |
| `appendU8/U16LE/I16LE/U32LE/I32LE/F32LE` | Little-endian buffer serialization |
| `readU16LE` / `readI16LE` | Little-endian buffer deserialization |
| `toI16Scaled` / `toU16ScaledOrNull` | Numeric scaling with NaN→sentinel |

Also: `FRAME_DELIMITER` constant.

**Not extracted** (depend on globals/hardware): `sendFrame`, `emitXxx`, `guessPose`, `pickClosestTarget`, `pickForcedCluster`, state machine, serial I/O.

## Test Cases

1. **CRC cross-validation** — same byte inputs → same u16 output as Python `crc16_ccitt_false()`
2. **COBS round-trip** — encode then decode recovers original; edge cases (empty, contains 0x00, all zeros, max-length blocks)
3. **COBS cross-validation** — C `cobsEncode` matches Python `cobs_encode` byte-for-byte
4. **Serialization** — `appendU16LE(0x1234)` → `[0x34, 0x12]`; `appendF32LE(1.0)` → IEEE-754 LE bytes; `readU16LE` round-trips
5. **toI16Scaled** — normal scaling, clamp at INT16_MIN/MAX, NaN→0, Inf→0
6. **toU16ScaledOrNull** — normal scaling, clamp at 65534, negative→0, NaN→0xFFFF sentinel
7. **isFinitePositive** — NaN=false, +Inf=false, -Inf=false, 0.0=false, -1.0=false, 0.001=true

## Build

The Makefile auto-detects macOS vs Linux for the shared library extension. Build is triggered by pytest via a conftest fixture or manually via `make -C hardware/tests`.

## Not in Scope

- Testing impure functions (emitters, state machine, serial I/O)
- Arduino IDE integration or CI compilation of the .ino
- Mocking the mmWave library structs
