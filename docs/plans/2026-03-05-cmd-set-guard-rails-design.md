# FW-MED-4: CMD_SET_GUARD_RAILS — Design

## Problem

The firmware's vitals acceptance ranges (BR 4–30 bpm, HR 35–200 bpm) are compile-time constants. Different users, sensor placements, or clinical contexts may need different ranges without reflashing.

## Solution: CMD_SET_GUARD_RAILS (0x06)

A new host command that tunes the 4 vitals guard rail values at runtime. Defaults match the current compile-time constants. Values reset to defaults on power cycle.

## Wire Format

**Command payload (8 bytes, little-endian):**

| Offset | Type | Field | Default (centi-bpm) |
|--------|------|-------|---------------------|
| 0 | u16 | br_min | 400 (4.0 bpm) |
| 2 | u16 | br_max | 3000 (30.0 bpm) |
| 4 | u16 | hr_min | 3500 (35.0 bpm) |
| 6 | u16 | hr_max | 20000 (200.0 bpm) |

Values are in **centi-bpm** (bpm × 100), matching EVT_BIO's existing wire encoding.

## Validation

Firmware clamps each value to absolute physiological bounds:
- BR: [100, 6000] → 1–60 bpm
- HR: [2000, 30000] → 20–300 bpm

After clamping, if min ≥ max for either pair, reject with `ERR_BAD_VALUE`.

## Response

`EVT_ACK` with `ACK_OK` or `ACK_CLAMPED`. ACK value field is 0 (applied values are 4 fields; too many for the single i32 ACK value).

## Firmware Changes

1. Add `CMD_SET_GUARD_RAILS = 0x06` protocol constant
2. Add 4 float fields to `HostSettings`: `brMin`, `brMax`, `hrMin`, `hrMax` (initialized from compile-time defaults `BR_MIN`, `BR_MAX`, `HR_MIN`, `HR_MAX`)
3. Add handler in `applyBinaryCommand()` — validate payload length (8), decode 4× u16 centi-bpm, convert to float, clamp to absolute bounds, validate min < max, apply to `host`, emit ACK
4. Replace `BR_MIN`/`BR_MAX`/`HR_MIN`/`HR_MAX` references in `loop()` vitals gating (lines ~1097–1098) with `host.brMin`/`host.brMax`/`host.hrMin`/`host.hrMax`

## Python-Side Changes

1. Add `CMD_SET_GUARD_RAILS = 0x06` to `mmwave_protocol.py`
2. Add `pack_cmd_set_guard_rails(br_min_bpm, br_max_bpm, hr_min_bpm, hr_max_bpm)` — converts float bpm to centi-bpm u16
3. No ACK decode changes needed (generic ACK format)

## Tests

- Protocol: encode/decode round-trip for CMD_SET_GUARD_RAILS frame
- Clamping: values outside absolute bounds get clamped to limits
- Min ≥ max rejection: ERR_BAD_VALUE
- Default values: verify HostSettings initializes to compile-time defaults

## Not in Scope

- Near zone distance tuning (hardware/physics constraint)
- Movement threshold tuning
- Hysteresis timing tuning
- Persisting guard rails across power cycles (EEPROM)
