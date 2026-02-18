# MR60BHA2 Reachy Firmware Binary Protocol (MMWAVE_PROTO_V1)

This firmware now uses a **binary-only** serial protocol for host commands and telemetry.

This is a **breaking switch** from the previous JSON over Serial format:
- Host software must support `MMWAVE_PROTO_V1`.
- Arduino Serial Monitor text parsing is no longer applicable.
- Use `hardware/tools/mmwave_decode.py` for readable inspection.

---

## 1) Transport

- Interface: USB CDC Serial (`Serial`)
- Baud: `115200`
- Framing: `COBS(packet) + 0x00`
- Endianness: little-endian for all multi-byte fields

### Packet layout (before COBS)

1. `u8 version` (currently `1`)
2. `u8 msg_type`
3. `u16 seq`
4. `u16 payload_len`
5. `payload[payload_len]`
6. `u16 crc16`

CRC details:
- Algorithm: `CRC-16/CCITT-FALSE`
- Poly: `0x1021`
- Init: `0xFFFF`
- XorOut: `0x0000`
- RefIn/RefOut: false
- CRC covers bytes `version..payload` (not CRC bytes)

---

## 2) Message IDs

### Host -> Device commands

- `0x01 CMD_SET_HM` payload: `u8 hm` (`0|1`)
- `0x02 CMD_SET_FOCUS` payload: `i16 cluster` (`-1` for auto focus)
- `0x03 CMD_SET_BIO_MS` payload: `u16 ms`
- `0x04 CMD_SET_TARGETS_MS` payload: `u16 ms`
- `0x05 CMD_PING` payload: none

### Device -> Host events

- `0x81 EVT_ACK` payload: `u8 cmd_id, u8 status_code, i32 value`
- `0x82 EVT_ERR` payload: `u8 cmd_id, u8 err_code`
- `0x83 EVT_PONG` payload: `u32 t_ms`
- `0x90 EVT_HELLO` payload: `u8 proto_version, u16 feature_bits`
- `0x91 EVT_STATE`
- `0x92 EVT_TARGETS`
- `0x93 EVT_BIO`
- `0x94 EVT_LIGHT`

---

## 3) Status and Error Codes

### ACK `status_code`

- `0` = `OK`
- `1` = `CLAMPED`
- `2` = `IGNORED`

### ERR `err_code`

- `1` = `UNKNOWN_CMD`
- `2` = `BAD_LEN`
- `3` = `BAD_VALUE`
- `4` = `CRC_FAIL`
- `5` = `UNSUPPORTED_VERSION`

---

## 4) Telemetry Payload Schemas

### 4.1 `EVT_STATE` (`0x91`)

Payload fields:
- `u32 t_ms`
- `u8 state_enum`
  - `0=NO_TARGET, 1=MULTI_TARGET, 2=PRESENT_FAR, 3=MOVING, 4=STILL_NEAR, 5=RESTING_VITALS`
- `u8 pose_enum`
  - `0=UNKNOWN, 1=SITTING, 2=STANDING`
- `u8 head_moving`
- `u8 human`
- `u8 n_targets`
- `u8 dist_new`
- `u16 dist_mm` (`0xFFFF` = null)

### 4.2 `EVT_BIO` (`0x93`)

Payload fields:
- `u32 t_ms`
- `u8 allowed`
- `u8 valid`
- `u8 br_new`
- `u8 hr_new`
- `u16 br_centi_bpm` (`0xFFFF` = null)
- `u16 hr_centi_bpm` (`0xFFFF` = null)

### 4.3 `EVT_TARGETS` (`0x92`)

Header fields:
- `u32 t_ms`
- `i16 forced_focus_cluster`
- `i16 focus_cluster` (`-1` when no focus)
- `i16 focus_x_mm`
- `i16 focus_y_mm`
- `u16 focus_r_mm`
- `i16 focus_bearing_cdeg`
- `i16 focus_v_cms_x10`
- `u8 flags`
  - `bit0`: focus valid
  - `bit1`: targets truncated
- `u8 n_targets`

Then `n_targets` entries:
- `i16 cluster`
- `i16 x_mm`
- `i16 y_mm`
- `u16 r_mm`
- `i16 bearing_cdeg`
- `i16 v_cms_x10`

Firmware cap:
- `MAX_TARGETS_WIRE = 8`
- If more targets exist, only first 8 are sent and `flags.bit1` is set.

### 4.4 `EVT_LIGHT` (`0x94`)

Payload fields:
- `u32 t_ms`
- `u8 valid`
- `f32 lux` (little-endian IEEE-754)

Cadence and source:
- Emitted at fixed `1s` cadence.
- Sensor: BH1750 on I2C address `0x23`.

Null semantics:
- When `valid=0`, wire payload sets `lux=NaN`.
- Python decoders normalize invalid/NaN lux to `None`.

---

## 5) Numeric scales and sentinels

- Distances/positions: millimeters (`mm`)
- Bearing: centi-degrees (`cdeg`)
- Velocity: deci-centimeters/second (`0.1 cm/s`)
- Bio rates: centi-bpm
- Illuminance: lux (`f32`)

Sentinels:
- `u16 0xFFFF` for missing nullable values (`dist_mm`, `br_centi_bpm`, `hr_centi_bpm`)
- `focus_cluster = -1` when no valid focus target
- `EVT_LIGHT.lux` uses `NaN` with `valid=0` for missing values

---

## 6) Command semantics (unchanged behavior)

The logical behavior remains the same as before:
- `HM=1` equivalent: enable head-moving mode (tracking/scan)
- `HM=0` equivalent: stable mode for vitals
- `FOCUS=-1` or specific cluster
- `BIO_MS` / `TARGETS_MS` cadence controls
- `PING` health check

Vitals gating and state machine logic are unchanged.
Light telemetry is passive-only and does not change command semantics.

---

## 7) Decoder utility

Use the repository tool:

```bash
uv run python hardware/tools/mmwave_decode.py --port /dev/cu.usbmodemXXXX --format pretty
```

Decode from a capture file:

```bash
uv run python hardware/tools/mmwave_decode.py --input-file capture.bin --format json
```

Options:
- `--baud` (default `115200`)
- `--show-bad-frames` to log decode failures
- Input file may be raw bytes or a text hex dump

---

## 8) Migration note

This firmware no longer emits JSON lines.

Required paired components:
1. Updated firmware: `hardware/arduino/reachy-sensor/reachy-sensor.ino`
2. Updated host parser: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py`

If either side is old, communication will fail.

---

## 9) Firmware dependencies

- Arduino BH1750 library (`BH1750.h`) is required for `EVT_LIGHT`.
