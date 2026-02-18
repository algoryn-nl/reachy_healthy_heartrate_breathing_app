# MR60BHA2 + XIAO ESP32C6 “Reachy-ready” Firmware (USB Serial Control + JSONL Telemetry)

This document describes the firmware sketch that:
- Talks to the **Seeed MR60BHA2 mmWave** module over a hardware UART (`mmWaveSerial`)
- Talks to the **host (Reachy / macOS / Python)** over **USB CDC Serial** (`Serial`)
- Emits **machine-readable JSON Lines** (one JSON object per line) for easy parsing
- Accepts newline-terminated **commands** from the host to control behavior (head motion gating, focus selection, output rates)

---

## 1) Serial Interfaces

### 1.1 Host Control + Telemetry (USB)
- **Interface:** `Serial` (USB CDC)
- **Baud:** `115200`
- **Direction:**
  - Host → ESP: commands such as `HM=1`
  - ESP → Host: JSONL telemetry (state/targets/bio) + JSON ack/error responses

> Note: On macOS, only one process can open the port. Close Arduino Serial Monitor when Python owns the port.

### 1.2 MR60BHA2 mmWave Sensor (UART)
- **Interface:** `mmWaveSerial`
- **Purpose:** communication with MR60BHA2 module via Seeed library `Seeed_Arduino_mmWave`

---

## 2) High-Level Behavior

The sketch implements a “two-mode” runtime model:

### 2.1 Multi-target room awareness
- Always tries to read `PeopleCounting` info (targets list).
- Emits periodic `targets` JSON containing:
  - Count of targets
  - Per-target approximate `(x,y)` in meters
  - Derived range `r` and bearing angle (rough)
  - Doppler-derived speed estimate

### 2.2 Focus + vitals measurement (breath/heart)
Vitals are **gated** to reduce false values:
- Only considered **allowed** when:
  - Exactly **one target** is detected (`nTargets == 1`)
  - Head is stable (`HM=0`, i.e., `head_moving == false`)

When allowed, vitals are checked against guard rails:
- `BR_MIN..BR_MAX` breaths/min
- `HR_MIN..HR_MAX` beats/min
- Must be valid for `VITALS_CONFIRM` consecutive windows before declaring `RESTING_VITALS`.

### 2.3 “Head moving” compensation strategy
Since Reachy’s head can move (Stewart platform), you should tell the ESP when the head is moving:
- `HM=1` while tracking/scanning
- `HM=0` when stable and ready to measure vitals

This prevents ego-motion from corrupting bio readings.

---

## 3) Host Commands (USB Serial)

All commands are:
- **ASCII**
- **Newline-terminated** (`\n`)
- Case-sensitive as written below

### 3.1 Head motion gating
#### `HM=0`
Head stable. Vitals measurement becomes *eligible* (if single target).

#### `HM=1`
Head moving. Vitals measurement becomes *ineligible* and vitals streak resets.

**ESP ACK:**
```json
{"type":"ack","hm":0}
```
or
```json
{"type":"ack","hm":1}
```

---

### 3.2 Focus selection (optional)
#### `FOCUS=-1`
Auto focus selection (default): firmware picks the **closest target**.

#### `FOCUS=<cluster_id>`
Force focus to a specific target cluster id (from `targets[].cluster`).

If the cluster id is not present in the current frame, firmware falls back to closest target.

**ESP ACK:**
```json
{"type":"ack","focus":-1}
```
or
```json
{"type":"ack","focus":3}
```

> Tip: Cluster IDs may not be stable in crowded scenes. The host can “refresh” focus by re-sending `FOCUS=<id>` each cycle while locked.

---

### 3.3 Output rate control
#### `BIO_MS=<ms>`
Controls how often the firmware emits a `bio` JSON message (in milliseconds).
- Minimum clamp is `50ms`.
- Default is `1000ms`.

**Examples:**
- `BIO_MS=250` (4 Hz)
- `BIO_MS=1000` (1 Hz)
- `BIO_MS=2000` (0.5 Hz)

**ESP ACK:**
```json
{"type":"ack","bio_ms":250}
```

#### `TARGETS_MS=<ms>`
Controls how often the firmware emits a `targets` JSON message.
- Minimum clamp is `50ms`.
- Default is `250ms`.

**ESP ACK:**
```json
{"type":"ack","targets_ms":250}
```

---

### 3.4 Ping/Pong
#### `PING`
Health check command.

**ESP response:**
```json
{"type":"pong","t_ms":12345}
```

---

### 3.5 Unknown command error
If the firmware doesn’t recognize a command:
```json
{"type":"err","msg":"unknown_cmd"}
```

---

## 4) JSONL Telemetry Output (ESP → Host)

The firmware prints **one JSON object per line** (“JSON Lines” / “jsonl”).
Each message includes `type` and `t_ms` (milliseconds since boot).

### 4.1 `hello` message (once at boot)
```json
{
  "type": "hello",
  "msg": "MR60BHA2 ready",
  "cmds": ["HM=0/1","FOCUS=-1/<cluster>","BIO_MS=<ms>","TARGETS_MS=<ms>","PING"]
}
```

---

### 4.2 `targets` message (multi-target + locations)
Emitted at cadence `TARGETS_MS` when targets exist.

```json
{
  "type": "targets",
  "t_ms": 12345,
  "n": 2,
  "forced_focus": -1,
  "focus": { "cluster": 4, "x": 0.12, "y": 1.34, "r": 1.35, "bearing": 5.1, "v": 0.00 },
  "targets": [
    { "cluster": 4, "x": 0.12, "y": 1.34, "r": 1.35, "bearing": 5.1, "v": 0.00 },
    { "cluster": 7, "x": -0.31, "y": 2.02, "r": 2.04, "bearing": -8.7, "v": 1.00 }
  ]
}
```

Field notes:
- `x`, `y` in **meters** (sensor coordinate frame)
- `r` is derived range `sqrt(x² + y²)` in meters
- `bearing` is computed with `atan2(x, y)` in degrees (axis convention may differ; calibrate)
- `v` is doppler-derived “speed” estimate in cm/s (`dop_index * RANGE_STEP`)

---

### 4.3 `state` message (classifier + environment snapshot)
Emitted when:
- state materially changes, or
- at least once per second as keepalive

```json
{
  "type": "state",
  "t_ms": 12500,
  "state": "RESTING_VITALS",
  "pose": "SITTING",
  "head_moving": 0,
  "human": 1,
  "n_targets": 1,
  "dist_cm": 80.12,
  "dist_new": 1
}
```

State definitions:
- `NO_TARGET`: no target for a sustained window (hysteresis applied)
- `MULTI_TARGET`: >1 target detected (vitals disabled)
- `MOVING`: either head moving (`HM=1`) or focus target moving above threshold
- `STILL_NEAR`: in the near zone, but vitals not yet stable/confirmed
- `RESTING_VITALS`: near + stable + vitals valid for `VITALS_CONFIRM` checks
- `PRESENT_FAR`: target present but outside near zone

Pose definitions:
- `SITTING`/`STANDING` are **distance-only heuristics**; calibrate `SIT_STAND_THRESHOLD_CM` for your geometry.

---

### 4.4 `bio` message (heart + breathing telemetry)
Emitted at cadence `BIO_MS` always, but includes `allowed` and `valid` flags.

```json
{
  "type": "bio",
  "t_ms": 13000,
  "allowed": 1,
  "valid": 1,
  "br": 12.00,
  "br_new": 1,
  "hr": 68.00,
  "hr_new": 1
}
```

Field notes:
- `allowed` means: `n_targets == 1` AND `head_moving == 0`
- `valid` means: allowed AND inside guard rails (and contributing to streak)
- `br_new`/`hr_new` are “fresh this frame” indicators from the library call

---

## 5) How to “Trigger scans” and “Trigger bio”

This firmware is designed so the **host** orchestrates behavior.

### 5.1 Trigger a scan (room awareness mode)
Set head moving and let targets stream:
1. `HM=1`
2. Optionally set `TARGETS_MS=200` to get faster tracking updates
3. Parse incoming `targets` JSON lines to get count and approximate positions

### 5.2 Lock onto a person and measure vitals
1. Host selects a person to focus on:
   - Either leave auto (`FOCUS=-1`) or set `FOCUS=<cluster>`
2. Move head to center the focus target
3. When stable:
   - Send `HM=0`
4. Optionally increase bio rate:
   - `BIO_MS=250`
5. Wait for `bio.valid==1` and/or `state.state=="RESTING_VITALS"`

### 5.3 Return to tracking
- Send `HM=1`

---

## 6) Key Configuration Knobs (in code)

### 6.1 Bio output rate
- Default:
  - `BIO_JSON_MS_DEFAULT = 1000`
- Runtime override:
  - `BIO_MS=<ms>` command

### 6.2 Targets output rate
- Default:
  - `TARGETS_JSON_MS_DEFAULT = 250`
- Runtime override:
  - `TARGETS_MS=<ms>` command

### 6.3 Vitals gating
Vitals are allowed only if:
- `singleTarget = (nTargets == 1)`
- `!headMoving` (host command `HM=0`)

### 6.4 Vitals stability confirmation
- `VITALS_CONFIRM`
- Higher = more stable, slower to enter `RESTING_VITALS`

### 6.5 Presence hysteresis
- `ABSENT_HOLD_MS` and `ABSENT_CONFIRM`
- Prevents flicker when the sensor drops frames briefly

---

## 7) Recommended Host (Python) Integration Pattern (macOS)

- Open port `/dev/cu.usbmodem*` at 115200
- Write commands with `\n`
- Read lines, parse JSON with `json.loads(line)`
- Maintain a state machine on the host:
  - **SCAN**: HM=1, parse targets, choose focus
  - **TRACK**: HM=1, keep focus near center, update focus cluster if needed
  - **MEASURE**: HM=0, expect `bio.allowed==1`, wait for `bio.valid==1`
  - **RESUME**: HM=1

---

## 8) Operational Notes / Caveats

- **Serial Monitor vs Python:** only one client can own the serial port.
- **Multi-target + vitals:** vitals are designed for single stationary target; firmware disables “confidence” in vitals unless exactly one target and head stable.
- **Cluster stability:** `cluster_index` can change; host should reselect focus frequently.
- **Bearing convention:** `bearing = atan2(x, y)` is a convenience; you may need to swap axes depending on physical orientation.

---

## 9) Quick Command Cheatsheet

```text
PING
HM=1              # head moving / scan or tracking mode
HM=0              # head stable / allow vitals (if 1 target)

FOCUS=-1          # auto focus (closest)
FOCUS=3           # force cluster 3

BIO_MS=250        # bio output every 250ms
BIO_MS=1000       # bio output every 1s

TARGETS_MS=200    # targets output every 200ms
TARGETS_MS=500    # targets output every 500ms
```

---

## 10) What Codex/AI Editor Should Know

This firmware:
- Provides a **clear serial command API** to orchestrate a robot head:
  - head motion gate (`HM`)
  - focus selection (`FOCUS`)
  - telemetry rate control (`BIO_MS`, `TARGETS_MS`)
- Emits **stable JSON schema** suitable for robust parsing and higher-level behavior.

Common next steps:
- Add a host command to set “mode” explicitly (SCAN/TRACK/MEASURE)
- Add “focus selection policy” (closest vs centerline vs forced)
- Add an on-demand snapshot command (e.g. `GET`) for immediate state reporting
