---
title: Healthy Heartrate Breathing
emoji: ❤️
colorFrom: purple
colorTo: gray
sdk: static
pinned: false
tags:
  - reachy_mini
  - reachy_mini_python_app
---

# Healthy Heartrate Breathing

[![Watch the video](https://img.youtube.com/vi/HkTQhnZpP7Q/maxresdefault.jpg)](https://youtu.be/HkTQhnZpP7Q)

### [Reachy Mini with a mmWave heart rate sensor ](https://youtu.be/HkTQhnZpP7Q)

A wellness-aware conversation app for the Reachy Mini robot. It uses an mmWave radar sensor for presence detection, heart rate, and breathing rate measurement, and a lux sensor for proximity/occlusion detection (the sensor sits behind the user — low lux means someone is close, not that the room is dark).

Forked from the Reachy Mini conversation app. The original README is in `README_OLD.md`.

## Hardware Requirements

This app requires a custom sensor module connected via USB:

| Component | Link | Purpose |
|---|---|---|
| Reachy Mini robot | [pollen-robotics.com](https://www.pollen-robotics.com/reachy-mini/) | Robot host |
| **Seeed MR60BHA2** | [**Buy on SeeedStudio**](https://www.seeedstudio.com/MR60BHA2-60GHz-mmWave-Sensor-Breathing-and-Heartbeat-Module-p-5945.html) | 60 GHz mmWave radar (heart rate, breathing, presence) |
| BH1750 | — | Lux sensor for proximity/occlusion (I2C, `0x23`) |
| Seeed XIAO ESP32 | [seeedstudio.com](https://www.seeedstudio.com/Seeed-XIAO-ESP32S3-p-5627.html) | Microcontroller driving both sensors |
| USB cable | — | CDC serial at 115 200 baud to robot host |

Firmware source: `hardware/arduino/reachy-sensor/`. Protocol docs: [`hardware/README.md`](hardware/README.md).

## Sensor Overview

### mmWave Radar (`mmWave` tool)

The mmWave tool communicates with custom firmware on an Arduino-connected mmWave radar module over USB serial. It uses a compact binary protocol (`mmwave_protocol.py`) for all communication.

**Modes:**

| Mode | Purpose | Default Duration |
|---|---|---|
| `scan` | Detect people in the field of view (targets telemetry) | 8 s |
| `measure` | Read heart rate and breathing rate from a focused target | 15 s |
| `locate_and_measure` | Scan first, then measure the closest target | 8 s scan + 15 s measure |

**What it returns:**

- **Targets**: Cluster ID, x/y position, range (m), bearing, velocity; `max_target_count` and `targets_truncated` flag (set when >8 targets exceed the wire cap)
- **Bio**: Heart rate (bpm), breathing rate (bpm), validity flags
- **Light**: Ambient lux readings collected throughout the session
- **Device State**: Firmware person state (`NO_TARGET`, `MULTI_TARGET`, `PRESENT_FAR`, `MOVING`, `STILL_NEAR`, `RESTING_VITALS`) surfaced as `device_state` in scan/measure results
- **Device Context**: Structured `device_context` dict injected into tool results with `vitals_reliability` (high/moderate/low/unavailable), state transition detection, and contextual notes for LLM conversation guidance

The tool picks the closest valid target during scan and automatically focuses the radar on that cluster for bio measurement. An optional sweep mode rotates the robot head left-center-right to widen the scan field.

### Idle Scanning Policy (`idle_policy.py`)

When the robot has no active conversation, an `IdlePolicy` state machine controls periodic mmWave probes to detect approaching people. The policy escalates from passive probes to active head sweeps after repeated misses.

When the sensor disconnects (USB unplug, serial errors), the policy tracks consecutive errors and suppresses probing after a configurable threshold, backing off for `error_backoff_s` seconds before retrying. Successful communication resets the error counter automatically.

When multiple targets are detected (`max_target_count > 1`), the policy backs off probing (interval multiplied by `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER`, default 2x) and switches to scan-only mode (skipping the measure phase since vitals require a single stationary person). A protocol version handshake at session start (`CMD_PING` → version byte validation) fails fast on firmware/host mismatch.

```
    WAITING (idle timer not met or robot moving)
        │
        │ idle_duration > probe_interval_s AND not moving
        ▼
    TRIGGER GATE (post-focus quiet expired? sensor error backoff expired?)
        │
        ├── No  → suppress, stay WAITING
        │
        ├── Yes, misses < N → PASSIVE PROBE (no head sweep)
        │       │
        │       ├── target found   → reset misses + errors, enter POST_FOCUS_QUIET
        │       ├── no target      → misses += 1, reset errors
        │       ├── error          → errors += 1 (suppress after threshold)
        │       └── inconclusive   → no change
        │
        └── Yes, misses >= N, cooldown expired → ACTIVE SWEEP (head rotates L/C/R)
                │
                ├── target found   → reset misses + errors, enter POST_FOCUS_QUIET
                ├── no target      → reset misses (sweep counts as full attempt)
                ├── error          → errors += 1 (suppress after threshold)
                └── inconclusive   → no change
```

**Parameters** (all configurable via env vars):

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S` | 40.0 | Seconds idle before probe |
| `HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S` | 5.0 | Duration of each probe |
| `HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP` | 3 | Consecutive misses before escalation |
| `HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S` | 150.0 | Seconds between sweeps |
| `HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S` | 45.0 | Quiet window after target found |
| `HEALTHY_MM_WAVE_ERRORS_BEFORE_SUPPRESSION` | 3 | Consecutive errors before suppression |
| `HEALTHY_MM_WAVE_ERROR_BACKOFF_S` | 120.0 | Seconds to wait before retry after suppression |
| `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER` | 2.0 | Probe interval multiplier when multi-target active |

Full state diagram and transition details in `idle_policy.py` module docstring.

**Serial port resolution** (in order):
1. Explicit `serial_port` parameter
2. `MMWAVE_SERIAL_PORT` environment variable
3. Auto-detect using three-tier strategy:
   - **VID/PID match**: Filter by USB device signature (VID `0x303A`, PID `0x1001` for Seeed XIAO ESP32)
   - **Glob fallback**: Scan `/dev/cu.usbmodem*`, `/dev/tty.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*` if VID/PID unavailable
   - **HELLO probe**: When multiple candidates exist, probe each with `CMD_PING` or DTR reset → `EVT_HELLO` handshake to confirm firmware

### Proximity/Occlusion Context (`light_context` tool)

The BH1750 lux sensor sits physically behind/below the person at the Reachy Mini robot. When someone sits down, their body occludes the sensor and lux drops dramatically. The `light_context` tool reframes lux as a **proximity/occlusion signal** that complements mmWave radar data.

**Context states (in precedence order):**

```
lux data available?
  No  --> neutral (lux_missing)
  Yes --> sharp lux drop + presence?
            Yes --> sudden_occlusion (engaged)
            No  --> low lux + presence?
                      Yes --> close_presence (engaged)
                      No  --> moderate lux + presence?
                                Yes --> partial_occlusion (balanced)
                                No  --> clear_path (balanced)
```

**Lux thresholds (defaults):**

| Threshold | Default | Env Variable |
|---|---|---|
| Low lux | 40 lux | `HEALTHY_LIGHT_LOW_LUX_THRESHOLD` |
| Moderate lux | 120 lux | `HEALTHY_LIGHT_MODERATE_LUX_THRESHOLD` |
| Sharp drop | 60 lux decrease in 60 s | `HEALTHY_LIGHT_SHARP_DROP_LUX` |

**Lux source**: Can be passed directly via the `lux` parameter, or auto-extracted from an `mmwave_result` dict (path: `measure.latest_light.lux`).

## Binary Protocol

The mmWave firmware speaks a COBS-framed binary protocol over serial at 115200 baud. Each frame contains:

| Field | Size | Description |
|---|---|---|
| Version | 1 byte | Protocol version (currently 1) |
| Message type | 1 byte | Command or event type |
| Sequence number | 2 bytes | TX sequence counter (little-endian) |
| Payload length | 2 bytes | Byte count of payload (little-endian) |
| Payload | variable | Type-specific data |
| CRC-16 | 2 bytes | CRC-16/CCITT-FALSE over all preceding fields |

Encoding and decoding is handled by `mmwave_protocol.py`. The tool sends configuration commands (set heartbeat mode, focus cluster, report intervals) and receives event frames (targets, bio, state, light).

## Configuration

All configuration is via environment variables. Key variables:

| Variable | Default | Description |
|---|---|---|
| `MMWAVE_SERIAL_PORT` | auto-detect | Serial port for mmWave sensor |
| `HEALTHY_LIGHT_CONTEXT_ENABLED` | `true` | Enable/disable light context tool |
| `HEALTHY_LIGHT_LOW_LUX_THRESHOLD` | `40` | Lux below this = close proximity |
| `HEALTHY_LIGHT_MODERATE_LUX_THRESHOLD` | `120` | Lux below this = partial occlusion |
| `HEALTHY_LIGHT_SHARP_DROP_LUX` | `60` | Lux drop in 60 s to trigger sudden_occlusion |
| `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` | `90` | Days before old analytics rows are pruned |
| `HEALTHY_DISABLED_TOOLS` | *(empty)* | Comma-separated tool names to disable |
| `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` | `30` | Max seconds per tool call before timeout |

## Tool System

Tools are loaded from the active profile's `tools.txt`. Each tool is a Python class subclassing `Tool` from `core_tools.py`. The current profile enables:

- `dance` / `stop_dance` -- expressive body movements
- `play_emotion` / `stop_emotion` -- facial animations
- `mmWave` -- radar sensor (see above)
- `light_context` -- proximity/occlusion context (see above)
- `sweep_look` -- rotate head to look around

Custom tools can be added by creating a `.py` file in the profile folder that defines a `Tool` subclass.

## Project Structure

```
src/healthy_heartrate_breathing/
  config.py                 -- app configuration (env vars, profile selection)
  env_utils.py              -- shared coercion & env-var helpers
  openai_realtime.py        -- WebSocket lifecycle, reconnection, session orchestration
  audio_router.py           -- receive/emit: decode base64 audio deltas, feed head wobbler, enqueue output
  idle_policy.py            -- idle detection state machine, mmWave probe scheduling
  tool_dispatcher.py        -- non-blocking tool dispatch with timeout, serialisation, sensor state extraction, and device_context enrichment
  transcript_handler.py     -- partial transcript debouncing and completed transcript output routing
  light_orchestrator.py     -- lux delta tracking, auto light_context dispatch after mmWave
  tools/
    core_tools.py           -- Tool base class, registry, dispatcher
  profiles/
    _healthy_heartrate_breathing_locked_profile/
      instructions.txt      -- system prompt for the conversation model
      tools.txt             -- enabled tools list
      mmWave.py             -- mmWave radar tool
      mmwave_protocol.py    -- binary protocol encoder/decoder
      light_context.py      -- proximity/occlusion context classifier
      sweep_look.py         -- head sweep tool
```

## Customization

Use the `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile` folder to customize:

- `instructions.txt` -- system prompt and behavior rules
- `tools.txt` -- which tools are available to the model
- Add custom tools by creating `.py` files with `Tool` subclasses

Also customize:
- `index.html` -- Hugging Face Spaces landing page
- `src/healthy_heartrate_breathing/static/index.html` -- web app settings page with sensor dashboard (headless mode)

## Gradio Sensor Dashboard

When running with `--gradio`, the web UI shows a real-time sensor dashboard:

- **Live vitals**: heart rate, breathing rate, device state, target count, proximity — updated in real-time via WebSocket
- **Radar view**: top-down canvas showing detected people with range rings and colored position dots
- **Vitals history**: rolling Chart.js graph of HR and BR over the last 2–4 hours (persisted to SQLite)
- **Conversation log**: chatbot transcript in a collapsed accordion

The dashboard connects via WebSocket (`/ws/sensor`) for instant updates and fetches historical data from `/api/vitals/history`.
