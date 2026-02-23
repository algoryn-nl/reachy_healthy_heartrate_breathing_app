---
title: Healthy Heartrate Breathing
emoji: 🤖
colorFrom: purple
colorTo: gray
sdk: static
pinned: false
tags:
  - reachy_mini
  - reachy_mini_python_app
---

# Healthy Heartrate Breathing

A wellness-aware conversation app for the Reachy Mini robot. It uses an mmWave radar sensor for presence detection, heart rate, and breathing rate measurement, and an ambient light sensor (lux) to adapt conversation tone to the environment.

Forked from the Reachy Mini conversation app. The original README is in `README_OLD.md`.

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

- **Targets**: Cluster ID, x/y position, range (m), bearing, velocity
- **Bio**: Heart rate (bpm), breathing rate (bpm), validity flags
- **Light**: Ambient lux readings collected throughout the session
- **State**: Firmware state transitions

The tool picks the closest valid target during scan and automatically focuses the radar on that cluster for bio measurement. An optional sweep mode rotates the robot head left-center-right to widen the scan field.

**Serial port resolution** (in order):
1. Explicit `serial_port` parameter
2. `MMWAVE_SERIAL_PORT` environment variable
3. Auto-detect using three-tier strategy:
   - **VID/PID match**: Filter by USB device signature (VID `0x303A`, PID `0x1001` for Seeed XIAO ESP32)
   - **Glob fallback**: Scan `/dev/cu.usbmodem*`, `/dev/tty.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*` if VID/PID unavailable
   - **HELLO probe**: When multiple candidates exist, probe each with `CMD_PING` or DTR reset → `EVT_HELLO` handshake to confirm firmware

### Ambient Light Context (`light_context` tool)

The light context tool classifies the current ambient environment based on lux level, time of day, and user preferences. It outputs a `context_state` and `recommended_mode` that the conversation model uses to adapt its tone.

**Context states (in precedence order):**

```
lux data available?
  No  --> neutral (lux_missing)
  Yes --> sharp lux drop + active + presence?
            Yes --> unexpected_darkening (quiet)
            No  --> nighttime + low lux + presence?
                      Yes --> night_wind_down (quiet)
                      No  --> daytime + low lux + user prefers dim?
                                Yes --> intentional_dim_work (balanced/quiet)
                                No  --> daytime + low lux + prolonged duration?
                                          Yes --> prolonged_low_light_strain_risk (quiet)
                                          No  --> daytime + bright lux + presence?
                                                    Yes --> bright_active (active)
                                                    No  --> neutral
```

Order matters: a sharp lux drop at night is classified as `unexpected_darkening`, not `night_wind_down`. User preferences (`prefers_dim`, `light_sensitive`) prevent strain-risk nudges.

**Lux thresholds (defaults):**

| Threshold | Default | Env Variable |
|---|---|---|
| Low lux | 40 lux | `HEALTHY_LIGHT_LOW_LUX_THRESHOLD` |
| Bright lux | 250 lux | `HEALTHY_LIGHT_BRIGHT_LUX_THRESHOLD` |
| Sharp drop | 80 lux decrease in 60 s | `HEALTHY_LIGHT_SHARP_DROP_LUX` |
| Prolonged low-light | 45 min | `HEALTHY_LIGHT_PROLONGED_MIN` |

**Lux source**: Can be passed directly via the `lux` parameter, or auto-extracted from an `mmwave_result` dict (path: `measure.latest_light.lux`).

## Binary Protocol

The mmWave firmware speaks a COBS-framed binary protocol over serial at 115200 baud. Each frame contains:

| Field | Size | Description |
|---|---|---|
| Version | 1 byte | Protocol version (currently 1) |
| Message type | 1 byte | Command or event type |
| Sequence number | 2 bytes | TX sequence counter (little-endian) |
| Payload | variable | Type-specific data |

Encoding and decoding is handled by `mmwave_protocol.py`. The tool sends configuration commands (set heartbeat mode, focus cluster, report intervals) and receives event frames (targets, bio, state, light).

## Configuration

All configuration is via environment variables. Key variables:

| Variable | Default | Description |
|---|---|---|
| `MMWAVE_SERIAL_PORT` | auto-detect | Serial port for mmWave sensor |
| `HEALTHY_LIGHT_CONTEXT_ENABLED` | `true` | Enable/disable light context tool |
| `HEALTHY_LIGHT_LOW_LUX_THRESHOLD` | `40` | Lux below this is "low" |
| `HEALTHY_LIGHT_BRIGHT_LUX_THRESHOLD` | `250` | Lux above this is "bright" |
| `HEALTHY_LIGHT_SHARP_DROP_LUX` | `80` | Lux drop in 60 s to trigger unexpected_darkening |
| `HEALTHY_LIGHT_PROLONGED_MIN` | `45` | Minutes of low light before strain-risk nudge |
| `HEALTHY_DISABLED_TOOLS` | *(empty)* | Comma-separated tool names to disable |
| `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` | `30` | Max seconds per tool call before timeout |

## Tool System

Tools are loaded from the active profile's `tools.txt`. Each tool is a Python class subclassing `Tool` from `core_tools.py`. The current profile enables:

- `dance` / `stop_dance` -- expressive body movements
- `play_emotion` / `stop_emotion` -- facial animations
- `mmWave` -- radar sensor (see above)
- `light_context` -- ambient light classification (see above)
- `sweep_look` -- rotate head to look around

Custom tools can be added by creating a `.py` file in the profile folder that defines a `Tool` subclass.

## Project Structure

```
src/healthy_heartrate_breathing/
  config.py                 -- app configuration (env vars, profile selection)
  env_utils.py              -- shared coercion & env-var helpers
  openai_realtime.py        -- WebSocket lifecycle, reconnection, session orchestration
  audio_router.py           -- mic/speaker frame routing (resample, emit audio deltas)
  idle_policy.py            -- idle detection state machine, mmWave probe scheduling
  tool_dispatcher.py        -- non-blocking tool dispatch with timeout and serialisation
  transcript_handler.py     -- conversation transcript capture and formatting
  light_orchestrator.py     -- auto-invokes light context after mmWave lux data
  tools/
    core_tools.py           -- Tool base class, registry, dispatcher
  profiles/
    _healthy_heartrate_breathing_locked_profile/
      instructions.txt      -- system prompt for the conversation model
      tools.txt             -- enabled tools list
      mmWave.py             -- mmWave radar tool
      mmwave_protocol.py    -- binary protocol encoder/decoder
      light_context.py      -- ambient light context classifier
      sweep_look.py         -- head sweep tool
```

## Customization

Use the `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile` folder to customize:

- `instructions.txt` -- system prompt and behavior rules
- `tools.txt` -- which tools are available to the model
- Add custom tools by creating `.py` files with `Tool` subclasses

Also customize:
- `index.html` -- Hugging Face Spaces landing page
- `src/healthy_heartrate_breathing/static/index.html` -- web app parameters page
