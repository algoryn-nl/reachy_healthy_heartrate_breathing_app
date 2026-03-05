# Product Requirements: Healthy Heartrate Breathing

## Purpose

Wellness-aware conversation app for the Reachy Mini robot. Combines an OpenAI Realtime API audio stream with mmWave radar health sensing (heart rate, breathing rate), ambient light context, and adaptive robot behavior. Forked from the Reachy Mini Conversation App.

## Target Users

- Reachy Mini robot owners and operators
- Wellness and assistive robotics researchers

## Core Features

### 1. Conversational Audio (OpenAI Realtime API)

- Bidirectional audio streaming via WebSocket (resampled to 24kHz mono)
- Audio-reactive head motion via HeadWobbler
- WebSocket reconnection with exponential backoff
- Per-response cost tracking
- Profile-driven system prompt (`instructions.txt`) with placeholder expansion from prompts library

### 2. mmWave Radar Sensing

Three modes:

| Mode | Purpose | Default Duration |
|---|---|---|
| `scan` | Detect people in the field of view (targets telemetry) | 8 s |
| `measure` | Read heart rate and breathing rate from a focused target | 15 s |
| `locate_and_measure` | Scan first, then measure the closest target | 8 s scan + 15 s measure |

Returns:
- **Targets**: up to 8 clusters with x/y position, range (m), bearing, velocity; `max_target_count` (highest count seen in session), `targets_truncated` flag (set when sensor detected more targets than the 8-target wire cap)
- **Bio**: heart rate (bpm), breathing rate (bpm), validity flags
- **Light**: ambient lux readings collected throughout the session
- **Device State**: firmware person state (`device_state`) surfaced as a named field in scan and measure results: NO_TARGET, MULTI_TARGET, PRESENT_FAR, MOVING, STILL_NEAR, RESTING_VITALS

Vitals gating: heart/breath rates only reported when single-target, still, not head-moving, within near zone (35-150 cm). Guard rails: BR 4-30 bpm, HR 35-200 bpm.

Serial port resolution order: explicit parameter, `MMWAVE_SERIAL_PORT` env var, three-tier auto-detection (VID/PID `0x303A:0x1001` for Seeed XIAO ESP32, glob fallback to `/dev/cu.usbmodem*`, `/dev/tty.usbmodem*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, HELLO probe to disambiguate multiple candidates).

Protocol version handshake: at session start, `_handshake_version()` sends `CMD_PING` and validates the first response frame's version byte, returning `status: "version_mismatch"` on mismatch or timeout. Defense-in-depth: mismatched frames are also drop-and-warned in the main polling loop.

### 3. Proximity / Occlusion Context

The BH1750 lux sensor sits behind/below the person — when someone sits down, their body occludes the sensor and lux drops. This is used as a proximity signal, not an ambient light measurement.

Context states: `clear_path`, `close_presence`, `sudden_occlusion`, `partial_occlusion` (+ `neutral` when lux missing). Auto-invoked after mmWave returns lux data via `LightOrchestrator`.

Default thresholds: low lux 40 (close proximity), moderate lux 120 (partial occlusion), sharp drop 60 lux in 60s (sudden occlusion). All configurable via `HEALTHY_LIGHT_*` env vars. SQLite analytics storage with time-based retention (default 90 days).

### 4. Idle Scanning Policy

Calm periodic mmWave probing when the robot is idle:
- Passive probes at configurable intervals
- Escalating to physical head sweep after N consecutive misses
- Sweep cooldown to prevent repetitive scanning
- Post-focus quiet window suppresses probes after a confirmed target
- **Disconnect handling**: consecutive sensor errors tracked; after configurable threshold, probing suppressed with backoff before retry; auto-recovery on successful communication
- **Multi-target awareness**: when >1 target detected, probe interval multiplied (default 2x via `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER`), scan-only mode used (measure phase skipped since vitals require single person), system prompt guides LLM to avoid private health topics
- All timings configurable via `HEALTHY_MM_WAVE_*` env vars

### 5. Movement System

- **Primary moves** (mutually exclusive, queued): dance, emotion, goto, breathing
- **Secondary moves** (additive offsets): speech wobble, face tracking
- Single control point: `ReachyMini.set_target` at ~100 Hz from dedicated worker thread
- Other threads communicate via command queue; secondary offsets use locks

### 6. Tool System

- `Tool` abstract base class with `name`, `description`, `parameters_schema`, `async __call__`
- Auto-discovery via subclass introspection (`get_concrete_subclasses(Tool)`)
- Profile-driven loading: `tools.txt` lists enabled tools by name
- Resolution order: profile folder, then `tools/` (core), then external directory
- Core dispatch: `dispatch_tool_call(name, args_json, deps)` looks up and calls the tool
- **Non-blocking dispatch** via `ToolDispatcher` class:
  - Fire-and-forget: `asyncio.create_task()` returns immediately to the event loop
  - Serialisation: `Semaphore(1)` ensures at most one tool runs at a time
  - Timeout: `asyncio.wait_for()` with configurable `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` (default 30 s)
  - Cancellation: `cancel()` method for session teardown
  - Integrates idle policy (mmWave probe overrides) and auto light-context orchestration
- External tools/profiles via `REACHY_MINI_EXTERNAL_*_DIRECTORY` env vars with collision detection

Current profile tools: `dance`, `stop_dance`, `play_emotion`, `stop_emotion`, `mmWave`, `light_context`, `sweep_look`, `vitals_trends`, `camera`, `move_head`, `head_tracking`, `do_nothing`.

### 7. UI Modes

- **Gradio web UI**: `healthy-heartrate-breathing --gradio` at `http://127.0.0.1:7860/`
  - **Sensor dashboard**: live vitals card (HR, BR, device state, target count, lux), radar canvas (top-down target view with range rings), vitals history chart (Chart.js rolling graph), trends panel (7-day averages, stats cards, recent insights)
  - Real-time updates via WebSocket (`/ws/sensor`), history via REST (`/api/vitals/history`, `/api/vitals/trends`)
  - Chatbot in collapsed accordion at bottom
- **Headless console**: `healthy-heartrate-breathing` (default)
  - Settings page includes a **sensor dashboard** panel showing person state, target count, truncation warnings, heart rate, breathing rate, and lux — updated via `GET /sensor` polling (3 s interval)
- Both support `--debug` for verbose logging

### 8. Handler Architecture

The realtime session (`_run_realtime_session()`) is decomposed into five handler classes, each receiving closures via callback injection:

| Handler | Responsibility |
|---------|----------------|
| `AudioRouter` | Decode base64 audio deltas, feed HeadWobbler, enqueue output |
| `IdlePolicy` | Idle detection state machine, mmWave probe scheduling, multi-target aware |
| `ToolDispatcher` | Non-blocking tool dispatch (asyncio.create_task + Semaphore(1) + timeout), sensor state extraction, trend insight injection |
| `TranscriptHandler` | Partial transcript debouncing and completed transcript routing |
| `LightOrchestrator` | Auto light_context dispatch after mmWave, lux delta tracking, analytics |

`HeadWobbler` runs in its own thread, converting audio amplitude to head pitch/yaw offsets.

### 9. Health Trend Analysis

Long-term vitals storage with trend detection and wellness insights.

**Two-tier aggregation** (same SQLite DB as VitalsStore):
- Raw readings: 4h rolling window (unchanged)
- Hourly aggregates: 30-day retention (avg/min/max HR, BR, lux, dominant state, resting minutes)
- Daily aggregates: 90-day retention (same schema, keyed by day)

**Anomaly detection** (either gate triggers):
1. Statistical: session avg > 1.5 stddev from 7-day rolling mean
2. Absolute: HR > 100 or < 45, BR > 25 or < 6

**Delivery**:
- Passive: trend insights injected into mmWave tool results for LLM to mention conversationally
- On-demand: `vitals_trends` tool returns 7-day summary when user asks
- Dashboard: trends panel with daily bar chart, stats cards, recent insights list
- Cooldown: 15 min between spoken insights (configurable)

All thresholds configurable via `HEALTHY_TREND_*` and `HEALTHY_VITALS_*` env vars.

## Hardware Requirements

| Component | Description |
|---|---|
| Reachy Mini robot | SDK auto-detects hardware vs simulator |
| Seeed MR60BHA2 | 60 GHz mmWave radar sensor |
| BH1750 | Ambient light sensor (I2C, address `0x23`) |
| XIAO microcontroller | Arduino-compatible, drives both sensors |
| USB cable | CDC serial at 115200 baud to host |

## Binary Protocol

COBS-framed packets terminated by `0x00`, CRC-16/CCITT-FALSE verified, little-endian. Each frame: version (1 byte) + message type (1 byte) + sequence number (2 bytes) + variable payload. Full specification in `hardware/README.md`.

## Configuration

All configuration via environment variables. See `.env.example` for defaults and `CLAUDE.md` for the full variable reference.

## Dependencies

- Python >=3.12
- Core: `reachy_mini`, `fastrtc`, `openai`, `gradio`, `pyserial`, `eclipse-zenoh`
- Optional vision: `torch`, `transformers`, `ultralytics` (via `yolo_vision` extra)
- Dev: `pytest`, `pytest-asyncio`, `ruff`, `mypy`, `pre-commit`
