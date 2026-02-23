# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Healthy Heartrate Breathing is a conversational app for the **Reachy Mini** robot. It is forked from the Reachy Mini Conversation App and adds mmWave radar-based health sensing (heart rate, breathing rate), ambient light context, and wellness-oriented behavior.

The app connects an OpenAI Realtime API audio stream to robot motion, vision, and sensor tools via a profile-driven tool/prompt system. It is a Reachy Mini App (entry point registered via `project.entry-points.reachy_mini_apps` in pyproject.toml).

## Common Commands

```bash
# Setup
uv venv --python python3.12 .venv && source .venv/bin/activate
uv sync --group dev              # install with dev tools

# Run
healthy-heartrate-breathing                   # headless console mode
healthy-heartrate-breathing --gradio          # web UI at http://127.0.0.1:7860/
healthy-heartrate-breathing --gradio --debug  # verbose logging

# Lint and format
ruff check .          # lint
ruff check --fix .    # auto-fix
ruff format .         # format

# Type check
mypy src/

# Tests
pytest                           # full suite
pytest tests/test_mmwave.py      # single file
pytest tests/test_mmwave.py -k test_name  # single test

# Hardware decode tool
uv run python hardware/tools/mmwave_decode.py --port /dev/cu.usbmodemXXXX --format pretty
```

## Architecture

### Startup Flow

`main.py:run()` orchestrates boot:
1. Connect to Reachy Mini robot (SDK auto-detects hardware/simulator)
2. Initialize camera worker, head tracker, and optional vision manager
3. Create `MovementManager` (dedicated thread, ~100 Hz control loop)
4. Create `HeadWobbler` (audio-reactive head motion)
5. Build `ToolDependencies` dataclass injected into all tools
6. Create `OpenaiRealtimeHandler` (WebSocket to OpenAI Realtime API)
7. Launch either Gradio web UI (`Stream`) or headless console (`LocalStream`)

### Realtime Audio Loop

`openai_realtime.py:OpenaiRealtimeHandler` extends `fastrtc.AsyncStreamHandler` and owns the WebSocket lifecycle (reconnection with exponential backoff, per-response cost tracking).

The session logic in `_run_realtime_session()` is decomposed into five handler classes, each receiving closures (not raw WebSocket) via callback injection:

| Handler | File | Responsibility |
|---------|------|----------------|
| `AudioRouter` | `audio_router.py` | receive/emit: resample mic frames to 24kHz mono, poll output queue for audio deltas and `AdditionalOutputs` |
| `IdlePolicy` | `idle_policy.py` | State machine for idle detection; triggers mmWave probes after inactivity |
| `ToolDispatcher` | `tool_dispatcher.py` | Non-blocking tool dispatch: `asyncio.create_task()` + `Semaphore(1)` + configurable timeout |
| `TranscriptHandler` | `transcript_handler.py` | Captures and formats conversation transcripts |
| `LightOrchestrator` | `light_orchestrator.py` | Auto-invokes light context analysis after mmWave returns lux data |

### Tool System

Tools are the primary extension point. The system is in `tools/core_tools.py`:

- **`Tool`** abstract base class: subclass it, define `name`, `description`, `parameters_schema`, and `async __call__`
- **Registry**: tools auto-register via subclass introspection (`get_concrete_subclasses(Tool)`)
- **Profile-driven loading**: `tools.txt` in the active profile lists enabled tools by name
- **Resolution order**: profile folder first, then `tools/` (core), then external directory
- **Dispatch**: `dispatch_tool_call(name, args_json, deps)` looks up and calls the tool

Built-in core tools live in `src/.../tools/` (dance, camera, head_tracking, etc.).
Profile-specific tools live in the profile folder (mmWave, light_context, sweep_look).

### Profile System

Profiles live under `src/.../profiles/<name>/` and contain:
- `instructions.txt` — LLM system prompt (supports `[placeholder]` expansion from `prompts/` library)
- `tools.txt` — enabled tool names, one per line
- `voice.txt` — optional OpenAI voice selection
- Python files — custom `Tool` subclasses

The locked profile `_healthy_heartrate_breathing_locked_profile` is set via `config.py:LOCKED_PROFILE` and cannot be changed at runtime.

External profiles/tools are supported via `REACHY_MINI_EXTERNAL_PROFILES_DIRECTORY` and `REACHY_MINI_EXTERNAL_TOOLS_DIRECTORY` env vars, with collision detection at startup.

### Movement System (`moves.py`)

- **Primary moves** (dance, emotion, goto, breathing) are mutually exclusive and queued
- **Secondary moves** (speech wobble, face tracking) are additive offsets
- Single control point: `ReachyMini.set_target` at ~100 Hz from a dedicated worker thread
- Other threads communicate via command queue; secondary offsets use locks

### Hardware: mmWave Sensor Module

The project includes a custom hardware component under `hardware/`.

**Physical setup**: Seeed MR60BHA2 60GHz mmWave radar sensor + BH1750 ambient light sensor (I2C, address `0x23`), driven by an Arduino-compatible XIAO microcontroller. The sensors connect to the host (robot) over USB CDC serial at 115200 baud.

**Firmware**: `hardware/arduino/reachy-sensor/reachy-sensor.ino`
- Reads mmWave data (presence, targets, distance, heart rate, breathing rate) and lux values
- Implements a state machine with person states: `NO_TARGET`, `MULTI_TARGET`, `PRESENT_FAR`, `MOVING`, `STILL_NEAR`, `RESTING_VITALS`
- Vitals gating: heart/breath rates only reported when single-target, still, not head-moving, within near zone (35–150cm)
- Guard rails: BR 4–30 bpm, HR 35–200 bpm
- Depends on Seeed mmWave library (git submodule in `hardware/arduino/lib/Seeed-mmWave-library/`) and Arduino BH1750 library

**Binary protocol** (`MMWAVE_PROTO_V1`): fully documented in `hardware/README.md`
- Transport: COBS-framed packets terminated by `0x00`, CRC-16/CCITT-FALSE verified, little-endian
- Host commands: `CMD_SET_HM` (head-moving mode), `CMD_SET_FOCUS` (target cluster), `CMD_SET_BIO_MS`/`CMD_SET_TARGETS_MS` (telemetry cadence), `CMD_PING`
- Device events: `EVT_STATE`, `EVT_TARGETS` (up to 8 targets with x/y/r/bearing/velocity), `EVT_BIO` (heart rate + breath rate in centi-bpm), `EVT_LIGHT` (lux as f32), `EVT_ACK`/`EVT_ERR`/`EVT_PONG`/`EVT_HELLO`

**Paired Python components** (firmware and host must match):
- Protocol codec: `profiles/_healthy_heartrate_breathing_locked_profile/mmwave_protocol.py` — encode/decode frames, COBS, CRC
- Tool: `profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py` — modes: `scan`, `measure`, `locate_and_measure`; runs serial I/O in `asyncio.to_thread`
  - Serial port auto-detection: three-tier strategy (VID/PID `0x303A:0x1001` → glob fallback → HELLO probe with `CMD_PING`/`EVT_PONG` or DTR reset → `EVT_HELLO`)
- Decode utility: `hardware/tools/mmwave_decode.py` — CLI for live serial or capture file decoding (`--format pretty|json`)

**Idle scanning policy** (in `OpenaiRealtimeHandler`):
- Calm periodic mmWave probing when the robot is idle
- Escalating sweep behavior: passive probes first, physical head sweep only after N consecutive misses with cooldown
- Post-focus quiet window suppresses probes after a confirmed target
- All timing configurable via `HEALTHY_MM_WAVE_*` env vars

### Light Context System

`light_context.py` is a policy tool that classifies ambient light conditions:
- Auto-invoked after mmWave returns lux data (orchestrated in `openai_realtime.py:_run_light_context_from_mmwave`)
- Maintains per-user rolling lux baseline (EMA per hour-of-day, persisted to JSON)
- Outputs context state, recommended conversation mode, and action suggestions
- JSONL analytics logging

### Key Dependencies

- `reachy_mini` / `reachy_mini_toolbox` / `reachy_mini_dances_library` — robot SDK and motion assets
- `fastrtc` — low-latency WebRTC audio streaming
- `openai` — Realtime API client
- `gradio` — web UI
- `pyserial` — mmWave serial communication
- `eclipse-zenoh` — robot pub/sub transport

## Code Style

- **Ruff** for linting and formatting (line length 119)
- Lint rules: E, F, W, I, C4, D (with D100, D203, D213 ignored)
- isort: length-sort, two blank lines after imports
- Quote style: double quotes
- `mypy --strict` with `ignore_missing_imports`

## Environment Variables

Key configuration (see `.env.example`):
- `OPENAI_API_KEY` — required
- `REACHY_MINI_CUSTOM_PROFILE` — select a profile (overridden by `LOCKED_PROFILE` in config.py)
- `MMWAVE_SERIAL_PORT` — override auto-detection of mmWave USB port
- `HEALTHY_DISABLED_TOOLS` — comma-separated tool names to disable at runtime
- `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` — tool dispatch timeout in seconds (default 30)
- `HEALTHY_MM_WAVE_*` — idle scanning policy tuning (intervals, sweep cooldown, etc.)
- `HEALTHY_LIGHT_*` / `HEALTHY_AUTO_LIGHT_CONTEXT_ENABLED` — light context policy tuning

## Testing Notes

- `conftest.py` sets `REACHY_MINI_SKIP_DOTENV=1` and clears profile env vars for isolation
- Tests do not require a connected robot or OpenAI key
- The tool registry is module-global and initializes on first import of `core_tools`

## Documentation Maintenance

When completing a feature or fix, **always update all four documentation surfaces**:

1. `CLAUDE.md` — Architecture, env vars, handler table
2. `README.md` — Project structure, configuration table, user-facing descriptions
3. `docs/TODO.md` — Move task to Done, update technical debt notes
4. `docs/20260223_roadmap.md` — Remove from Known Issues, update Future Vision

Do not consider work complete until all relevant docs are synced.

## Project Documentation

- `docs/TODO.md` — Development log, task tracker, technical debt notes
- `docs/20260223_product_requirements.md` — Current-state product specification
- `docs/20260223_roadmap.md` — Known issues and future vision
- `docs/plans/` — Design documents and implementation plans
