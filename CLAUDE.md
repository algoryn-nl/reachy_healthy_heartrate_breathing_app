# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Healthy Heartrate Breathing (v0.2.2) is a conversational app for the **Reachy Mini** robot. It is forked from the Reachy Mini Conversation App and adds mmWave radar-based health sensing (heart rate, breathing rate), ambient light context, and wellness-oriented behavior.

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

# CLI arguments
#   --head-tracker {yolo,mediapipe,None}  head tracker backend (default: None)
#   --no-camera                           disable camera usage
#   --local-vision                        use local vision model instead of gpt-realtime
#   --gradio                              open Gradio web UI (auto-enabled in simulation)
#   --debug                               enable DEBUG-level logging
#   --robot-name NAME                     Zenoh topic prefix (multi-robot dev setups)

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
2. Initialize camera worker (`camera_worker.py`), head tracker, and optional vision manager
3. Create `MovementManager` (dedicated thread, ~100 Hz control loop)
4. Create `HeadWobbler` (audio-reactive head motion)
5. Build `ToolDependencies` dataclass injected into all tools
6. Create `OpenaiRealtimeHandler` (WebSocket to OpenAI Realtime API)
7. Launch either Gradio web UI (`Stream`) or headless console (`LocalStream`)

Simulation mode is auto-detected — `--gradio` is enabled automatically when running against MuJoCo or desktop mockup.

### Realtime Audio Loop

`openai_realtime.py:OpenaiRealtimeHandler` extends `fastrtc.AsyncStreamHandler` and owns the WebSocket lifecycle (reconnection with exponential backoff, per-response cost tracking).

The session logic in `_run_realtime_session()` is decomposed into five handler classes, each receiving closures (not raw WebSocket) via callback injection:

| Handler | File | Responsibility |
|---------|------|----------------|
| `AudioRouter` | `audio_router.py` | receive/emit: decode base64 audio deltas, feed head wobbler, enqueue output |
| `IdlePolicy` | `idle_policy.py` | State machine for idle detection; triggers mmWave probes after inactivity; multi-target aware (backs off probing, suggests scan-only when >1 target) (state diagram in module docstring) |
| `ToolDispatcher` | `tool_dispatcher.py` | Non-blocking tool dispatch: `asyncio.create_task()` + `Semaphore(1)` + configurable timeout; extracts sensor state after mmWave calls via `extract_sensor_state()`; injects `device_context` dict into mmWave results via `build_device_context()` (vitals reliability, state transitions) |
| `TranscriptHandler` | `transcript_handler.py` | Partial transcript debouncing (configurable delay) and completed transcript output routing |
| `LightOrchestrator` | `light_orchestrator.py` | Auto-invokes light context analysis after mmWave returns lux data; owns baseline persistence and analytics |

### Tool System

Tools are the primary extension point. The system is in `tools/core_tools.py`:

- **`Tool`** abstract base class: subclass it, define `name`, `description`, `parameters_schema`, and `async __call__`
- **`ToolDependencies`** dataclass: injected into every tool call, provides `reachy_mini`, `movement_manager`, `camera_worker`, `vision_manager`, `head_wobbler`, `motion_duration_s`
- **Registry**: tools auto-register via subclass introspection (`get_concrete_subclasses(Tool)`)
- **Lazy initialization**: registry runs on first call to `get_tool_specs()` or `dispatch_tool_call()`, not at import time
- **Profile-driven loading**: `tools.txt` in the active profile lists enabled tools by name
- **Resolution order**: profile folder first, then `tools/` (core), then external directory
- **Dispatch**: `dispatch_tool_call(name, args_json, deps)` looks up and calls the tool
- **Disabled tools**: `HEALTHY_DISABLED_TOOLS` env var disables tools by name at runtime
- **Validation**: post-registration check warns if `tools.txt` requests tools that didn't register a `Tool` subclass
- **Errors**: `ToolRegistryError` (not `sys.exit`) on fatal init failures

Built-in core tools in `src/.../tools/`:

| Tool | File | Purpose |
|------|------|---------|
| `dance` | `dance.py` | Expressive body movements |
| `stop_dance` | `stop_dance.py` | Stop dancing |
| `play_emotion` | `play_emotion.py` | Facial animations |
| `stop_emotion` | `stop_emotion.py` | Stop emotion animation |
| `camera` | `camera.py` | Capture and send camera image |
| `do_nothing` | `do_nothing.py` | No-op tool |
| `head_tracking` | `head_tracking.py` | Toggle head tracking |
| `move_head` | `move_head.py` | Move head to position |

Profile-specific tools in `profiles/_healthy_heartrate_breathing_locked_profile/`:

| Tool | File | Purpose |
|------|------|---------|
| `mmWave` | `mmWave.py` | Radar sensor: scan, measure, locate_and_measure modes |
| `light_context` | `light_context.py` | Ambient light classification |
| `sweep_look` | `sweep_look.py` | Rotate head to widen scan field |
| `custom_tool` | `custom_tool.py` | Template for creating new tools |

### Profile System

Profiles live under `src/.../profiles/<name>/` and contain:
- `instructions.txt` — LLM system prompt (supports `[placeholder]` expansion from `prompts/` library)
- `tools.txt` — enabled tool names, one per line (comments with `#`, blank lines ignored)
- `voice.txt` — optional OpenAI voice selection (default: "cedar")
- Python files — custom `Tool` subclasses

The locked profile `_healthy_heartrate_breathing_locked_profile` is set via `config.py:LOCKED_PROFILE` and cannot be changed at runtime.

External profiles/tools are supported via `REACHY_MINI_EXTERNAL_PROFILES_DIRECTORY` and `REACHY_MINI_EXTERNAL_TOOLS_DIRECTORY` env vars, with collision detection at startup.

### Prompt System

`prompts.py` resolves session instructions and voice from the active profile:
- `get_session_instructions()` — loads `instructions.txt`, expands `[name]` placeholders from `prompts/` library files
- `get_session_voice()` — reads `voice.txt` or returns default "cedar"
- `_expand_prompt_includes()` — replaces `[name]` lines with content from `prompts/<name>.txt`

Prompts library structure (`src/.../prompts/`):
- `default_prompt.txt` — fallback when no profile selected
- `behaviors/` — behavioral instruction fragments (e.g., `silent_robot.txt`)
- `identities/` — identity fragments (e.g., `basic_info.txt`, `witty_identity.txt`)

### Personality Management

Two parallel UI systems for managing personalities (profiles):

**Gradio mode** (`gradio_personality.py`): `PersonalityUI` class creates Gradio components — dropdown, preview, editor, save/create. Wired to `handler.apply_personality()` for live updates.

**Headless mode** (`headless_personality.py` + `headless_personality_ui.py`):
- `headless_personality.py` — filesystem helpers: list/resolve/read/write profiles
- `headless_personality_ui.py:mount_personality_routes()` — REST API on the settings FastAPI app

User-created personalities are stored under `profiles/user_personalities/<name>/`.

### Movement System (`moves.py`)

- **Primary moves** (dance, emotion, goto, breathing) are mutually exclusive and queued
- **Secondary moves** (speech wobble, face tracking) are additive offsets
- Single control point: `ReachyMini.set_target` at ~100 Hz from a dedicated worker thread
- Other threads communicate via command queue; secondary offsets use locks

### Audio System (`audio/`)

- `head_wobbler.py` — `HeadWobbler`: audio-reactive head motion, runs in its own thread, receives base64 audio deltas and converts amplitude to head pitch/yaw offsets via `set_speech_offsets`
- `speech_tapper.py` — speech amplitude analysis utilities

### Vision System (`vision/`)

- `processors.py` — local vision model integration (requires `[local_vision]` extra)
- `yolo_head_tracker.py` — YOLO-based head tracking (requires `[yolo_vision]` extra)

### Headless Console Mode (`console.py`)

`LocalStream` manages bidirectional audio streaming without Gradio:
- Connects to Reachy Mini's recorder/player (GStreamer or default backend)
- Runs three async tasks: `openai-handler`, `stream-record-loop`, `stream-play-loop`
- API key resolution: instance `.env` → HuggingFace download → settings UI wait
- Settings UI served on the Reachy Mini Apps FastAPI settings server

### REST API (Headless Mode)

When running in headless mode with a settings app, these endpoints are mounted:

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Settings page (`static/index.html`) |
| `/status` | GET | Whether API key is configured |
| `/ready` | GET | Whether tools have finished initializing |
| `/openai_api_key` | POST | Set/persist OpenAI API key |
| `/validate_api_key` | POST | Validate API key against OpenAI |
| `/sensor` | GET | Latest mmWave sensor readings for dashboard |
| `/personalities` | GET | List profiles, current/startup selection, lock status |
| `/personalities/load` | GET | Load a profile's instructions, tools, voice |
| `/personalities/save` | POST | Save a user personality |
| `/personalities/apply` | POST | Apply a personality at runtime |
| `/voices` | GET | Available OpenAI voices |

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
- Device events: `EVT_STATE` (rate-limited to 200ms min interval on change), `EVT_TARGETS` (up to 8 targets with x/y/r/bearing/velocity), `EVT_BIO` (heart rate + breath rate in centi-bpm), `EVT_LIGHT` (lux as f32), `EVT_DIAG` (diagnostic counters every 10s: mmWave fail count, consecutive fails, TX drops), `EVT_ACK`/`EVT_ERR`/`EVT_PONG`/`EVT_HELLO`

**Paired Python components** (firmware and host must match):
- Protocol codec: `profiles/_healthy_heartrate_breathing_locked_profile/mmwave_protocol.py` — encode/decode frames, COBS, CRC
- Tool: `profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py` — modes: `scan`, `measure`, `locate_and_measure`; runs serial I/O in `asyncio.to_thread`
  - Serial port auto-detection: three-tier strategy (VID/PID `0x303A:0x1001` → glob fallback → HELLO probe with `CMD_PING`/`EVT_PONG` or DTR reset → `EVT_HELLO`)
  - Protocol version handshake: `_handshake_version()` sends `CMD_PING` at session start and validates the first response frame's version byte; fails fast with `status: "version_mismatch"` on mismatch or timeout (defense-in-depth: `_poll_events()` still drops and warns on mismatched frames)
- Decode utility: `hardware/tools/mmwave_decode.py` — CLI for live serial or capture file decoding (`--format pretty|json`)

**Idle scanning policy** (in `ToolDispatcher` via `IdlePolicy`; state diagram in `idle_policy.py` module docstring):
- Calm periodic mmWave probing when the robot is idle
- Two trigger gates: idle duration exceeded AND robot stationary AND post-focus quiet expired
- Escalating sweep behavior: passive probes first, physical head sweep only after N consecutive misses with cooldown
- Post-focus quiet window suppresses probes after a confirmed target
- **Disconnect handling**: consecutive sensor errors tracked; after `errors_before_suppression` (default 3) errors, probing suppressed for `error_backoff_s` (default 120s) before retry; successful communication auto-resets
- All timing configurable via `HEALTHY_MM_WAVE_*` env vars
- Full state diagram and transition table in `idle_policy.py` module docstring

### Sensor Dashboard (headless mode)

The headless settings page (`static/index.html` + `static/main.js` + `static/style.css`) includes a sensor dashboard panel that displays live mmWave readings:
- **Device state**: firmware person state (NO_TARGET, MOVING, STILL_NEAR, RESTING_VITALS, etc.)
- **Target count**: number of detected targets, with truncation warning if >8 targets exceed the wire cap
- **Heart rate / Breathing rate**: vitals when available (from `measure` or `locate_and_measure`)
- **Ambient light**: lux reading from the BH1750 sensor
- **Disconnected state**: when the sensor returns errors, the chip shows "Disconnected" with an error banner; stale data (>120s) shows "Stale" chip
- **Last scan mode and timestamp**

Data flows: mmWave tool result (including errors) → `extract_sensor_state()` in `tool_dispatcher.py` → `handler.sensor_state` dict (replaced, not merged, to clear stale error keys) → `GET /sensor` REST endpoint in `console.py` → frontend polls every 3 seconds.

### Light Context System

`light_context.py` is a policy tool that classifies ambient light conditions:
- Auto-invoked after mmWave returns lux data (orchestrated by `LightOrchestrator` via `ToolDispatcher._run_tool()`)
- Maintains per-user rolling lux baseline (EMA per hour-of-day, persisted to JSON; stale entries pruned on load after `HEALTHY_LIGHT_BASELINE_MAX_AGE_DAYS`, default 90)
- Baseline saves throttled with dirty flag + configurable interval (default 60s); flushed at shutdown
- Outputs context state, recommended conversation mode, and action suggestions
- SQLite analytics storage (when `HEALTHY_LIGHT_ANALYTICS_ENABLED` is true); time-based retention via `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` (default 90)

### Key Dependencies

- `reachy_mini` / `reachy_mini_toolbox` / `reachy_mini_dances_library` — robot SDK and motion assets
- `fastrtc` — low-latency WebRTC audio streaming
- `openai` — Realtime API client
- `gradio` — web UI
- `pyserial` — mmWave serial communication
- `eclipse-zenoh` — robot pub/sub transport
- `python-dotenv` — `.env` file loading
- `scipy` — audio resampling

Optional dependency groups (install with `uv sync --group dev` or `pip install '.[group]'`):
- `dev` — pytest, pytest-asyncio, ruff, mypy, pre-commit, python-semantic-release
- `local_vision` — torch, transformers, num2words
- `yolo_vision` — ultralytics, supervision
- `mediapipe_vision` — mediapipe
- `all_vision` — all vision backends combined
- `reachy_mini_wireless` — PyGObject, gst-signalling (GStreamer wireless backend)

## Project Structure

```
src/healthy_heartrate_breathing/
  main.py                   -- entrypoint: run(), HealthyHeartrateBreathing app class
  config.py                 -- app configuration (env vars, profile selection, collision detection)
  env_utils.py              -- shared coercion (coerce_bool/float/int/ms) & env-var helpers (env_flag/float/int), extract_lux_from_mmwave_result
  prompts.py                -- prompt resolution: get_session_instructions(), get_session_voice(), [placeholder] expansion
  utils.py                  -- CLI argument parsing, vision setup, logger setup
  openai_realtime.py        -- WebSocket lifecycle, reconnection, session orchestration, cost tracking
  audio_router.py           -- audio delta routing: decode base64, feed wobbler, enqueue output
  idle_policy.py            -- idle detection state machine, mmWave probe scheduling
  tool_dispatcher.py        -- non-blocking tool dispatch with timeout, serialisation, sensor state extraction, idle/light policy integration
  transcript_handler.py     -- partial transcript debouncing and completed transcript output routing
  light_orchestrator.py     -- lux baseline tracking (EMA per hour-of-day), auto light_context dispatch, analytics logging
  console.py                -- LocalStream: headless bidirectional audio, settings UI, REST endpoints
  gradio_personality.py     -- PersonalityUI: Gradio components for profile management
  headless_personality.py   -- filesystem helpers for headless personality management
  headless_personality_ui.py -- REST endpoints for headless personality management
  moves.py                  -- MovementManager: primary/secondary move system, ~100 Hz control loop
  dance_emotion_moves.py    -- dance and emotion move definitions
  camera_worker.py          -- camera frame buffering and head tracker integration
  audio/
    head_wobbler.py         -- HeadWobbler: audio-reactive head motion thread
    speech_tapper.py        -- speech amplitude analysis
  vision/
    processors.py           -- local vision model integration
    yolo_head_tracker.py    -- YOLO-based head tracking
  tools/
    core_tools.py           -- Tool base class, ToolDependencies, registry, dispatcher
    dance.py                -- dance tool
    stop_dance.py           -- stop dance tool
    play_emotion.py         -- play emotion tool
    stop_emotion.py         -- stop emotion tool
    camera.py               -- camera capture tool
    do_nothing.py           -- no-op tool
    head_tracking.py        -- head tracking toggle tool
    move_head.py            -- move head to position tool
  prompts/
    default_prompt.txt      -- fallback system prompt
    behaviors/              -- behavioral instruction fragments for [placeholder] expansion
    identities/             -- identity fragments for [placeholder] expansion
  profiles/
    _healthy_heartrate_breathing_locked_profile/
      instructions.txt      -- system prompt for the conversation model
      tools.txt             -- enabled tools list
      mmWave.py             -- mmWave radar tool
      mmwave_protocol.py    -- binary protocol encoder/decoder (COBS, CRC-16)
      light_context.py      -- ambient light context classifier
      sweep_look.py         -- head sweep tool
      custom_tool.py        -- template for creating new tools
  static/
    index.html              -- headless settings page with sensor dashboard
    main.js                 -- settings page JavaScript
    style.css               -- settings page styles
  images/                   -- avatar images for chatbot UI

tests/
  conftest.py               -- pytest config: sets REACHY_MINI_SKIP_DOTENV, clears profile env vars
  test_audio_router.py      -- AudioRouter unit tests
  test_config_name_collisions.py -- profile/tool collision detection tests
  test_env_utils.py         -- env_utils coercion and extraction tests
  test_external_loading.py  -- external tool/profile loading tests
  test_idle_policy.py       -- IdlePolicy state machine tests
  test_light_context.py     -- light_context tool classification tests
  test_light_orchestrator.py -- LightOrchestrator baseline tracking, dispatch, corrupted JSON / permission error handling
  test_mmwave.py            -- mmWave protocol, tool integration, bio rate boundary conditions, acceptance gate
  test_openai_realtime.py   -- OpenaiRealtimeHandler tests (receive, emit, idle, event loop, shutdown, personality, cost tracking)
  test_tool_dispatcher.py   -- ToolDispatcher dispatch, timeout, sensor extraction tests
  test_transcript_handler.py -- TranscriptHandler debouncing tests
  audio/                    -- audio subsystem tests
  vision/                   -- vision subsystem tests

hardware/
  README.md                 -- binary protocol specification
  arduino/
    reachy-sensor/          -- firmware source (.ino)
    lib/                    -- Seeed mmWave library (git submodule)
  tools/
    mmwave_decode.py        -- CLI decode utility for serial/capture files

docs/
  TODO.md                   -- development log, task tracker, technical debt notes
  20260223_product_requirements.md -- current-state product specification
  20260223_roadmap.md       -- known issues and future vision
  assets/                   -- documentation assets
  scheme.mmd                -- architecture diagram (Mermaid)
  plans/                    -- design documents and implementation plans
```

## Code Style

- **Ruff** for linting and formatting (line length 119)
- Lint rules: `E`, `F`, `W`, `I`, `C4`, `D` (with `E501`, `D100`, `D203`, `D213` ignored)
- isort: length-sort, two blank lines after imports, no lines before standard-library/local-folder
- Known first-party: `reachy_mini`, `reachy_mini_dances_library`, `reachy_mini_toolbox`
- Known local-folder: `healthy_heartrate_breathing`
- Quote style: double quotes, space indent, auto line endings
- `mypy --strict` with `ignore_missing_imports`, covers both `src/` and `tests/`
- Python target: `>=3.12` (aligned across `requires-python`, mypy, and runtime)

## Environment Variables

Key configuration (see `.env.example`):

### Core

| Variable | Default | Description |
|---|---|---|
| `OPENAI_API_KEY` | *(required)* | OpenAI API key |
| `MODEL_NAME` | `gpt-realtime` | OpenAI model name |
| `REACHY_MINI_CUSTOM_PROFILE` | *(none)* | Select a profile (overridden by `LOCKED_PROFILE` in config.py) |
| `REACHY_MINI_SKIP_DOTENV` | `false` | Skip `.env` file loading (used in tests) |
| `HEALTHY_DISABLED_TOOLS` | *(empty)* | Comma-separated tool names to disable at runtime |
| `HEALTHY_TOOL_DISPATCH_TIMEOUT_S` | `30` | Max seconds per tool call before timeout |

### Serial / mmWave

| Variable | Default | Description |
|---|---|---|
| `MMWAVE_SERIAL_PORT` | *(auto-detect)* | Override mmWave USB serial port |

### mmWave Idle Scanning Policy

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_MM_WAVE_IDLE_DEFAULT_INTERVAL_S` | `15.0` | Idle interval base (seconds) |
| `HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S` | `40.0` | Seconds of idle before triggering probe |
| `HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S` | `5.0` | Duration of idle probe (seconds) |
| `HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP` | `3` | Consecutive misses before physical sweep |
| `HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S` | `150.0` | Cooldown between sweeps (seconds) |
| `HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S` | `45.0` | Quiet window after confirmed target (seconds) |
| `HEALTHY_MM_WAVE_ERRORS_BEFORE_SUPPRESSION` | `3` | Consecutive sensor errors before suppressing probes |
| `HEALTHY_MM_WAVE_ERROR_BACKOFF_S` | `120.0` | Seconds to wait before retry after error suppression |
| `HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER` | `2.0` | Probe interval multiplier when multi-target is active |

### Light Context Policy

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_LIGHT_CONTEXT_ENABLED` | `true` | Enable/disable light context tool entirely |
| `HEALTHY_AUTO_LIGHT_CONTEXT_ENABLED` | `true` | Auto-invoke light context after mmWave |
| `HEALTHY_LIGHT_ANALYTICS_ENABLED` | `true` | Enable JSONL analytics logging |
| `HEALTHY_LIGHT_CONTEXT_USER_ID` | `default` | User ID for per-user baseline tracking |
| `HEALTHY_LIGHT_PREFERS_DIM` | `false` | User preference: prefers dim environments |
| `HEALTHY_LIGHT_SENSITIVE` | `false` | User preference: light sensitive |
| `HEALTHY_LIGHT_ALLOW_WELLNESS_NUDGES` | `true` | Allow low-light strain-risk nudges |
| `HEALTHY_LIGHT_DAY_START_HOUR` | `7` | Hour when daytime starts (0–23) |
| `HEALTHY_LIGHT_NIGHT_START_HOUR` | `20` | Hour when nighttime starts (0–23) |
| `HEALTHY_LIGHT_LOW_LUX_THRESHOLD` | `40.0` | Lux below this is "low" |
| `HEALTHY_LIGHT_BRIGHT_LUX_THRESHOLD` | `250` | Lux above this is "bright" |
| `HEALTHY_LIGHT_SHARP_DROP_LUX` | `80` | Lux drop in 60s to trigger unexpected_darkening |
| `HEALTHY_LIGHT_PROLONGED_MIN` | `45` | Minutes of low light before strain-risk nudge |
| `HEALTHY_LIGHT_BASELINE_ALPHA` | `0.15` | EMA smoothing factor for lux baseline |
| `HEALTHY_LIGHT_BASELINE_MIN_SAMPLES` | `5` | Minimum samples before baseline is used |
| `HEALTHY_LIGHT_BASELINE_MAX_AGE_DAYS` | `90` | Days before stale user baselines are pruned |
| `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` | `90` | Days before old analytics rows are pruned |

### External Profiles/Tools

| Variable | Default | Description |
|---|---|---|
| `REACHY_MINI_EXTERNAL_PROFILES_DIRECTORY` | *(none)* | External profiles root directory |
| `REACHY_MINI_EXTERNAL_TOOLS_DIRECTORY` | *(none)* | External tools root directory |
| `AUTOLOAD_EXTERNAL_TOOLS` | `false` | Auto-discover `.py` tools in external tools directory |

### Vision / HuggingFace

| Variable | Default | Description |
|---|---|---|
| `LOCAL_VISION_MODEL` | `HuggingFaceTB/SmolVLM2-2.2B-Instruct` | Local vision model (with `--local-vision`) |
| `HF_HOME` | `./cache` | HuggingFace cache directory |
| `HF_TOKEN` | *(none)* | HuggingFace API token (falls back to `hf auth login`) |

## Testing Notes

- `conftest.py` sets `REACHY_MINI_SKIP_DOTENV=1` and clears profile env vars for isolation
- Tests do not require a connected robot or OpenAI key
- The tool registry uses lazy initialization — it runs on first call to `get_tool_specs()` or `dispatch_tool_call()`, not at import time
- Test coverage is comprehensive across all handler classes (IdlePolicy, LightOrchestrator, ToolDispatcher, TranscriptHandler, AudioRouter) and `openai_realtime.py` (318 tests total; includes multi-person tracking logic, protocol version handshake, bio rate boundary conditions at firmware guard rails, LightOrchestrator file I/O error handling, device_context integration, EVT_DIAG diagnostics decode and integration, and full openai_realtime coverage)
- `pytest-asyncio` is used for async test support
- mypy covers both `src/` and `tests/`

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
