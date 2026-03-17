# mmWave TUI Monitor — Design Spec

## Purpose

Replace the plain-text `mmwave_decode.py` output with a Textual-based terminal UI as the default experience. The TUI provides a real-time sensor dashboard for development and demos, while preserving `--format pretty` and `--format json` for scripting.

## Use Cases

- **Firmware development**: watch state transitions, bio gating, and target tracking in real time
- **Integration debugging**: see what the sensor is sending while working on the Python side
- **Demos**: visually appealing dashboard with large vitals numbers readable from across the room

Not a production monitor — optimized for clarity and quick comprehension.

## Architecture

### Module Split

```
src/healthy_heartrate_breathing/
  sensor_models.py       # NEW — shared data models + event processing (top-level for reuse)

src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/
  mmwave_protocol.py     # existing — wire format encode/decode (unchanged)
  mmWave.py              # existing tool (unchanged)

hardware/tools/
  mmwave_decode.py       # existing — plain-text/json CLI (--format tui added as default)
  mmwave_monitor.py      # NEW — Textual TUI app
```

`sensor_models.py` is placed at the package top level (`src/healthy_heartrate_breathing/`) rather than inside the locked profile directory, because it is shared infrastructure consumed by both `hardware/tools/mmwave_monitor.py` and the Gradio dashboard. The protocol codec stays in the profile directory because it is paired with the firmware.

### `sensor_models.py` — Shared Layer

This module is consumed by both the TUI and the existing/future Gradio dashboard. It contains:

**Data models** (dataclasses, field names match protocol dict keys from `decode_event()`):
- `DecodedEvent`: event_type (str), host_ts (float, `time.monotonic()` at decode time), seq (int), data (union of model types below). This is the core type flowing through the entire system — produced by `read_events()`, consumed by all widgets and processing classes. Firmware `t_ms` is available in `SensorSnapshot` for drift analysis.
- `SensorSnapshot`: t_ms, state, pose, human, n_targets, dist_cm, head_moving, dist_new
- `BioReading`: hr, br, allowed, valid, hr_new, br_new
- `LightReading`: lux, valid
- `TargetInfo`: cluster, x, y, r, bearing, velocity (mapped from `v` in protocol)
- `TargetsEvent`: n_targets, forced_focus, focus (TargetInfo | None), targets (list[TargetInfo]), targets_truncated
- `DiagCounters`: mmwave_fail_count, mmwave_consecutive_fails, tx_drop_count (field names match protocol wire format exactly)
- `ConnectionInfo`: port, baud, proto_version, feature_bits
- `FrameStats`: frames_per_sec, bytes_per_sec, bad_frame_count (single counter for all `ProtocolError` types — CRC mismatch, bad payload length, invalid COBS are not distinguished)
- `EventRates`: per-event-type rates (state/sec, bio/sec, targets/sec, etc.), computed as a sliding window count (constructor parameter `window_s: float = 10.0`)

**Event processing**:
- `EventBuffer`: ring buffer of recent events with configurable max size
- `NotableFilter`: determines if an event is "notable" (state transitions, new bio readings with changed values, errors) vs repetitive
- `BioAcceptanceTracker`: rolling percentage of bio events where allowed=1 AND valid=1
- `StateTransitionLog`: compact log of state changes with timestamps and durations
- `TimeSeriesBuffer`: fixed-size ring buffer for sparkline chart data (HR, BR over time), stores `(timestamp, value, device_state)` tuples for the state annotation bar

**Serial reader** (async iterator):
- `async def read_events(port, baud) -> AsyncIterator[DecodedEvent]`: wraps the existing frame decode loop from `mmwave_decode.py` into a reusable async generator that yields typed events. Uses `asyncio.to_thread()` for each serial read cycle (matching the pattern in `mmWave.py`), yielding decoded events back to the async caller. Both the TUI and future WebSocket bridge can consume this.

### `mmwave_monitor.py` — Textual TUI

The Textual app. Launched by `mmwave_decode.py --format tui` (default) or directly.

### `mmwave_decode.py` — CLI Changes

- `--format` choices become: `tui` (default), `pretty`, `json`
- `--format tui` imports and launches `mmwave_monitor.py`
- `--format pretty` / `--format json` behave exactly as today
- `--filter` in TUI mode: only affects the log panel, static panels always show latest data
- `--filter` in pretty/json mode: unchanged behavior
- **`--port` in TUI mode**: becomes optional. When omitted, auto-detects the serial port using the VID/PID strategy from `mmWave.py` (`0x303A:0x1001` → glob fallback → HELLO probe). If auto-detection fails, the TUI shows a "No sensor found" status with retry option. For `--format pretty`/`--format json`, `--port` remains required (no interactive fallback).
- **`--input-file` + `--format tui`**: plays back all frames from the file, then enters a frozen state showing final values. A "Replay complete" indicator appears in the header. Press `q` to quit.
- **`--show-bad-frames` in TUI mode**: always effectively enabled — bad frame errors appear in the Diagnostics tab protocol log and increment `FrameStats.bad_frame_count`. The flag is accepted but ignored (no stderr output in TUI mode).
- **Breaking change note**: the default `--format` changes from `pretty` to `tui`. Existing scripts piping output should explicitly pass `--format pretty` or `--format json`.

## Layout

### Terminal Requirements

Minimum terminal size: **80 columns x 24 rows**. Below 80 columns, the radar panel is hidden and the layout goes single-column. Textual's responsive CSS handles the reflow.

### Main Tab

```
┌─────────────────────────────────────────────────────────────────┐
│ mmWave Monitor — /dev/cu.usbmodem2101 @ 115200  [Main] [Diag]  │
├──────────────────────────┬──────────────────────────────────────┤
│ STATE              LUX ▊ 21.7 │ RADAR                  3 targets │
│                                │                                  │
│  ╭──╮                          │      ·  4.8m 30°                │
│  │  │  STILL_NEAR              │                                  │
│  ├──┤  Seated · Vitals ready   │         · 2.5m -25°             │
│  │  │                          │    ╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌             │
│  ╰┬─┴╮                        │    :  vitals zone  :             │
│   │   │ h=1 tgt=1 d=40.2cm    │    :   ◉ 0.44m 10° :            │
│   ╵   ╵ hm=0       new=1      │    ╌╌╌╌╌╌╌▲╌╌╌╌╌╌╌╌             │
│                                │    ─── 1m ─── 3m ─── 6m         │
├────────────────────────────────┴─────────────────────────────────┤
│ VITALS                                                  ok/ok    │
│ ┌─ HEART RATE ──────────── 83 bpm ┐ ┌─ BREATHING ────────── 6 bpm ┐
│ │  100┊                           │ │   20┊                        │
│ │   80┊    ╱╲  ╱╲    ╱╲          │ │   12┊                        │
│ │   60┊╲╱╱  ╲╱  ╲╱╲╱  ╲╱╲      │ │    4┊ ─────╲╱──╲╱──────     │
│ │  ▇▇▇▇▇▇▓▓▓▇▇▇▇▇▇▇▒▒▇▇▇▇▇▇   │ │  ▇▇▇▇▇▇▓▓▓▇▇▇▇▇▇▇▒▒▇▇▇▇▇▇  │
│ │  5m ago                   now   │ │  5m ago                  now  │
│ └─────────────────────────────────┘ └──────────────────────────────┘
├──────────────────────────────────────────────────────────────────┤
│ LOG (3 lines — l to expand)                  notable · [f] firehose │
│ 12:04:33 state STILL_NEAR → MOVING  h=1 n=1 d=52.1cm              │
│ 12:04:32 bio   ok/ok br=6.0 hr=83.0 +br +hr                      │
│ 12:04:32 light lux=21.7                                            │
├──────────────────────────────────────────────────────────────────┤
│ q quit · l toggle log · f firehose/notable · d diagnostics · ? help│
└──────────────────────────────────────────────────────────────────┘
```

**State annotation bar** (colored strip under each chart): rendered as a single-row `Static` widget of colored block characters (`\u2588`), one character per time bucket, aligned with the chart's X-axis range. Uses Rich markup for coloring. Color mapping:
- Green: STILL_NEAR
- Yellow: MOVING, PRESENT_FAR
- Blue: RESTING_VITALS
- Gray: NO_TARGET
- Purple: MULTI_TARGET

### State Icon System

Each device state gets a distinct visual representation in the State panel. Icons are rendered as pre-drawn multi-line Rich `Text` blocks (approximately 8 characters wide x 5 rows tall) that swap per state. Pose (SITTING/STANDING/UNKNOWN) changes the figure's posture within the same state.

| State | Visual | Color | Description |
|-------|--------|-------|-------------|
| `NO_TARGET` | Dashed person outline | Gray (#555) | No one detected |
| `PRESENT_FAR` | Small standing figure | Yellow (#fbbf24) | Person beyond vitals range |
| `MOVING` | Walking pose + motion lines | Yellow (#fbbf24) | Person in motion |
| `STILL_NEAR` | Seated/standing figure | Green (#4ade80) | Still, in vitals range |
| `RESTING_VITALS` | Seated + heart + pulse wave | Blue (#60a5fa) | Vitals actively measuring |
| `MULTI_TARGET` | Two figures + warning | Purple (#a78bfa) | Multiple people detected |

The exact character art will be defined during implementation. Each icon is a Rich `Text` renderable stored in a lookup dict keyed by `(state, pose)`. Fallback: if terminal doesn't support Unicode well, degrade to a single large emoji + text label.

### Radar Panel

- **Range**: 0–6m with rings at 1m intervals (detection range)
- **Vitals zone**: dashed pink arc at 0.35–1.5m
- **Focus target**: bright purple dot with glow ring, labeled with range + bearing
- **Other targets**: dimmed dots, colored by distance (green=near, yellow=mid, orange=far)
- **Sensor**: small icon at bottom center
- **Rendering**: braille characters (Unicode `U+2800`–`U+28FF`) in a custom widget extending `Static`. Each braille cell encodes a 2x4 dot grid, giving sub-character resolution for plotting targets and arcs. No third-party canvas dependency.

### Diagnostics Tab

Accessible via `d` key or `[Diag]` tab. Contains:

**Protocol event log** (scrolling):
- ack, err, pong, hello events with timestamps
- Color-coded: green for ack/pong, red for err

**Counters panel** (updated in place):
- mmWave fail count, consecutive fails, TX drops (from `EVT_DIAG`)
- Bad frame count (computed locally, all `ProtocolError` types combined)

**Connection info** (static):
- Port, baud rate, protocol version, feature bits, uptime

**Computed stats** (updated in place):
- Frames/sec, bytes/sec
- Event rate breakdown: state/sec, bio/sec, targets/sec, light/sec
- Bio acceptance rate: % of bio events where allowed=1 AND valid=1

## Keybindings

| Key | Action |
|-----|--------|
| `q` | Quit |
| `l` | Toggle log panel (collapsed 3 lines ↔ expanded) |
| `f` | Toggle log mode: notable-only ↔ firehose |
| `d` | Switch to diagnostics tab |
| `m` | Switch to main tab |
| `Tab` | Cycle between tabs |
| `?` | Show help overlay |

## Log Behavior

**Notable mode** (default): shows only:
- State transitions (state changed from previous)
- Bio readings with new values (br_new=1 or hr_new=1)
- Light readings with significant change (>5 lux delta)
- Error events

**Firehose mode**: shows every decoded event (like current `--format pretty`)

**`--filter` interaction**: in TUI mode, `--filter bio,state` restricts which event types appear in the log. Static panels always show latest values for all event types regardless of filter.

## Dependencies

Added to the `dev` dependency group (since this is a development/debugging tool):
- `textual` — TUI framework
- `textual-plotext` — multi-row terminal charts with Y-axis labels (the built-in Textual `Sparkline` widget only renders a single row of block characters, which is insufficient for the depicted vitals charts with axis labels and gradient fills)

## File Structure

```
src/healthy_heartrate_breathing/
  sensor_models.py                # NEW: shared data models + event processing

src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/
  mmwave_protocol.py              # UNCHANGED: wire format encode/decode
  mmWave.py                       # UNCHANGED: mmWave tool

hardware/tools/
  mmwave_monitor.py               # NEW: Textual TUI app
  mmwave_decode.py                # MODIFIED: --format tui as default
```

## Reuse Path

The shared `sensor_models.py` layer is designed for reuse:

- **TUI** (`mmwave_monitor.py`): consumes `read_events()` async iterator, populates model instances, renders via Textual widgets
- **Gradio dashboard** (`dashboard.js` + `sensor_ws.py`): can import the same data models and event processing logic, push via WebSocket to the existing dashboard JS
- **Both frontends** share: data models, notable filter logic, time series buffers, bio acceptance tracking, state transition detection

This means visual parity between TUI and web dashboard comes naturally — same data, same processing, different renderers.
