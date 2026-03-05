# Gradio Sensor Dashboard Design

## Goal

Replace the Gradio personality editor with a real-time sensor dashboard showing live vitals, a radar room view, and vitals history graphs. The chatbot becomes secondary (collapsed accordion). WebSocket push for instant updates.

## Audience

End-user facing: wellness researchers, caregivers, demo audiences. Must be clean, intuitive, and visually compelling — not a developer debug panel.

## Layout

```
┌─────────────────────────────────────────────────────────┐
│  Healthy Heartrate Breathing        [API Key] [Status]  │
├──────────────────────────┬──────────────────────────────┤
│                          │                              │
│     LIVE VITALS          │       RADAR VIEW             │
│                          │                              │
│  Heart Rate:  72 bpm     │         ·  0.8m              │
│  Breathing:   16 bpm     │     ·       ·                │
│  State: RESTING_VITALS   │  (sensor)      ·             │
│  Targets: 1              │     ·       ·                │
│  Proximity: 42 lux       │         ·                    │
│                          │    rings: 0.5m 1.0m 1.5m    │
│                          │                              │
├──────────────────────────┴──────────────────────────────┤
│                                                         │
│   VITALS HISTORY (rolling 2-4 hours)                    │
│   ───── Heart Rate  ───── Breathing Rate                │
│   ▁▂▃▅▆▇▇▆▅▃▂▁▂▃▅▆▇▇▆▅▃▂▁▂▃                          │
│                                                         │
├─────────────────────────────────────────────────────────┤
│  ▾ Conversation Log (collapsed by default)              │
│    [chatbot transcript when expanded]                   │
└─────────────────────────────────────────────────────────┘
```

- **Top left (~50%)**: Live vitals card with large numbers and status indicators
- **Top right (~50%)**: Radar plot canvas showing detected people in the room
- **Middle (full-width)**: Vitals history line chart (HR + BR over time)
- **Bottom (full-width)**: Collapsible conversation log (Gradio `gr.Accordion`, closed by default)
- **Header**: App name, API key input (kept from old personality editor), WebSocket connection status

## Architecture

### Data Flow

```
mmWave tool result
  → extract_sensor_state() (existing)
  → handler.sensor_state (existing dict)
  → NEW: append to vitals_history SQLite table
  → NEW: push to WebSocket clients via /ws/sensor

Gradio page loads
  → JS connects to /ws/sensor
  → Receives live sensor state on each change
  → Updates vitals card, radar canvas, history graph
  → History graph fetches full dataset from /api/vitals/history on load + periodically
```

### WebSocket Endpoint (`/ws/sensor`)

Mounted on the Gradio ASGI app (Gradio 5 exposes the underlying Starlette app).

- On `sensor_state` change: push JSON to all connected clients
- Payload: `{ "live": {...sensor_state...}, "targets": [...recent_targets...] }`
- `targets` array: `[{cluster, x_mm, y_mm, r_mm, bearing_cdeg, v_cms_x10}, ...]`
- Heartbeat: server sends ping every 30s to keep connection alive
- Client auto-reconnects on disconnect (exponential backoff)

### Vitals History Storage

New SQLite table in the existing `light_context_analytics.db`:

```sql
CREATE TABLE vitals_history (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp       TEXT NOT NULL,
    heart_rate_bpm  REAL,
    breath_rate_bpm REAL,
    device_state    TEXT,
    target_count    INTEGER,
    lux             REAL
);
CREATE INDEX idx_vitals_history_timestamp ON vitals_history(timestamp);
```

- Appended each time `extract_sensor_state()` runs with valid data (not on errors)
- Auto-pruned: rows older than `HEALTHY_VITALS_HISTORY_MAX_HOURS` (default 4, configurable via env var)
- Same WAL mode and lazy init pattern as existing light_context analytics
- Schema versioned via `PRAGMA user_version` (increment from current version)

### REST Endpoint (`/api/vitals/history`)

- `GET /api/vitals/history?hours=4`
- Returns: `[{timestamp, heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux}, ...]`
- Fetched by JS on page load and every 60s to catch up on any missed WS data

## Components

### 1. Live Vitals Card

Rendered as `gr.HTML` component updated via WebSocket JS.

- Heart rate: large font, red accent, with "high/moderate/low/unavailable" reliability badge
- Breathing rate: large font, blue accent
- Device state: human-friendly label (e.g., "Resting (vitals active)" instead of "RESTING_VITALS")
- Target count: with icon
- Lux: with proximity interpretation (e.g., "Close presence" / "Clear path")
- Connection status: green dot (connected), orange (stale >30s), red (disconnected/error)
- Smooth CSS transitions between value updates

Human-friendly state labels:
| Firmware State | Display Label |
|---|---|
| NO_TARGET | No one detected |
| MULTI_TARGET | Multiple people |
| PRESENT_FAR | Someone nearby (far) |
| MOVING | Person moving |
| STILL_NEAR | Person still (close) |
| RESTING_VITALS | Resting — vitals active |

### 2. Radar Plot (Canvas)

Custom `gr.HTML` component with embedded JS Canvas.

- Dark background (#0a0f1e) matching app theme
- Concentric range rings at 0.5m, 1.0m, 1.5m (subtle dashed lines)
- Sensor/robot position at center-bottom of the canvas
- Forward-facing: 180-degree arc above the sensor
- Each detected person as a dot:
  - Green: RESTING_VITALS / STILL_NEAR (vitals-eligible)
  - Cyan: MOVING / PRESENT_FAR
  - Gray: other states
  - Focused target (closest, being measured): bright highlight ring + pulse animation
  - Dot size: fixed (not proportional to signal — simpler, cleaner)
- Smooth position interpolation between updates (lerp over 500ms)
- Fade in/out animation for appearing/disappearing targets
- Range label on each ring
- Bearing line from sensor to focused target (subtle)

Canvas size: ~300x300px, responsive.

### 3. Vitals History Graph

Embedded JS chart (lightweight library like Chart.js or uPlot) via `gr.HTML`.

- Two lines: HR (red/coral) and BR (blue/teal)
- X-axis: time, rolling window (default 2 hours visible, scrollable to 4)
- Y-axes: HR scale on left (40-140 bpm), BR scale on right (4-30 bpm)
- Shaded "normal range" bands: HR 60-100 bpm (light red), BR 12-20 bpm (light blue)
- Data gaps shown as line breaks (no interpolation across >60s gaps)
- Updated in real-time as new data arrives via WebSocket
- Full history loaded from `/api/vitals/history` on page load
- Tooltip on hover showing exact values and timestamp
- Responsive width

### 4. Conversation Log (Accordion)

`gr.Accordion` wrapping `gr.Chatbot`, collapsed by default.

- Preserves existing chatbot functionality for simulator users
- Tool results still appear as messages
- No changes to audio streaming behavior

## Gradio App Changes

### Removed
- `PersonalityUI` class and all personality components (dropdown, preview, editor, save/create)
- `gradio_personality.py` import/usage in `main.py`
- Voice selector, tools editor, profile management
- All personality-related event handlers

### Kept
- API key textbox (moved to header area)
- Chatbot (moved into accordion)
- Audio streaming (unchanged)

### Added
- `gr.HTML` for live vitals card (updated via embedded JS + WebSocket)
- `gr.HTML` for radar canvas (updated via embedded JS + WebSocket)
- `gr.HTML` for vitals history chart (updated via embedded JS + WebSocket)
- `gr.Accordion` wrapping chatbot
- WebSocket mount on Gradio app
- Vitals history SQLite table + REST endpoint

## New Files

| File | Purpose |
|---|---|
| `src/.../sensor_dashboard.py` | Gradio dashboard builder: creates components, mounts WebSocket, history endpoint |
| `src/.../vitals_store.py` | Vitals history SQLite storage: append, query, prune |
| `src/.../static/dashboard.js` | Client-side JS: WebSocket connection, radar canvas, chart, vitals card updates |
| `src/.../static/dashboard.css` | Dashboard-specific styles |

## Modified Files

| File | Changes |
|---|---|
| `main.py` | Replace `PersonalityUI` setup with `SensorDashboard` setup; mount WS/REST |
| `tool_dispatcher.py` | After `extract_sensor_state()`, also append to vitals store + notify WS clients |
| `openai_realtime.py` | Add WS broadcast callback; pass to tool dispatcher |

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_VITALS_HISTORY_MAX_HOURS` | `4` | Rolling window for vitals history retention |

## Error Handling

- WebSocket disconnect: client auto-reconnects with exponential backoff (1s, 2s, 4s, max 30s)
- Sensor disconnect: vitals card shows "Disconnected" state with red indicator; radar clears; graph stops updating but preserves history
- SQLite errors: logged, non-fatal; dashboard continues with live data only (no history)
- No targets: radar shows empty room with range rings; vitals show dashes

## Testing Strategy

- `test_vitals_store.py`: SQLite append, query, prune, schema migration
- `test_sensor_dashboard.py`: WebSocket message format, REST endpoint responses
- Existing tests unaffected (personality editor removal is UI-only)

## Not In Scope

- Multi-user support (single dashboard per app instance)
- Historical data export (CSV, etc.)
- Alert thresholds / notifications
- Mobile-optimized layout (desktop-first, basic responsive)
