# Gradio Sensor Dashboard Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the Gradio personality editor with a real-time sensor dashboard showing live vitals, radar room view, and vitals history graph — pushed via WebSocket.

**Architecture:** Mount a WebSocket endpoint (`/ws/sensor`) on the Gradio ASGI app. When `sensor_state` changes in `tool_dispatcher.py`, it broadcasts to all connected WS clients. A new `vitals_store.py` persists HR/BR history to SQLite. The Gradio UI swaps out the personality editor for three `gr.HTML` components (vitals card, radar canvas, history chart) driven by embedded JS connecting to the WebSocket. The chatbot is tucked into a collapsed `gr.Accordion`.

**Tech Stack:** Python 3.12, Gradio 5, Starlette WebSocket, SQLite (WAL mode), HTML5 Canvas, Chart.js (CDN)

**Design doc:** `docs/plans/2026-03-05-gradio-sensor-dashboard-design.md`

---

### Task 1: Vitals History Store (`vitals_store.py`)

**Files:**
- Create: `src/healthy_heartrate_breathing/vitals_store.py`
- Test: `tests/test_vitals_store.py`

**Step 1: Write the failing tests**

Create `tests/test_vitals_store.py`:

```python
"""Tests for VitalsStore SQLite persistence."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import time
from pathlib import Path

import pytest

from healthy_heartrate_breathing.vitals_store import VitalsStore


@pytest.fixture
def store(tmp_path: Path) -> VitalsStore:
    return VitalsStore(db_path=tmp_path / "test_vitals.db", max_hours=4)


class TestAppendAndQuery:
    def test_append_and_query_returns_rows(self, store: VitalsStore) -> None:
        store.append(heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0)
        rows = store.query(hours=1)
        assert len(rows) == 1
        assert rows[0]["heart_rate_bpm"] == 72.0
        assert rows[0]["breath_rate_bpm"] == 16.0
        assert rows[0]["device_state"] == "RESTING_VITALS"

    def test_append_with_nones(self, store: VitalsStore) -> None:
        store.append(heart_rate_bpm=None, breath_rate_bpm=None, device_state="NO_TARGET", target_count=0, lux=None)
        rows = store.query(hours=1)
        assert len(rows) == 1
        assert rows[0]["heart_rate_bpm"] is None

    def test_query_returns_empty_before_any_append(self, store: VitalsStore) -> None:
        rows = store.query(hours=1)
        assert rows == []

    def test_query_respects_hours_window(self, store: VitalsStore) -> None:
        """Rows outside the window are not returned."""
        store.append(heart_rate_bpm=60.0, breath_rate_bpm=12.0, device_state="RESTING_VITALS", target_count=1, lux=50.0)
        # Query with 0 hours should return nothing (row is >0h old)
        rows = store.query(hours=0)
        assert rows == []


class TestPruning:
    def test_prune_removes_old_rows(self, tmp_path: Path) -> None:
        store = VitalsStore(db_path=tmp_path / "prune.db", max_hours=0)
        store.append(heart_rate_bpm=70.0, breath_rate_bpm=15.0, device_state="RESTING_VITALS", target_count=1, lux=40.0)
        store.prune()
        rows = store.query(hours=999)
        assert rows == []


class TestSchemaInit:
    def test_creates_db_file(self, tmp_path: Path) -> None:
        db_path = tmp_path / "new.db"
        assert not db_path.exists()
        store = VitalsStore(db_path=db_path, max_hours=4)
        store.append(heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0)
        assert db_path.exists()

    def test_idempotent_init(self, tmp_path: Path) -> None:
        db_path = tmp_path / "idem.db"
        s1 = VitalsStore(db_path=db_path, max_hours=4)
        s1.append(heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0)
        s2 = VitalsStore(db_path=db_path, max_hours=4)
        rows = s2.query(hours=1)
        assert len(rows) == 1
```

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_vitals_store.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'healthy_heartrate_breathing.vitals_store'`

**Step 3: Write minimal implementation**

Create `src/healthy_heartrate_breathing/vitals_store.py`:

```python
"""Vitals history SQLite store for dashboard graphs."""

from __future__ import annotations
import sqlite3
import logging
from typing import Any
from pathlib import Path
from datetime import datetime, timezone, timedelta


logger = logging.getLogger(__name__)

_SCHEMA_VERSION = 1


class VitalsStore:
    """Append-only vitals history with rolling time window pruning."""

    def __init__(self, *, db_path: Path, max_hours: int = 4) -> None:
        self._db_path = db_path
        self._max_hours = max_hours
        self._initialized = False

    def _ensure_schema(self) -> None:
        if self._initialized:
            return
        try:
            self._db_path.parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(self._db_path)
            conn.execute("PRAGMA journal_mode=WAL")
            (version,) = conn.execute("PRAGMA user_version").fetchone()
            if version != _SCHEMA_VERSION:
                conn.execute("DROP TABLE IF EXISTS vitals_history")
                conn.execute("DROP INDEX IF EXISTS idx_vitals_history_timestamp")
            conn.executescript(f"""
                CREATE TABLE IF NOT EXISTS vitals_history (
                    id              INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp       TEXT    NOT NULL,
                    heart_rate_bpm  REAL,
                    breath_rate_bpm REAL,
                    device_state    TEXT,
                    target_count    INTEGER,
                    lux             REAL
                );
                CREATE INDEX IF NOT EXISTS idx_vitals_history_timestamp
                    ON vitals_history(timestamp);
                PRAGMA user_version={_SCHEMA_VERSION};
            """)
            conn.close()
            self._initialized = True
        except Exception:
            logger.warning("Failed to init vitals DB at %s", self._db_path, exc_info=True)

    def append(
        self,
        *,
        heart_rate_bpm: float | None,
        breath_rate_bpm: float | None,
        device_state: str | None,
        target_count: int | None,
        lux: float | None,
    ) -> None:
        """Insert one vitals reading."""
        self._ensure_schema()
        try:
            conn = sqlite3.connect(self._db_path)
            conn.execute(
                "INSERT INTO vitals_history (timestamp, heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                (datetime.now(timezone.utc).isoformat(), heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux),
            )
            conn.commit()
            conn.close()
        except Exception:
            logger.warning("Failed to append vitals", exc_info=True)

    def query(self, *, hours: int = 4) -> list[dict[str, Any]]:
        """Return vitals rows from the last N hours."""
        self._ensure_schema()
        cutoff = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat()
        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                "SELECT timestamp, heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux "
                "FROM vitals_history WHERE timestamp >= ? ORDER BY timestamp ASC",
                (cutoff,),
            ).fetchall()
            conn.close()
            return [dict(r) for r in rows]
        except Exception:
            logger.warning("Failed to query vitals", exc_info=True)
            return []

    def prune(self) -> None:
        """Delete rows older than max_hours."""
        self._ensure_schema()
        cutoff = (datetime.now(timezone.utc) - timedelta(hours=self._max_hours)).isoformat()
        try:
            conn = sqlite3.connect(self._db_path)
            conn.execute("DELETE FROM vitals_history WHERE timestamp < ?", (cutoff,))
            conn.commit()
            conn.close()
        except Exception:
            logger.warning("Failed to prune vitals", exc_info=True)
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_vitals_store.py -v`
Expected: All PASS

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/vitals_store.py tests/test_vitals_store.py
git commit -m "feat: add VitalsStore for dashboard vitals history persistence"
```

---

### Task 2: Wire Vitals Store into Tool Dispatcher

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py` (add vitals_store init + pass to dispatcher)
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py` (append to vitals_store after sensor update)
- Test: `tests/test_tool_dispatcher.py` (add test for vitals append)

**Step 1: Write the failing test**

Add to `tests/test_tool_dispatcher.py`:

```python
class TestVitalsStoreIntegration:
    @pytest.mark.asyncio
    async def test_mmwave_result_appended_to_vitals_store(self, dispatcher_factory):
        """When mmWave returns valid data, it should be appended to vitals_store."""
        appended = []

        def mock_append(**kwargs):
            appended.append(kwargs)

        d = dispatcher_factory(on_sensor_update=lambda s: None)
        d._vitals_append = mock_append

        d.dispatch(tool_name="mmWave", args_json='{"mode":"scan"}', call_id="c1", is_idle=False)
        for _ in range(50):
            await asyncio.sleep(0)

        assert len(appended) == 1
        assert appended[0]["device_state"] == "STILL_NEAR"
```

**Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_tool_dispatcher.py::TestVitalsStoreIntegration -v`
Expected: FAIL — `_vitals_append` not on ToolDispatcher

**Step 3: Modify ToolDispatcher**

In `tool_dispatcher.py`, add `vitals_append` parameter to `__init__`:

```python
def __init__(
    self,
    *,
    # ... existing params ...
    on_sensor_update: Optional[Callable[[dict[str, Any]], None]] = None,
    vitals_append: Optional[Callable[..., None]] = None,
) -> None:
    # ... existing assignments ...
    self._vitals_append = vitals_append
```

In `_run_tool()`, after the `extract_sensor_state()` / `_on_sensor_update` block (around line 426), add:

```python
if self._vitals_append is not None:
    try:
        self._vitals_append(
            heart_rate_bpm=new_sensor.get("heart_rate_bpm"),
            breath_rate_bpm=new_sensor.get("breath_rate_bpm"),
            device_state=new_sensor.get("device_state"),
            target_count=new_sensor.get("target_count"),
            lux=new_sensor.get("lux"),
        )
    except Exception:
        logger.debug("Vitals store append failed", exc_info=True)
```

In `openai_realtime.py`, in `__init__`, add vitals_store creation:

```python
from healthy_heartrate_breathing.vitals_store import VitalsStore
self.vitals_store = VitalsStore(
    db_path=self._resolve_runtime_data_path("vitals_history.db"),
    max_hours=env_int("HEALTHY_VITALS_HISTORY_MAX_HOURS", 4, min_value=1),
)
```

In `_run_realtime_session()`, pass to ToolDispatcher:

```python
dispatcher = ToolDispatcher(
    # ... existing params ...
    vitals_append=self.vitals_store.append,
)
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_tool_dispatcher.py::TestVitalsStoreIntegration -v`
Expected: PASS

Run: `uv run pytest tests/ --ignore=tests/vision -x`
Expected: All existing tests still pass

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py src/healthy_heartrate_breathing/openai_realtime.py tests/test_tool_dispatcher.py
git commit -m "feat: wire VitalsStore into tool dispatch pipeline"
```

---

### Task 3: WebSocket Broadcast Infrastructure

**Files:**
- Create: `src/healthy_heartrate_breathing/sensor_ws.py`
- Test: `tests/test_sensor_ws.py`

**Step 1: Write the failing tests**

Create `tests/test_sensor_ws.py`:

```python
"""Tests for WebSocket sensor broadcast."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
import asyncio

import pytest

from healthy_heartrate_breathing.sensor_ws import SensorBroadcaster


@pytest.fixture
def broadcaster() -> SensorBroadcaster:
    return SensorBroadcaster()


class TestBroadcaster:
    @pytest.mark.asyncio
    async def test_no_clients_no_error(self, broadcaster: SensorBroadcaster) -> None:
        broadcaster.broadcast({"device_state": "NO_TARGET"})

    @pytest.mark.asyncio
    async def test_broadcast_delivers_to_client(self, broadcaster: SensorBroadcaster) -> None:
        queue: asyncio.Queue[str] = asyncio.Queue()

        class FakeWS:
            async def send_text(self, text: str) -> None:
                await queue.put(text)

        ws = FakeWS()
        broadcaster.connect(ws)
        broadcaster.broadcast({"device_state": "MOVING"})
        # Give the background tasks a chance to run
        for _ in range(20):
            await asyncio.sleep(0)
        msg = await asyncio.wait_for(queue.get(), timeout=1.0)
        data = json.loads(msg)
        assert data["device_state"] == "MOVING"

    @pytest.mark.asyncio
    async def test_disconnect_removes_client(self, broadcaster: SensorBroadcaster) -> None:
        class FakeWS:
            async def send_text(self, text: str) -> None:
                pass

        ws = FakeWS()
        broadcaster.connect(ws)
        assert len(broadcaster._clients) == 1
        broadcaster.disconnect(ws)
        assert len(broadcaster._clients) == 0

    @pytest.mark.asyncio
    async def test_dead_client_auto_removed(self, broadcaster: SensorBroadcaster) -> None:
        class DeadWS:
            async def send_text(self, text: str) -> None:
                raise ConnectionError("gone")

        ws = DeadWS()
        broadcaster.connect(ws)
        broadcaster.broadcast({"test": True})
        for _ in range(20):
            await asyncio.sleep(0)
        assert len(broadcaster._clients) == 0
```

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_sensor_ws.py -v`
Expected: FAIL — module not found

**Step 3: Write implementation**

Create `src/healthy_heartrate_breathing/sensor_ws.py`:

```python
"""WebSocket broadcast for live sensor data."""

from __future__ import annotations
import json
import asyncio
import logging
from typing import Any


logger = logging.getLogger(__name__)


class SensorBroadcaster:
    """Manages WebSocket clients and broadcasts sensor state updates."""

    def __init__(self) -> None:
        self._clients: set[Any] = set()

    def connect(self, ws: Any) -> None:
        """Register a WebSocket client."""
        self._clients.add(ws)
        logger.debug("WS client connected (%d total)", len(self._clients))

    def disconnect(self, ws: Any) -> None:
        """Remove a WebSocket client."""
        self._clients.discard(ws)
        logger.debug("WS client disconnected (%d total)", len(self._clients))

    def broadcast(self, data: dict[str, Any]) -> None:
        """Send data to all connected clients (fire-and-forget)."""
        if not self._clients:
            return
        text = json.dumps(data)
        dead: list[Any] = []
        for ws in list(self._clients):
            asyncio.ensure_future(self._safe_send(ws, text, dead))

    async def _safe_send(self, ws: Any, text: str, dead: list[Any]) -> None:
        try:
            await ws.send_text(text)
        except Exception:
            self._clients.discard(ws)
            logger.debug("Removed dead WS client")
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_sensor_ws.py -v`
Expected: All PASS

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/sensor_ws.py tests/test_sensor_ws.py
git commit -m "feat: add SensorBroadcaster for WebSocket push to dashboard"
```

---

### Task 4: Wire WebSocket into Handler + Mount on Gradio App

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py` (create broadcaster, wire to sensor update)
- Modify: `src/healthy_heartrate_breathing/main.py` (mount WS endpoint + history REST on Gradio app)

**Step 1: Modify openai_realtime.py**

In `__init__`, add:

```python
from healthy_heartrate_breathing.sensor_ws import SensorBroadcaster
self.sensor_broadcaster = SensorBroadcaster()
```

Modify `_replace_sensor_state()` to also broadcast:

```python
def _replace_sensor_state(self, state: dict[str, Any]) -> None:
    """Replace sensor_state with a fresh snapshot and broadcast to WS clients."""
    self.sensor_state.clear()
    self.sensor_state.update(state)
    self.sensor_broadcaster.broadcast(state)
```

**Step 2: Modify main.py — mount WS + REST endpoints**

After creating `handler` and `stream` in the `if args.gradio:` block, mount routes on the FastAPI app:

```python
from starlette.websockets import WebSocket, WebSocketDisconnect

@app.websocket("/ws/sensor")
async def ws_sensor(websocket: WebSocket) -> None:
    await websocket.accept()
    handler.sensor_broadcaster.connect(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep alive; ignore client messages
    except WebSocketDisconnect:
        handler.sensor_broadcaster.disconnect(websocket)

@app.get("/api/vitals/history")
async def vitals_history(hours: int = 4) -> dict:
    return {"rows": handler.vitals_store.query(hours=min(hours, 24))}
```

**Step 3: Run full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x`
Expected: All pass (no existing tests break)

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/openai_realtime.py src/healthy_heartrate_breathing/main.py
git commit -m "feat: mount WebSocket and vitals history endpoints on Gradio app"
```

---

### Task 5: Extract Targets for Radar

**Files:**
- Modify: `src/healthy_heartrate_breathing/tool_dispatcher.py` (extract `recent_targets` into sensor state)

**Step 1: Write the failing test**

Add to `tests/test_tool_dispatcher.py`:

```python
class TestExtractSensorStateTargets:
    def test_recent_targets_included_in_state(self) -> None:
        result = {
            "status": "ok",
            "mode": "scan",
            "scan": {
                "device_state": "MULTI_TARGET",
                "max_target_count": 2,
                "targets_truncated": False,
                "latest_target": {"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0},
                "recent_targets": [
                    {"cluster": 0, "x": 0.5, "y": 0.1, "r": 0.8, "bearing": 0.12, "v": 0.0},
                    {"cluster": 1, "x": -0.3, "y": 0.6, "r": 0.9, "bearing": -0.4, "v": 0.02},
                ],
            },
        }
        state = extract_sensor_state(result)
        assert "targets" in state
        assert len(state["targets"]) == 2
        assert state["targets"][0]["x"] == 0.5
```

**Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_tool_dispatcher.py::TestExtractSensorStateTargets -v`
Expected: FAIL — `"targets"` not in state

**Step 3: Add to `extract_sensor_state()`**

In `tool_dispatcher.py`, inside the `if isinstance(scan, dict):` block, add:

```python
recent = scan.get("recent_targets")
if isinstance(recent, list):
    state["targets"] = recent
```

**Step 4: Run tests**

Run: `uv run pytest tests/test_tool_dispatcher.py -v`
Expected: All PASS

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/tool_dispatcher.py tests/test_tool_dispatcher.py
git commit -m "feat: extract recent_targets into sensor state for radar display"
```

---

### Task 6: Dashboard JS — WebSocket Client, Vitals Card, Radar Canvas, History Chart

**Files:**
- Create: `src/healthy_heartrate_breathing/static/dashboard.js`
- Create: `src/healthy_heartrate_breathing/static/dashboard.css`

This is the client-side code. It's pure JS/CSS, not Python — testing is manual via the Gradio UI.

**Step 1: Create `dashboard.css`**

Style the vitals card, radar canvas, and chart matching the existing dark theme (`style.css` uses `--bg: #060b1a`, `--accent: #45c4ff`, `--accent-2: #5ef0c1`).

Key classes:
- `.vitals-card` — dark panel with large numbers
- `.vitals-value` — big font for HR/BR
- `.vitals-label` — muted label text
- `.radar-container` — canvas wrapper
- `.status-dot` — green/orange/red connection indicator
- `.state-badge` — human-friendly device state label

**Step 2: Create `dashboard.js`**

Core sections:

1. **WebSocket connection** with auto-reconnect (exponential backoff 1s→30s max):
```javascript
function connectWS() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${proto}//${location.host}/ws/sensor`);
  ws.onmessage = (e) => {
    const data = JSON.parse(e.data);
    updateVitals(data);
    updateRadar(data);
    appendToChart(data);
  };
  ws.onclose = () => setTimeout(connectWS, reconnectDelay());
}
```

2. **Vitals card updater** (`updateVitals(data)`):
- Set HR/BR values with units, show `--` when null
- Map device_state to human labels (same as headless: NO_TARGET → "No one detected", etc.)
- Connection status dot: green (fresh), orange (>30s stale), red (error/disconnect)
- Lux → proximity interpretation

3. **Radar canvas** (`updateRadar(data)`):
- 300x300 Canvas, dark background
- Sensor at center-bottom
- Range rings at 0.5m, 1.0m, 1.5m (subtle dashed circles)
- Each target from `data.targets` as a colored dot:
  - Position: convert (x_mm, y_mm) → canvas pixels (scale to fit 1.5m range)
  - Color: green for RESTING_VITALS/STILL_NEAR, cyan for MOVING, gray for others
  - Focused target gets a pulsing ring animation
- Smooth lerp between positions over 500ms using `requestAnimationFrame`

4. **History chart** (Chart.js via CDN):
- Loaded from `https://cdn.jsdelivr.net/npm/chart.js`
- Two datasets: HR (coral red) and BR (teal blue)
- Time x-axis, dual y-axes
- Updated incrementally when WS data includes HR/BR
- Full data loaded from `/api/vitals/history?hours=4` on page load

**Step 3: Commit**

```bash
git add src/healthy_heartrate_breathing/static/dashboard.js src/healthy_heartrate_breathing/static/dashboard.css
git commit -m "feat: add dashboard client JS (WebSocket, vitals, radar canvas, chart)"
```

---

### Task 7: Gradio Layout — Replace Personality Editor with Dashboard

**Files:**
- Modify: `src/healthy_heartrate_breathing/main.py` (new Gradio layout)
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py` (remove personality editor dependency in `start_up`)

**Step 1: Rewrite the Gradio block in main.py**

Replace the `if args.gradio:` block. Remove `PersonalityUI` import and usage. Build new layout:

```python
if args.gradio:
    api_key_textbox = gr.Textbox(
        label="OpenAI API Key",
        type="password",
        value=os.getenv("OPENAI_API_KEY") if not get_space() else "",
    )

    # Dashboard HTML components (updated by embedded JS via WebSocket)
    vitals_html = gr.HTML(elem_id="vitals-panel")
    radar_html = gr.HTML(elem_id="radar-panel")
    chart_html = gr.HTML(elem_id="chart-panel")

    stream = Stream(
        handler=handler,
        mode="send-receive",
        modality="audio",
        additional_inputs=[chatbot, api_key_textbox],
        additional_outputs=[chatbot],
        additional_outputs_handler=update_chatbot,
        ui_args={"title": "Healthy Heartrate Breathing"},
    )

    # Build custom layout
    with stream.ui:
        # Inject CSS and JS
        gr.HTML(f"""
            <link rel="stylesheet" href="/file={static_dir}/dashboard.css">
            <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
            <script src="/file={static_dir}/dashboard.js" defer></script>
        """)

        with gr.Row():
            vitals_html.render()
            radar_html.render()
        chart_html.render()
        with gr.Accordion("Conversation Log", open=False):
            chatbot.render()
        api_key_textbox.render()

    stream_manager = stream.ui
    # ... mount WS + REST (from Task 4) ...
```

The exact Gradio layout API may need adjustment during implementation — Gradio 5's `Stream.ui` context manager and component rendering may require adapting. The key pattern is: render HTML components that the embedded JS finds by `elem_id` and updates.

**Step 2: Create initial HTML content for vitals/radar/chart panels**

The `gr.HTML` components need initial content with the DOM structure that `dashboard.js` will target:

- Vitals panel: div with `#vitals-card`, child elements for HR, BR, state, targets, lux, status dot
- Radar panel: `<canvas id="radar-canvas" width="300" height="300"></canvas>`
- Chart panel: `<canvas id="vitals-chart"></canvas>`

**Step 3: Static file serving**

Ensure the `static/` directory files are accessible. In the FastAPI app mount:

```python
from starlette.staticfiles import StaticFiles
static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")
```

**Step 4: Test manually**

Run: `uv run healthy-heartrate-breathing --gradio --debug`

Verify:
- Page loads with vitals card, radar, and chart panels
- WebSocket connects (check browser console)
- Chatbot is in collapsed accordion at bottom
- API key input is present

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/main.py
git commit -m "feat: replace personality editor with sensor dashboard layout in Gradio"
```

---

### Task 8: Remove Unused Personality Editor Code

**Files:**
- Modify: `src/healthy_heartrate_breathing/main.py` (remove import of PersonalityUI if still there)
- Keep: `src/healthy_heartrate_breathing/gradio_personality.py` (keep file — headless mode may still need it, and it's referenced from headless_personality)
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py` (remove personality args from `start_up` if needed)

**Step 1: Clean up imports and dead code**

Remove any remaining `PersonalityUI` imports from `main.py`. The `additional_inputs` for `Stream` should only include `chatbot` and `api_key_textbox` (no personality components).

Verify `openai_realtime.py:start_up()` still works: it accesses `self.latest_args` for the API key at index 3. With the new layout (only `chatbot` + `api_key_textbox` as additional_inputs), the API key will be at index 1 (after chatbot). Update the index accordingly.

**Step 2: Run full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -x`
Expected: All pass

**Step 3: Run lint**

Run: `uv run ruff check .`
Expected: Clean

**Step 4: Commit**

```bash
git add src/healthy_heartrate_breathing/main.py src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "refactor: remove personality editor from Gradio mode"
```

---

### Task 9: Integration Testing & Polish

**Step 1: Manual integration test**

Run the app in simulation mode:
```bash
reachy-mini-daemon --sim  # terminal 1
uv run healthy-heartrate-breathing --gradio --debug  # terminal 2
```

Test checklist:
- [ ] WebSocket connects (green status dot)
- [ ] Vitals card shows `--` initially (no data yet)
- [ ] After an mmWave tool call, vitals update instantly
- [ ] Radar shows dots for detected people
- [ ] History chart draws HR/BR lines
- [ ] Chart loads historical data on page refresh
- [ ] Accordion opens to show chat transcript
- [ ] API key input works
- [ ] Browser console: no JS errors
- [ ] WebSocket auto-reconnects after page reload

**Step 2: Fix any issues found during integration**

**Step 3: Run full test suite + lint**

Run: `uv run pytest tests/ --ignore=tests/vision -x && uv run ruff check .`
Expected: All pass, clean lint

**Step 4: Commit any fixes**

```bash
git commit -am "fix: integration polish for sensor dashboard"
```

---

### Task 10: Update Documentation

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`
- Modify: `docs/TODO.md`

**Step 1: Update CLAUDE.md**

- Add `sensor_ws.py` and `vitals_store.py` to the Architecture and Project Structure sections
- Add `SensorBroadcaster` to the handler table
- Add `HEALTHY_VITALS_HISTORY_MAX_HOURS` to env vars table
- Update Gradio mode description (sensor dashboard replaces personality editor)
- Update test count

**Step 2: Update README.md**

- Update UI Modes section: Gradio now shows sensor dashboard (vitals, radar, chart)
- Mention WebSocket for real-time updates

**Step 3: Update docs/TODO.md**

- Mark Gradio sensor dashboard as done
- Add any new technical debt items found during implementation

**Step 4: Commit**

```bash
git add CLAUDE.md README.md docs/TODO.md
git commit -m "docs: update documentation for Gradio sensor dashboard"
```
