# Analytics SQLite Migration — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace JSONL analytics in LightOrchestrator with SQLite for crash safety, in-app queryability, and disk efficiency.

**Architecture:** Single SQLite `.db` file with one fully-flattened `light_events` table. Time-based retention replaces file-size rotation. Lazy DB init on first write. Open/close per write with WAL journal mode.

**Tech Stack:** Python `sqlite3` stdlib, pytest for tests.

**Design doc:** `docs/plans/2026-03-01-analytics-sqlite-design.md`

---

### Task 1: Write failing tests for SQLite analytics

**Files:**
- Modify: `tests/test_light_orchestrator.py` — replace `TestAnalyticsRotation` class (lines 418-470)

**Step 1: Replace TestAnalyticsRotation with TestAnalyticsSqlite**

Replace the entire `TestAnalyticsRotation` class with:

```python
import sqlite3


class TestAnalyticsSqlite:
    """Verify analytics SQLite storage, retention, and error handling."""

    def test_event_inserted_and_queryable(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path)
        o._append_analytics_event(
            source_tool="test",
            lux=142.5,
            result={
                "context_state": "bright_active",
                "recommended_mode": "active",
                "recommended_actions": ["use_action_oriented_prompts", "normal_interruption_policy"],
                "confidence": 0.7,
                "cooldown_hint_s": 120,
                "reason_codes": ["bright_daytime"],
                "observations": {
                    "lux": 142.5,
                    "lux_delta_60s": 3.2,
                    "local_hour": 14,
                    "is_night": False,
                    "presence_detected": True,
                    "active_interaction": True,
                    "low_light_duration_min": 0.0,
                    "prefers_dim": False,
                    "light_sensitive": False,
                    "allow_wellness_nudges": True,
                },
            },
        )
        assert db_path.exists()
        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT context_state, lux, recommended_actions, reason_codes, obs_local_hour FROM light_events").fetchall()
        conn.close()
        assert len(rows) == 1
        assert rows[0][0] == "bright_active"
        assert rows[0][1] == 142.5
        assert rows[0][2] == "use_action_oriented_prompts,normal_interruption_policy"
        assert rows[0][3] == "bright_daytime"
        assert rows[0][4] == 14

    def test_retention_prunes_old_rows(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path, analytics_max_age_days=1)
        # Insert a recent event
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "bright"})
        # Manually insert an old event (200 days ago)
        old_ts = (datetime.now(timezone.utc) - timedelta(days=200)).isoformat()
        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO light_events (timestamp, user_id, source_tool, context_state) VALUES (?, ?, ?, ?)",
            (old_ts, "test-user", "test", "old"),
        )
        conn.commit()
        conn.close()
        # Trigger prune by writing another event (prune runs on init)
        o2 = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path, analytics_max_age_days=1)
        o2._append_analytics_event(source_tool="test", lux=200.0, result={"context_state": "dim"})
        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT context_state FROM light_events ORDER BY id").fetchall()
        conn.close()
        states = [r[0] for r in rows]
        assert "old" not in states
        assert "bright" in states
        assert "dim" in states

    def test_db_created_lazily_on_first_write(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path)
        assert not db_path.exists()
        o._append_analytics_event(source_tool="test", lux=50.0, result={})
        assert db_path.exists()

    def test_disabled_analytics_no_db(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=False, analytics_path=db_path)
        o._append_analytics_event(source_tool="test", lux=100.0, result={})
        assert not db_path.exists()

    def test_db_write_failure_does_not_raise(self, tmp_path: Path) -> None:
        # Point analytics_path to a directory (not a file) to force sqlite3 error
        bad_path = tmp_path / "not_a_file"
        bad_path.mkdir()
        db_path = bad_path / "sub" / "analytics.db"
        # Make parent unwritable after creation
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path)
        o._append_analytics_event(source_tool="test", lux=100.0, result={})
        assert db_path.exists()
        bad_path.chmod(0o555)
        try:
            # Write to a new path that can't be created
            o2 = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=bad_path / "locked" / "analytics.db")
            o2._append_analytics_event(source_tool="test", lux=200.0, result={})
            # Should not raise
        finally:
            bad_path.chmod(0o755)
```

Also add `import sqlite3` near the top of the test file (after the existing imports).

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_light_orchestrator.py::TestAnalyticsSqlite -v`
Expected: FAIL — `_append_analytics_event` still writes JSONL, not SQLite.

**Step 3: Commit test changes**

```
git add tests/test_light_orchestrator.py
git commit -m "test: add SQLite analytics tests (red — pending implementation)"
```

---

### Task 2: Implement SQLite analytics in LightOrchestrator

**Files:**
- Modify: `src/healthy_heartrate_breathing/light_orchestrator.py`

**Step 1: Update constructor**

In `__init__` (line 42):
- Replace `analytics_max_bytes: int = 5_000_000` with `analytics_max_age_days: int = 90`
- Replace `self.analytics_max_bytes = analytics_max_bytes` with `self.analytics_max_age_days = analytics_max_age_days`
- Add: `self._analytics_db_initialized: bool = False`
- Add: `self._analytics_insert_count: int = 0`
- Add `import sqlite3` to the module imports

**Step 2: Replace `_maybe_rotate_analytics` with `_init_analytics_db` and `_prune_analytics`**

Delete `_maybe_rotate_analytics()` (lines 249-262). Add:

```python
def _init_analytics_db(self) -> None:
    """Create analytics SQLite schema and enable WAL mode (idempotent)."""
    if self._analytics_db_initialized or self.analytics_path is None:
        return
    try:
        self.analytics_path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(self.analytics_path)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS light_events (
                id                         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp                  TEXT    NOT NULL,
                user_id                    TEXT    NOT NULL,
                source_tool                TEXT    NOT NULL,
                context_state              TEXT,
                recommended_mode           TEXT,
                recommended_actions        TEXT,
                confidence                 REAL,
                cooldown_hint_s            REAL,
                reason_codes               TEXT,
                lux                        REAL,
                obs_lux_delta_60s          REAL,
                obs_local_hour             INTEGER,
                obs_is_night               INTEGER,
                obs_presence_detected      INTEGER,
                obs_active_interaction     INTEGER,
                obs_low_light_duration_min REAL,
                obs_prefers_dim            INTEGER,
                obs_light_sensitive        INTEGER,
                obs_allow_wellness_nudges  INTEGER
            );
            CREATE INDEX IF NOT EXISTS idx_light_events_timestamp ON light_events(timestamp);
            CREATE INDEX IF NOT EXISTS idx_light_events_user_id ON light_events(user_id);
        """)
        conn.close()
        self._analytics_db_initialized = True
        self._prune_analytics()
    except Exception as e:
        logger.warning("Failed initializing analytics DB at %s: %s", self.analytics_path, e)

def _prune_analytics(self) -> None:
    """Delete analytics rows older than analytics_max_age_days."""
    if self.analytics_path is None or not self.analytics_path.exists():
        return
    cutoff = (datetime.now(timezone.utc) - timedelta(days=self.analytics_max_age_days)).isoformat()
    try:
        conn = sqlite3.connect(self.analytics_path)
        conn.execute("DELETE FROM light_events WHERE timestamp < ?", (cutoff,))
        conn.commit()
        conn.close()
    except Exception as e:
        logger.warning("Failed pruning analytics at %s: %s", self.analytics_path, e)
```

**Step 3: Rewrite `_append_analytics_event`**

Replace the entire method body:

```python
def _append_analytics_event(self, *, source_tool: str, lux: float | None, result: dict[str, Any]) -> None:
    """Insert one light-context analytics row into SQLite."""
    if not self.analytics_enabled or self.analytics_path is None:
        return
    self._init_analytics_db()
    obs = result.get("observations")
    if not isinstance(obs, dict):
        obs = {}
    actions = result.get("recommended_actions")
    reasons = result.get("reason_codes")
    try:
        conn = sqlite3.connect(self.analytics_path)
        conn.execute(
            """INSERT INTO light_events (
                timestamp, user_id, source_tool, context_state, recommended_mode,
                recommended_actions, confidence, cooldown_hint_s, reason_codes, lux,
                obs_lux_delta_60s, obs_local_hour, obs_is_night, obs_presence_detected,
                obs_active_interaction, obs_low_light_duration_min, obs_prefers_dim,
                obs_light_sensitive, obs_allow_wellness_nudges
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                datetime.now(timezone.utc).isoformat(),
                self.user_id,
                source_tool,
                result.get("context_state"),
                result.get("recommended_mode"),
                ",".join(actions) if isinstance(actions, list) else None,
                result.get("confidence"),
                result.get("cooldown_hint_s"),
                ",".join(reasons) if isinstance(reasons, list) else None,
                lux,
                obs.get("lux_delta_60s"),
                obs.get("local_hour"),
                int(obs["is_night"]) if "is_night" in obs else None,
                int(obs["presence_detected"]) if "presence_detected" in obs else None,
                int(obs["active_interaction"]) if "active_interaction" in obs else None,
                obs.get("low_light_duration_min"),
                int(obs["prefers_dim"]) if "prefers_dim" in obs else None,
                int(obs["light_sensitive"]) if "light_sensitive" in obs else None,
                int(obs["allow_wellness_nudges"]) if "allow_wellness_nudges" in obs else None,
            ),
        )
        conn.commit()
        conn.close()
        self._analytics_insert_count += 1
        if self._analytics_insert_count % 100 == 0:
            self._prune_analytics()
    except Exception as e:
        logger.warning("Failed writing analytics event to %s: %s", self.analytics_path, e)
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_light_orchestrator.py -v`
Expected: All pass (including existing `TestAnalyticsPermissions` tests — update those too if they reference JSONL-specific behavior).

**Step 5: Commit**

```
git add src/healthy_heartrate_breathing/light_orchestrator.py
git commit -m "feat: replace JSONL analytics with SQLite storage"
```

---

### Task 3: Update existing analytics tests for SQLite

**Files:**
- Modify: `tests/test_light_orchestrator.py` — update `TestAnalyticsPermissions` class

**Step 1: Update TestAnalyticsPermissions**

The existing tests in `TestAnalyticsPermissions` (lines 382-415) reference JSONL behavior. Update:
- `test_analytics_write_to_unwritable_dir_does_not_raise`: Change to verify `.db` file creation, and test that writing to an unwritable dir after initial success doesn't raise.
- `test_analytics_creates_parent_dirs`: Change from reading `.jsonl` lines to querying SQLite.
- `test_analytics_disabled_no_write`: Change `.jsonl` assertion to `.db` assertion.

Also remove the `analytics_max_bytes` parameter from the `_orchestrator` defaults if present.

**Step 2: Run all orchestrator tests**

Run: `uv run pytest tests/test_light_orchestrator.py -v`
Expected: All pass.

**Step 3: Commit**

```
git add tests/test_light_orchestrator.py
git commit -m "test: update analytics permission tests for SQLite"
```

---

### Task 4: Wire env var and update constructor call site

**Files:**
- Modify: `src/healthy_heartrate_breathing/openai_realtime.py:127-143`

**Step 1: Update LightOrchestrator construction**

- Change `analytics_path=...("light_context_analytics.jsonl")` to `analytics_path=...("light_context_analytics.db")`
- Replace `analytics_max_bytes=env_int("HEALTHY_LIGHT_ANALYTICS_MAX_BYTES", 5_000_000, min_value=1024)` with `analytics_max_age_days=env_int("HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS", 90, min_value=1)`

**Step 2: Run full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision --ignore=tests/test_config_name_collisions.py --ignore=tests/test_external_loading.py -q`
Expected: All pass.

**Step 3: Run lint and type check**

Run: `uv run ruff check . && uv run mypy src/`
Expected: Clean (except pre-existing mypy errors).

**Step 4: Commit**

```
git add src/healthy_heartrate_breathing/openai_realtime.py
git commit -m "feat: wire HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS env var"
```

---

### Task 5: Update all 4 documentation surfaces

**Files:**
- Modify: `CLAUDE.md` — update Light Context System section and env var table
- Modify: `README.md` — update env var table
- Modify: `docs/TODO.md` — update PY-MED-8 entry
- Modify: `docs/20260223_roadmap.md` — update PY-MED-8 entries

**Step 1: CLAUDE.md**

- Light Context System section: replace "JSONL analytics logging" line with "SQLite analytics storage (when `HEALTHY_LIGHT_ANALYTICS_ENABLED` is true); time-based retention via `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` (default 90)"
- Env var table: remove `HEALTHY_LIGHT_ANALYTICS_MAX_BYTES` row, add `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` row

**Step 2: README.md**

- Env var table: remove `HEALTHY_LIGHT_ANALYTICS_MAX_BYTES` row, add `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` row

**Step 3: docs/TODO.md**

- Update PY-MED-8 done entry to reflect SQLite (not JSONL rotation)

**Step 4: docs/20260223_roadmap.md**

- Update PY-MED-8 entries to reflect SQLite migration

**Step 5: Commit**

```
git add CLAUDE.md README.md docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: update all surfaces for analytics SQLite migration"
```

**Step 6: Push**

```
git push
```
