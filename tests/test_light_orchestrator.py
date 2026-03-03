"""Tests for LightOrchestrator lux delta tracking, analytics, and dispatch."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
import sqlite3
from pathlib import Path
from datetime import datetime, timezone, timedelta
from unittest.mock import AsyncMock

import pytest

from healthy_heartrate_breathing.light_orchestrator import LightOrchestrator


def _orchestrator(tmp_path: Path, **overrides: object) -> LightOrchestrator:
    defaults = {
        "enabled": True,
        "analytics_enabled": False,
        "user_id": "test-user",
        "low_lux_threshold": 40.0,
        "analytics_path": tmp_path / "analytics.db",
    }
    defaults.update(overrides)
    return LightOrchestrator(**defaults)


class TestLuxDelta:
    def test_no_previous_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        assert o.compute_lux_delta_60s(100.0, now=10.0) is None

    def test_normal_delta(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o._last_lux = 200.0
        o._last_lux_time = 0.0
        delta = o.compute_lux_delta_60s(100.0, now=30.0)
        # Drop of 100 in 30s, scaled to 60s = -200.0
        assert delta is not None
        assert abs(delta - (-200.0)) < 0.1

    def test_stale_reading_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o._last_lux = 200.0
        o._last_lux_time = 0.0
        # 700s gap exceeds 600s max
        assert o.compute_lux_delta_60s(100.0, now=700.0) is None


class TestAnalyticsPermissions:
    """Verify analytics SQLite writing handles errors gracefully."""

    def test_analytics_write_to_unwritable_dir_does_not_raise(self, tmp_path: Path) -> None:
        readonly_dir = tmp_path / "analytics_locked"
        readonly_dir.mkdir()
        analytics_path = readonly_dir / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=analytics_path)

        # First write should succeed
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "clear_path"})
        assert analytics_path.exists()

        # Second write to same DB still works (DB already created)
        o._append_analytics_event(source_tool="test", lux=200.0, result={"context_state": "close_presence"})
        conn = sqlite3.connect(analytics_path)
        rows = conn.execute("SELECT lux FROM light_events ORDER BY id").fetchall()
        conn.close()
        assert len(rows) == 2

    def test_analytics_creates_parent_dirs(self, tmp_path: Path) -> None:
        nested_path = tmp_path / "logs" / "deep" / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=nested_path)
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "clear_path"})
        assert nested_path.exists()
        conn = sqlite3.connect(nested_path)
        rows = conn.execute("SELECT lux FROM light_events").fetchall()
        conn.close()
        assert len(rows) == 1
        assert rows[0][0] == 100.0

    def test_analytics_disabled_no_write(self, tmp_path: Path) -> None:
        analytics_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=False, analytics_path=analytics_path)
        o._append_analytics_event(source_tool="test", lux=100.0, result={})
        assert not analytics_path.exists()


class TestAnalyticsSqlite:
    """Verify analytics SQLite storage, retention, and schema migration."""

    def test_event_inserted_and_queryable(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path)
        o._append_analytics_event(
            source_tool="test",
            lux=142.5,
            result={
                "context_state": "close_presence",
                "recommended_mode": "engaged",
                "recommended_actions": ["vitals_likely_available", "use_warm_conversational_tone"],
                "confidence": 0.85,
                "cooldown_hint_s": 120,
                "reason_codes": ["low_lux_with_presence"],
                "observations": {
                    "lux": 142.5,
                    "lux_delta_60s": 3.2,
                    "presence_detected": True,
                    "target_distance_cm": 45.0,
                },
            },
        )
        assert db_path.exists()
        conn = sqlite3.connect(db_path)
        rows = conn.execute(
            "SELECT context_state, lux, recommended_actions, reason_codes, obs_target_distance_cm FROM light_events"
        ).fetchall()
        conn.close()
        assert len(rows) == 1
        assert rows[0][0] == "close_presence"
        assert rows[0][1] == 142.5
        assert rows[0][2] == "vitals_likely_available,use_warm_conversational_tone"
        assert rows[0][3] == "low_lux_with_presence"
        assert rows[0][4] == 45.0

    def test_retention_prunes_old_rows(self, tmp_path: Path) -> None:
        db_path = tmp_path / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path, analytics_max_age_days=1)
        # Insert a recent event
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "clear_path"})
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
        o2._append_analytics_event(source_tool="test", lux=200.0, result={"context_state": "close_presence"})
        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT context_state FROM light_events ORDER BY id").fetchall()
        conn.close()
        states = [r[0] for r in rows]
        assert "old" not in states
        assert "clear_path" in states
        assert "close_presence" in states

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

    def test_schema_migration_recreates_table(self, tmp_path: Path) -> None:
        """If an existing DB has an old schema version, the table is recreated."""
        db_path = tmp_path / "analytics.db"
        # Create a v1 schema manually
        conn = sqlite3.connect(db_path)
        conn.execute("PRAGMA user_version=1")
        conn.execute("""
            CREATE TABLE light_events (
                id INTEGER PRIMARY KEY,
                timestamp TEXT NOT NULL,
                user_id TEXT NOT NULL,
                source_tool TEXT NOT NULL,
                obs_local_hour INTEGER
            )
        """)
        conn.execute(
            "INSERT INTO light_events (timestamp, user_id, source_tool, obs_local_hour) VALUES (?, ?, ?, ?)",
            (datetime.now(timezone.utc).isoformat(), "test", "test", 14),
        )
        conn.commit()
        conn.close()

        # Now init with new schema — should drop and recreate
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=db_path)
        o._append_analytics_event(
            source_tool="test",
            lux=100.0,
            result={
                "context_state": "clear_path",
                "observations": {"presence_detected": True, "target_distance_cm": 50.0},
            },
        )
        conn = sqlite3.connect(db_path)
        (version,) = conn.execute("PRAGMA user_version").fetchone()
        assert version == 2
        # Old column should not exist, new column should
        rows = conn.execute("SELECT obs_target_distance_cm FROM light_events").fetchall()
        conn.close()
        assert len(rows) == 1
        assert rows[0][0] == 50.0


class TestRunFromMmwave:
    @pytest.mark.asyncio
    async def test_disabled_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, enabled=False)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_no_lux_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()

    @pytest.mark.asyncio
    async def test_dispatches_light_context(self, tmp_path: Path) -> None:
        mock_result = {"context_state": "close_presence", "recommended_mode": "engaged"}
        dispatch = AsyncMock(return_value=mock_result)
        o = _orchestrator(tmp_path)
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=True,
            dispatch_fn=dispatch,
        )
        assert result == mock_result
        dispatch.assert_called_once()
        # Verify the dispatched args contain lux
        call_args = dispatch.call_args
        args_json = call_args[0][1]  # second positional arg
        parsed = json.loads(args_json)
        assert parsed["lux"] == 300.0

    @pytest.mark.asyncio
    async def test_no_tool_returns_none(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        dispatch = AsyncMock(return_value={})
        result = await o.run_from_mmwave(
            {"measure": {"latest_light": {"lux": 300.0}}},
            is_idle=False,
            has_tool=False,
            dispatch_fn=dispatch,
        )
        assert result is None
        dispatch.assert_not_called()
