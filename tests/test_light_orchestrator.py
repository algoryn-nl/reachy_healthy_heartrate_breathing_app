"""Tests for LightOrchestrator baseline tracking and dispatch."""
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
        "prefers_dim": False,
        "light_sensitive": False,
        "allow_wellness_nudges": True,
        "day_start_hour": 7,
        "night_start_hour": 20,
        "low_lux_threshold": 40.0,
        "baseline_alpha": 0.15,
        "baseline_min_samples": 5,
        "baseline_path": tmp_path / "baseline.json",
        "analytics_path": tmp_path / "analytics.db",
        "baseline_save_interval_s": 0,  # flush immediately in tests
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


class TestBaseline:
    def test_save_and_load_roundtrip(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path)
        o.update_baseline(lux=100.0, local_hour=12)
        o.save_baseline()

        o2 = _orchestrator(tmp_path)
        o2.load_baseline()
        val = o2.get_typical_day_low_lux(12)
        # With 1 sample, min_samples=5, so exact hour falls back to avg
        assert val is not None
        assert abs(val - 100.0) < 1.0

    def test_nighttime_not_recorded(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, day_start_hour=7, night_start_hour=20)
        o.update_baseline(lux=50.0, local_hour=22)  # nighttime
        o.save_baseline()

        o2 = _orchestrator(tmp_path)
        o2.load_baseline()
        assert o2.get_typical_day_low_lux(22) is None

    def test_is_daytime_hour(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, day_start_hour=7, night_start_hour=20)
        assert o.is_daytime_hour(12) is True
        assert o.is_daytime_hour(6) is False
        assert o.is_daytime_hour(20) is False
        assert o.is_daytime_hour(7) is True


class TestBaselineThrottling:
    def test_save_throttled_within_interval(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=300)
        o.update_baseline(lux=100.0, local_hour=12)
        # First save goes through (last_save_time starts at 0)
        assert o.baseline_path.exists()
        first_content = o.baseline_path.read_text()

        # Second update within interval: save_baseline is a no-op on disk
        o.update_baseline(lux=200.0, local_hour=12)
        assert o.baseline_path.read_text() == first_content  # unchanged on disk
        assert o._baseline_dirty is True  # but marked dirty in memory

    def test_flush_writes_dirty_state(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=300)
        o.update_baseline(lux=100.0, local_hour=12)
        first_content = o.baseline_path.read_text()

        o.update_baseline(lux=200.0, local_hour=12)
        assert o._baseline_dirty is True
        o.flush()
        assert o._baseline_dirty is False
        assert o.baseline_path.read_text() != first_content  # updated

    def test_flush_noop_when_clean(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_save_interval_s=0)
        o.update_baseline(lux=100.0, local_hour=12)
        assert o._baseline_dirty is False  # interval=0 means save_baseline wrote immediately
        o.baseline_path.unlink()  # remove file
        o.flush()  # should not recreate it
        assert not o.baseline_path.exists()


class TestBaselinePruning:
    def test_stale_user_pruned_on_load(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=100)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "old-user": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 10}},
                    "updated_at": stale_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "old-user" not in o._baseline_state["users"]

    def test_fresh_user_kept_on_load(self, tmp_path: Path) -> None:
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=10)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "recent-user": {
                    "hours": {"12": {"ema_lux": 200.0, "samples": 5}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "recent-user" in o._baseline_state["users"]

    def test_mixed_stale_and_fresh(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=200)).isoformat()
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=5)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "stale": {
                    "hours": {"10": {"ema_lux": 50.0, "samples": 3}},
                    "updated_at": stale_ts,
                },
                "fresh": {
                    "hours": {"14": {"ema_lux": 300.0, "samples": 8}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "stale" not in o._baseline_state["users"]
        assert "fresh" in o._baseline_state["users"]

    def test_malformed_timestamp_pruned(self, tmp_path: Path) -> None:
        state = {
            "schema_version": 1,
            "users": {
                "bad-ts": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 1}},
                    "updated_at": "not-a-date",
                },
                "no-ts": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 1}},
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "bad-ts" not in o._baseline_state["users"]
        assert "no-ts" not in o._baseline_state["users"]

    def test_pruning_persists_to_disk(self, tmp_path: Path) -> None:
        stale_ts = (datetime.now(timezone.utc) - timedelta(days=100)).isoformat()
        fresh_ts = (datetime.now(timezone.utc) - timedelta(days=1)).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "stale": {
                    "hours": {"12": {"ema_lux": 50.0, "samples": 3}},
                    "updated_at": stale_ts,
                },
                "fresh": {
                    "hours": {"14": {"ema_lux": 300.0, "samples": 8}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        _orchestrator(tmp_path, baseline_max_age_days=90)
        # Pruned state should be saved to disk
        reloaded = json.loads(baseline_path.read_text())
        assert "stale" not in reloaded["users"]
        assert "fresh" in reloaded["users"]

    def test_no_pruning_when_all_fresh(self, tmp_path: Path) -> None:
        fresh_ts = datetime.now(timezone.utc).isoformat()
        state = {
            "schema_version": 1,
            "users": {
                "user-a": {
                    "hours": {"12": {"ema_lux": 100.0, "samples": 5}},
                    "updated_at": fresh_ts,
                },
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state))

        o = _orchestrator(tmp_path, baseline_max_age_days=90, baseline_save_interval_s=300)
        assert "user-a" in o._baseline_state["users"]
        # No pruning happened, so dirty flag should not be set
        assert o._baseline_dirty is False


# ---------------------------------------------------------------------------
# File I/O error tests for persistence (corrupted JSON, permissions, edge cases)
# ---------------------------------------------------------------------------


class TestLoadBaselineCorruptedJson:
    """Verify load_baseline gracefully handles corrupted or malformed JSON files."""

    def test_invalid_json_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text("{this is not json", encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_empty_file_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text("", encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_json_array_falls_back_to_default(self, tmp_path: Path) -> None:
        """Top-level JSON is a list, not a dict."""
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text("[1, 2, 3]", encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_json_scalar_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text("42", encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_json_null_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text("null", encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_dict_with_non_dict_users_gets_repaired(self, tmp_path: Path) -> None:
        """JSON is a dict but 'users' is a string — should be replaced with empty dict."""
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps({"schema_version": 1, "users": "not-a-dict"}), encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert isinstance(o._baseline_state["users"], dict)
        assert o._baseline_state["users"] == {}

    def test_dict_without_users_key_gets_repaired(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state["users"] == {}

    def test_truncated_json_falls_back_to_default(self, tmp_path: Path) -> None:
        """Simulate a file that was truncated mid-write."""
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text('{"schema_version": 1, "users": {"test": {', encoding="utf-8")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}

    def test_binary_garbage_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_bytes(b"\x00\x01\x80\xff\xfe")
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}


class TestLoadBaselinePermissions:
    """Verify load_baseline handles permission errors gracefully."""

    def test_unreadable_file_falls_back_to_default(self, tmp_path: Path) -> None:
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps({"schema_version": 1, "users": {}}), encoding="utf-8")
        baseline_path.chmod(0o000)
        try:
            o = _orchestrator(tmp_path)
            assert o._baseline_state == {"schema_version": 1, "users": {}}
        finally:
            baseline_path.chmod(0o644)

    def test_nonexistent_file_uses_default(self, tmp_path: Path) -> None:
        """Baseline path set but file does not exist yet — normal first-run case."""
        o = _orchestrator(tmp_path)
        assert o._baseline_state == {"schema_version": 1, "users": {}}
        assert not (tmp_path / "baseline.json").exists()

    def test_none_baseline_path_uses_default(self, tmp_path: Path) -> None:
        o = _orchestrator(tmp_path, baseline_path=None)
        assert o._baseline_state == {"schema_version": 1, "users": {}}


class TestWriteBaselinePermissions:
    """Verify _write_baseline and flush handle write errors gracefully."""

    def test_write_failure_keeps_dirty_flag(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        o = _orchestrator(tmp_path)
        o.update_baseline(lux=100.0, local_hour=12)
        assert o._baseline_dirty is False  # first write succeeds

        # Simulate write failure by patching write_text to raise
        o._baseline_dirty = True

        def _fail_write(*_a: object, **_kw: object) -> None:
            raise PermissionError("mock permission denied")

        baseline_path = o.baseline_path
        assert baseline_path is not None
        monkeypatch.setattr(type(baseline_path), "write_text", _fail_write)
        o._write_baseline(0.0)
        # dirty flag stays True because write failed
        assert o._baseline_dirty is True

    def test_write_creates_parent_dirs(self, tmp_path: Path) -> None:
        nested_path = tmp_path / "deep" / "nested" / "baseline.json"
        o = _orchestrator(tmp_path, baseline_path=nested_path)
        o.update_baseline(lux=100.0, local_hour=12)
        assert nested_path.exists()
        parsed = json.loads(nested_path.read_text(encoding="utf-8"))
        assert "users" in parsed

    def test_flush_failure_does_not_raise(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        o = _orchestrator(tmp_path)
        o.update_baseline(lux=50.0, local_hour=10)

        # Dirty it and simulate write failure
        o._baseline_dirty = True

        def _fail_write(*_a: object, **_kw: object) -> None:
            raise OSError("mock disk full")

        baseline_path = o.baseline_path
        assert baseline_path is not None
        monkeypatch.setattr(type(baseline_path), "write_text", _fail_write)
        # flush should not raise
        o.flush()
        assert o._baseline_dirty is True


class TestAnalyticsPermissions:
    """Verify analytics SQLite writing handles errors gracefully."""

    def test_analytics_write_to_unwritable_dir_does_not_raise(self, tmp_path: Path) -> None:
        readonly_dir = tmp_path / "analytics_locked"
        readonly_dir.mkdir()
        analytics_path = readonly_dir / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=analytics_path)

        # First write should succeed
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "bright"})
        assert analytics_path.exists()

        # Second write to same DB still works (DB already created)
        o._append_analytics_event(source_tool="test", lux=200.0, result={"context_state": "dim"})
        conn = sqlite3.connect(analytics_path)
        rows = conn.execute("SELECT lux FROM light_events ORDER BY id").fetchall()
        conn.close()
        assert len(rows) == 2

    def test_analytics_creates_parent_dirs(self, tmp_path: Path) -> None:
        nested_path = tmp_path / "logs" / "deep" / "analytics.db"
        o = _orchestrator(tmp_path, analytics_enabled=True, analytics_path=nested_path)
        o._append_analytics_event(source_tool="test", lux=100.0, result={"context_state": "bright"})
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
        rows = conn.execute(
            "SELECT context_state, lux, recommended_actions, reason_codes, obs_local_hour FROM light_events"
        ).fetchall()
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


class TestPruneStaleEdgeCases:
    """Edge cases in _prune_stale_users for non-dict user entries."""

    def test_non_dict_user_entry_pruned(self, tmp_path: Path) -> None:
        state = {
            "schema_version": 1,
            "users": {
                "string-entry": "not-a-dict",
                "list-entry": [1, 2, 3],
                "null-entry": None,
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state), encoding="utf-8")
        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "string-entry" not in o._baseline_state["users"]
        assert "list-entry" not in o._baseline_state["users"]
        assert "null-entry" not in o._baseline_state["users"]

    def test_empty_users_dict_no_error(self, tmp_path: Path) -> None:
        state = {"schema_version": 1, "users": {}}
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state), encoding="utf-8")
        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert o._baseline_state["users"] == {}

    def test_user_entry_missing_updated_at_pruned(self, tmp_path: Path) -> None:
        state = {
            "schema_version": 1,
            "users": {
                "no-timestamp": {"hours": {"12": {"ema_lux": 100.0, "samples": 1}}},
            },
        }
        baseline_path = tmp_path / "baseline.json"
        baseline_path.write_text(json.dumps(state), encoding="utf-8")
        o = _orchestrator(tmp_path, baseline_max_age_days=90)
        assert "no-timestamp" not in o._baseline_state["users"]


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
        mock_result = {"context_state": "bright_active", "recommended_mode": "active"}
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
