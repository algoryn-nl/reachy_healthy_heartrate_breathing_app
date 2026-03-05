"""Tests for VitalsStore SQLite persistence."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import sqlite3
from pathlib import Path

import pytest

from healthy_heartrate_breathing.vitals_store import VitalsStore


@pytest.fixture
def store(tmp_path: Path) -> VitalsStore:
    return VitalsStore(db_path=tmp_path / "test_vitals.db", max_hours=4)


class TestAppendAndQuery:
    def test_append_and_query_returns_rows(self, store: VitalsStore) -> None:
        store.append(
            heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0
        )
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
        store.append(
            heart_rate_bpm=60.0, breath_rate_bpm=12.0, device_state="RESTING_VITALS", target_count=1, lux=50.0
        )
        # Query with 0 hours should return nothing (row is >0h old)
        rows = store.query(hours=0)
        assert rows == []


class TestPruning:
    def test_prune_removes_old_rows(self, tmp_path: Path) -> None:
        store = VitalsStore(db_path=tmp_path / "prune.db", max_hours=0)
        store.append(
            heart_rate_bpm=70.0, breath_rate_bpm=15.0, device_state="RESTING_VITALS", target_count=1, lux=40.0
        )
        store.prune()
        rows = store.query(hours=999)
        assert rows == []


class TestSchemaInit:
    def test_creates_db_file(self, tmp_path: Path) -> None:
        db_path = tmp_path / "new.db"
        assert not db_path.exists()
        store = VitalsStore(db_path=db_path, max_hours=4)
        store.append(
            heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0
        )
        assert db_path.exists()

    def test_idempotent_init(self, tmp_path: Path) -> None:
        db_path = tmp_path / "idem.db"
        s1 = VitalsStore(db_path=db_path, max_hours=4)
        s1.append(heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0)
        s2 = VitalsStore(db_path=db_path, max_hours=4)
        rows = s2.query(hours=1)
        assert len(rows) == 1


def _table_exists(db_path: Path, table_name: str) -> bool:
    conn = sqlite3.connect(db_path)
    row = conn.execute(
        "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name=?",
        (table_name,),
    ).fetchone()
    conn.close()
    return row[0] == 1


def _create_v1_db(db_path: Path) -> None:
    """Create a minimal v1 schema DB with one row of raw data."""
    conn = sqlite3.connect(db_path)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.executescript("""
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
        PRAGMA user_version=1;
    """)
    conn.execute(
        "INSERT INTO vitals_history (timestamp, heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux) "
        "VALUES (?, ?, ?, ?, ?, ?)",
        ("2026-03-05T12:00:00+00:00", 68.0, 14.0, "RESTING_VITALS", 1, 55.0),
    )
    conn.commit()
    conn.close()


class TestSchemaV2Migration:
    def test_v2_creates_hourly_table(self, store: VitalsStore) -> None:
        store.append(
            heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0
        )
        assert _table_exists(store._db_path, "vitals_hourly")

    def test_v2_creates_daily_table(self, store: VitalsStore) -> None:
        store.append(
            heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0
        )
        assert _table_exists(store._db_path, "vitals_daily")

    def test_v2_creates_trend_insights_table(self, store: VitalsStore) -> None:
        store.append(
            heart_rate_bpm=72.0, breath_rate_bpm=16.0, device_state="RESTING_VITALS", target_count=1, lux=42.0
        )
        assert _table_exists(store._db_path, "trend_insights")

    def test_v1_to_v2_migration_preserves_raw_data(self, tmp_path: Path) -> None:
        db_path = tmp_path / "v1_migrate.db"
        _create_v1_db(db_path)
        store = VitalsStore(db_path=db_path, max_hours=4)
        rows = store.query(hours=999)
        assert len(rows) == 1
        assert rows[0]["heart_rate_bpm"] == 68.0
        assert rows[0]["breath_rate_bpm"] == 14.0
        assert rows[0]["device_state"] == "RESTING_VITALS"
