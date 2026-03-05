"""Vitals history SQLite store for dashboard graphs."""

from __future__ import annotations
import logging
import sqlite3
from typing import Any
from pathlib import Path
from datetime import datetime, timezone, timedelta


logger = logging.getLogger(__name__)

_SCHEMA_VERSION = 2


class VitalsStore:
    """Append-only vitals history with rolling time window pruning."""

    def __init__(self, *, db_path: Path, max_hours: int = 4) -> None:
        """Initialize store with database path and retention window."""
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
            if version == _SCHEMA_VERSION:
                conn.close()
                self._initialized = True
                return
            if version == 0:
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

                CREATE TABLE IF NOT EXISTS vitals_hourly (
                    id              INTEGER PRIMARY KEY AUTOINCREMENT,
                    hour_start      TEXT    NOT NULL UNIQUE,
                    hr_avg          REAL,
                    hr_min          REAL,
                    hr_max          REAL,
                    hr_count        INTEGER DEFAULT 0,
                    br_avg          REAL,
                    br_min          REAL,
                    br_max          REAL,
                    br_count        INTEGER DEFAULT 0,
                    lux_avg         REAL,
                    dominant_state  TEXT,
                    resting_minutes REAL    DEFAULT 0.0
                );
                CREATE INDEX IF NOT EXISTS idx_vitals_hourly_start
                    ON vitals_hourly(hour_start);

                CREATE TABLE IF NOT EXISTS vitals_daily (
                    id              INTEGER PRIMARY KEY AUTOINCREMENT,
                    day_start       TEXT    NOT NULL UNIQUE,
                    hr_avg          REAL,
                    hr_min          REAL,
                    hr_max          REAL,
                    hr_count        INTEGER DEFAULT 0,
                    br_avg          REAL,
                    br_min          REAL,
                    br_max          REAL,
                    br_count        INTEGER DEFAULT 0,
                    lux_avg         REAL,
                    dominant_state  TEXT,
                    resting_minutes REAL    DEFAULT 0.0
                );
                CREATE INDEX IF NOT EXISTS idx_vitals_daily_start
                    ON vitals_daily(day_start);

                CREATE TABLE IF NOT EXISTS trend_insights (
                    id              INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp       TEXT    NOT NULL,
                    category        TEXT    NOT NULL,
                    message         TEXT    NOT NULL,
                    severity        TEXT    NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_trend_insights_timestamp
                    ON trend_insights(timestamp);

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
                (
                    datetime.now(timezone.utc).isoformat(),
                    heart_rate_bpm,
                    breath_rate_bpm,
                    device_state,
                    target_count,
                    lux,
                ),
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
