"""Tests for VitalsAggregator hourly/daily rollup and pruning."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import sqlite3
from pathlib import Path
from datetime import datetime, timezone, timedelta

import pytest

from healthy_heartrate_breathing.vitals_store import VitalsStore
from healthy_heartrate_breathing.vitals_aggregator import VitalsAggregator


@pytest.fixture
def db_path(tmp_path: Path) -> Path:
    return tmp_path / "test_vitals.db"


@pytest.fixture
def store(db_path: Path) -> VitalsStore:
    s = VitalsStore(db_path=db_path, max_hours=999)
    s._ensure_schema()  # force schema creation
    return s


@pytest.fixture
def aggregator(db_path: Path) -> VitalsAggregator:
    return VitalsAggregator(db_path=db_path)


def _insert_raw_row(
    db_path: Path,
    ts: str,
    hr: float | None,
    br: float | None,
    state: str = "RESTING_VITALS",
    lux: float = 50.0,
) -> None:
    conn = sqlite3.connect(db_path)
    conn.execute(
        "INSERT INTO vitals_history (timestamp, heart_rate_bpm, breath_rate_bpm, device_state, target_count, lux) "
        "VALUES (?, ?, ?, ?, 1, ?)",
        (ts, hr, br, state, lux),
    )
    conn.commit()
    conn.close()


def _two_hours_ago() -> datetime:
    return datetime.now(timezone.utc).replace(minute=0, second=0, microsecond=0) - timedelta(hours=2)


def _yesterday() -> datetime:
    return (datetime.now(timezone.utc) - timedelta(days=1)).replace(hour=12, minute=0, second=0, microsecond=0)


# ---------------------------------------------------------------------------
# TestHourlyRollup
# ---------------------------------------------------------------------------


class TestHourlyRollup:
    def test_rollup_creates_hourly_row_for_completed_hour(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        ts = _two_hours_ago().isoformat()
        _insert_raw_row(db_path, ts, 72.0, 16.0)

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_hourly").fetchall()
        conn.close()
        assert len(rows) == 1

    def test_rollup_computes_correct_averages(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        base = _two_hours_ago()
        _insert_raw_row(db_path, base.isoformat(), 70.0, 14.0)
        _insert_raw_row(db_path, (base + timedelta(minutes=5)).isoformat(), 80.0, 18.0)

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        row = conn.execute("SELECT * FROM vitals_hourly").fetchone()
        conn.close()

        assert row["hr_avg"] == pytest.approx(75.0)
        assert row["hr_min"] == pytest.approx(70.0)
        assert row["hr_max"] == pytest.approx(80.0)
        assert row["hr_count"] == 2
        assert row["br_avg"] == pytest.approx(16.0)
        assert row["br_min"] == pytest.approx(14.0)
        assert row["br_max"] == pytest.approx(18.0)
        assert row["br_count"] == 2

    def test_rollup_skips_current_hour(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        now = datetime.now(timezone.utc)
        _insert_raw_row(db_path, now.isoformat(), 72.0, 16.0)

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_hourly").fetchall()
        conn.close()
        assert len(rows) == 0

    def test_rollup_is_idempotent(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        ts = _two_hours_ago().isoformat()
        _insert_raw_row(db_path, ts, 72.0, 16.0)

        aggregator.rollup_hourly()
        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_hourly").fetchall()
        conn.close()
        assert len(rows) == 1

    def test_rollup_skips_rows_without_vitals(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        base = _two_hours_ago()
        _insert_raw_row(db_path, base.isoformat(), None, None, state="NO_TARGET")
        _insert_raw_row(db_path, (base + timedelta(minutes=5)).isoformat(), 72.0, 16.0)

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        row = conn.execute("SELECT * FROM vitals_hourly").fetchone()
        conn.close()

        assert row["hr_count"] == 1
        assert row["hr_avg"] == pytest.approx(72.0)

    def test_dominant_state_is_most_frequent(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        base = _two_hours_ago()
        _insert_raw_row(db_path, base.isoformat(), 70.0, 14.0, state="RESTING_VITALS")
        _insert_raw_row(db_path, (base + timedelta(minutes=1)).isoformat(), 72.0, 15.0, state="RESTING_VITALS")
        _insert_raw_row(db_path, (base + timedelta(minutes=2)).isoformat(), 80.0, 18.0, state="MOVING")

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        row = conn.execute("SELECT * FROM vitals_hourly").fetchone()
        conn.close()
        assert row["dominant_state"] == "RESTING_VITALS"

    def test_resting_minutes_calculated(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        base = _two_hours_ago()
        _insert_raw_row(db_path, base.isoformat(), 70.0, 14.0, state="RESTING_VITALS")
        _insert_raw_row(db_path, (base + timedelta(minutes=1)).isoformat(), 72.0, 15.0, state="RESTING_VITALS")
        _insert_raw_row(db_path, (base + timedelta(minutes=2)).isoformat(), 80.0, 18.0, state="MOVING")

        aggregator.rollup_hourly()

        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        row = conn.execute("SELECT * FROM vitals_hourly").fetchone()
        conn.close()

        # 2 out of 3 rows are RESTING_VITALS → resting_minutes = 2/3 * 60 = 40.0
        assert row["resting_minutes"] == pytest.approx(40.0)


# ---------------------------------------------------------------------------
# TestDailyRollup
# ---------------------------------------------------------------------------


class TestDailyRollup:
    def test_rollup_creates_daily_row_from_hourly(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        yesterday = _yesterday().replace(hour=0, minute=0, second=0, microsecond=0)
        hour_start = yesterday.replace(hour=10).strftime("%Y-%m-%dT%H:00:00+00:00")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_min, hr_max, hr_count, "
            "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
            "VALUES (?, 72.0, 70.0, 74.0, 10, 16.0, 14.0, 18.0, 10, 50.0, 'RESTING_VITALS', 40.0)",
            (hour_start,),
        )
        conn.commit()
        conn.close()

        aggregator.rollup_daily()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_daily").fetchall()
        conn.close()
        assert len(rows) == 1

    def test_daily_rollup_computes_weighted_averages(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        yesterday = _yesterday().replace(hour=0, minute=0, second=0, microsecond=0)
        h1 = yesterday.replace(hour=10).strftime("%Y-%m-%dT%H:00:00+00:00")
        h2 = yesterday.replace(hour=11).strftime("%Y-%m-%dT%H:00:00+00:00")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_min, hr_max, hr_count, "
            "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
            "VALUES (?, 70.0, 65.0, 75.0, 10, 14.0, 12.0, 16.0, 10, 40.0, 'RESTING_VITALS', 50.0)",
            (h1,),
        )
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_min, hr_max, hr_count, "
            "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
            "VALUES (?, 80.0, 70.0, 90.0, 10, 18.0, 16.0, 20.0, 10, 60.0, 'MOVING', 10.0)",
            (h2,),
        )
        conn.commit()
        conn.close()

        aggregator.rollup_daily()

        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        row = conn.execute("SELECT * FROM vitals_daily").fetchone()
        conn.close()

        # Weighted avg: (70*10 + 80*10) / 20 = 75.0
        assert row["hr_avg"] == pytest.approx(75.0)
        assert row["hr_min"] == pytest.approx(65.0)
        assert row["hr_max"] == pytest.approx(90.0)
        assert row["hr_count"] == 20
        assert row["br_avg"] == pytest.approx(16.0)
        assert row["resting_minutes"] == pytest.approx(60.0)  # 50 + 10

    def test_daily_rollup_skips_today(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        today = datetime.now(timezone.utc).replace(hour=10, minute=0, second=0, microsecond=0)
        hour_start = today.strftime("%Y-%m-%dT%H:00:00+00:00")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_min, hr_max, hr_count, "
            "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
            "VALUES (?, 72.0, 70.0, 74.0, 10, 16.0, 14.0, 18.0, 10, 50.0, 'RESTING_VITALS', 40.0)",
            (hour_start,),
        )
        conn.commit()
        conn.close()

        aggregator.rollup_daily()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_daily").fetchall()
        conn.close()
        assert len(rows) == 0

    def test_daily_rollup_is_idempotent(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        yesterday = _yesterday().replace(hour=0, minute=0, second=0, microsecond=0)
        hour_start = yesterday.replace(hour=10).strftime("%Y-%m-%dT%H:00:00+00:00")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_min, hr_max, hr_count, "
            "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
            "VALUES (?, 72.0, 70.0, 74.0, 10, 16.0, 14.0, 18.0, 10, 50.0, 'RESTING_VITALS', 40.0)",
            (hour_start,),
        )
        conn.commit()
        conn.close()

        aggregator.rollup_daily()
        aggregator.rollup_daily()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_daily").fetchall()
        conn.close()
        assert len(rows) == 1


# ---------------------------------------------------------------------------
# TestPruneAggregates
# ---------------------------------------------------------------------------


class TestPruneAggregates:
    def test_prune_removes_old_hourly_rows(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        # Use retention=0 so everything is "old"
        agg = VitalsAggregator(db_path=db_path, hourly_retention_days=0, daily_retention_days=90)
        old_ts = (datetime.now(timezone.utc) - timedelta(days=2)).strftime("%Y-%m-%dT%H:00:00+00:00")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_count, br_avg, br_count) VALUES (?, 72.0, 5, 16.0, 5)",
            (old_ts,),
        )
        conn.commit()
        conn.close()

        agg.prune()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_hourly").fetchall()
        conn.close()
        assert len(rows) == 0

    def test_prune_removes_old_daily_rows(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        agg = VitalsAggregator(db_path=db_path, hourly_retention_days=30, daily_retention_days=0)
        old_ts = (datetime.now(timezone.utc) - timedelta(days=100)).strftime("%Y-%m-%d")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_daily (day_start, hr_avg, hr_count, br_avg, br_count) VALUES (?, 72.0, 50, 16.0, 50)",
            (old_ts,),
        )
        conn.commit()
        conn.close()

        agg.prune()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM vitals_daily").fetchall()
        conn.close()
        assert len(rows) == 0

    def test_prune_keeps_recent_rows(self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path) -> None:
        recent_hourly = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:00:00+00:00")
        recent_daily = datetime.now(timezone.utc).strftime("%Y-%m-%d")

        conn = sqlite3.connect(db_path)
        conn.execute(
            "INSERT INTO vitals_hourly (hour_start, hr_avg, hr_count, br_avg, br_count) VALUES (?, 72.0, 5, 16.0, 5)",
            (recent_hourly,),
        )
        conn.execute(
            "INSERT INTO vitals_daily (day_start, hr_avg, hr_count, br_avg, br_count) VALUES (?, 72.0, 50, 16.0, 50)",
            (recent_daily,),
        )
        conn.commit()
        conn.close()

        aggregator.prune()

        conn = sqlite3.connect(db_path)
        hourly = conn.execute("SELECT * FROM vitals_hourly").fetchall()
        daily = conn.execute("SELECT * FROM vitals_daily").fetchall()
        conn.close()
        assert len(hourly) == 1
        assert len(daily) == 1

    def test_prune_trims_trend_insights_to_50(
        self, store: VitalsStore, aggregator: VitalsAggregator, db_path: Path
    ) -> None:
        conn = sqlite3.connect(db_path)
        for i in range(60):
            ts = (datetime.now(timezone.utc) - timedelta(minutes=60 - i)).isoformat()
            conn.execute(
                "INSERT INTO trend_insights (timestamp, category, message, severity) VALUES (?, ?, ?, ?)",
                (ts, "hr", f"insight {i}", "info"),
            )
        conn.commit()
        conn.close()

        aggregator.prune()

        conn = sqlite3.connect(db_path)
        rows = conn.execute("SELECT * FROM trend_insights").fetchall()
        conn.close()
        assert len(rows) == 50
