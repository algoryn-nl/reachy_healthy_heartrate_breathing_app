"""Tests for TrendAnalyzer anomaly detection and trend summaries."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import sqlite3
from pathlib import Path
from datetime import datetime, timezone, timedelta

import pytest

from healthy_heartrate_breathing.vitals_store import VitalsStore
from healthy_heartrate_breathing.trend_analyzer import TrendInsight, TrendSummary, TrendAnalyzer


@pytest.fixture
def db_path(tmp_path: Path) -> Path:
    return tmp_path / "test_vitals.db"


@pytest.fixture
def store(db_path: Path) -> VitalsStore:
    s = VitalsStore(db_path=db_path, max_hours=999)
    s._ensure_schema()
    return s


@pytest.fixture
def analyzer(db_path: Path) -> TrendAnalyzer:
    return TrendAnalyzer(db_path=db_path)


def _insert_daily(db_path: Path, day_offset: int, hr_avg: float, br_avg: float, hr_count: int = 100) -> None:
    day = (datetime.now(timezone.utc) - timedelta(days=day_offset)).replace(hour=0, minute=0, second=0, microsecond=0)
    conn = sqlite3.connect(db_path)
    conn.execute(
        "INSERT OR REPLACE INTO vitals_daily "
        "(day_start, hr_avg, hr_min, hr_max, hr_count, br_avg, br_min, br_max, br_count, lux_avg, "
        "dominant_state, resting_minutes) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 50.0, 'RESTING_VITALS', 120.0)",
        (day.isoformat(), hr_avg, hr_avg - 5, hr_avg + 5, hr_count, br_avg, br_avg - 2, br_avg + 2, hr_count),
    )
    conn.commit()
    conn.close()


class TestAnalyzeSession:
    def test_returns_none_with_insufficient_data(self, store: VitalsStore, analyzer: TrendAnalyzer) -> None:
        result = analyzer.analyze_session(current_hr=72.0, current_br=16.0)
        assert result is None

    def test_returns_none_when_values_normal(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(7):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=15.0)
        result = analyzer.analyze_session(current_hr=70.0, current_br=15.0)
        assert result is None

    def test_detects_elevated_hr_statistical(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(7):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=65.0 + i * 0.5, br_avg=15.0)
        result = analyzer.analyze_session(current_hr=90.0, current_br=15.0)
        assert result is not None
        assert isinstance(result, TrendInsight)
        assert "hr" in result.category.lower()
        assert result.severity == "info"

    def test_detects_absolute_hr_high(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=95.0, br_avg=15.0)
        result = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert result is not None
        assert result.category == "elevated_hr"
        assert result.severity == "attention"

    def test_detects_absolute_hr_low(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=50.0, br_avg=15.0)
        result = analyzer.analyze_session(current_hr=40.0, current_br=15.0)
        assert result is not None
        assert result.category == "low_hr"
        assert result.severity == "attention"

    def test_detects_absolute_br_high(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=20.0)
        result = analyzer.analyze_session(current_hr=70.0, current_br=28.0)
        assert result is not None
        assert result.category == "elevated_br"
        assert result.severity == "attention"

    def test_cooldown_suppresses_repeat_insights(self, store: VitalsStore, db_path: Path) -> None:
        analyzer = TrendAnalyzer(db_path=db_path, cooldown_s=900.0)
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=15.0)
        first = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert first is not None
        second = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert second is None

    def test_cooldown_expires(self, store: VitalsStore, db_path: Path) -> None:
        analyzer = TrendAnalyzer(db_path=db_path, cooldown_s=0.0)
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=15.0)
        first = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert first is not None
        second = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert second is not None


class TestGetSummary:
    def test_returns_summary_with_data(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(7):
            _insert_daily(db_path, day_offset=i, hr_avg=70.0, br_avg=15.0)
        summary = analyzer.get_summary(days=7)
        assert isinstance(summary, TrendSummary)
        assert summary.days_of_data == 7
        assert summary.hr_avg_7d is not None
        assert abs(summary.hr_avg_7d - 70.0) < 1.0
        assert summary.br_avg_7d is not None
        assert summary.hr_trend == "stable"
        assert summary.br_trend == "stable"
        assert summary.resting_minutes_avg is not None
        assert len(summary.daily_series) == 7

    def test_returns_empty_summary_without_data(self, store: VitalsStore, analyzer: TrendAnalyzer) -> None:
        summary = analyzer.get_summary(days=7)
        assert summary.days_of_data == 0
        assert summary.hr_avg_7d is None
        assert summary.br_avg_7d is None

    def test_trend_detects_increasing_hr(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(7):
            _insert_daily(db_path, day_offset=7 - i, hr_avg=60.0 + i * 5.0, br_avg=15.0)
        summary = analyzer.get_summary(days=7)
        assert summary.hr_trend == "increasing"

    def test_trend_detects_decreasing_hr(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(7):
            _insert_daily(db_path, day_offset=7 - i, hr_avg=90.0 - i * 5.0, br_avg=15.0)
        summary = analyzer.get_summary(days=7)
        assert summary.hr_trend == "decreasing"


class TestGetDailySeries:
    def test_returns_daily_rows(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0 + i, br_avg=15.0)
        rows = analyzer.get_daily_series(days=7)
        assert len(rows) == 3
        assert "hr_avg" in rows[0]

    def test_returns_empty_without_data(self, store: VitalsStore, analyzer: TrendAnalyzer) -> None:
        rows = analyzer.get_daily_series(days=7)
        assert rows == []


class TestInsightPersistence:
    def test_insight_written_to_db(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=15.0)
        result = analyzer.analyze_session(current_hr=105.0, current_br=15.0)
        assert result is not None
        conn = sqlite3.connect(db_path)
        count = conn.execute("SELECT COUNT(*) FROM trend_insights").fetchone()[0]
        conn.close()
        assert count == 1

    def test_recent_insights_returns_latest(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        analyzer_no_cd = TrendAnalyzer(db_path=db_path, cooldown_s=0.0)
        for i in range(3):
            _insert_daily(db_path, day_offset=i + 1, hr_avg=70.0, br_avg=15.0)
        analyzer_no_cd.analyze_session(current_hr=105.0, current_br=15.0)
        analyzer_no_cd.analyze_session(current_hr=42.0, current_br=15.0)
        insights = analyzer_no_cd.recent_insights(limit=5)
        assert len(insights) == 2
        assert insights[0]["category"] == "low_hr"
