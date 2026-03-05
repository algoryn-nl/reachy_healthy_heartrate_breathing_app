"""Tests for vitals_trends on-demand tool."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import sqlite3
from pathlib import Path
from datetime import datetime, timezone, timedelta
from unittest.mock import MagicMock

import pytest

from healthy_heartrate_breathing.vitals_store import VitalsStore
from healthy_heartrate_breathing.trend_analyzer import TrendAnalyzer


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


def _insert_daily(db_path: Path, day_offset: int, hr_avg: float, br_avg: float) -> None:
    day = (datetime.now(timezone.utc) - timedelta(days=day_offset)).replace(hour=0, minute=0, second=0, microsecond=0)
    conn = sqlite3.connect(db_path)
    conn.execute(
        "INSERT OR REPLACE INTO vitals_daily "
        "(day_start, hr_avg, hr_min, hr_max, hr_count, br_avg, br_min, br_max, br_count, "
        "lux_avg, dominant_state, resting_minutes) "
        "VALUES (?, ?, ?, ?, 100, ?, ?, ?, 100, 50.0, 'RESTING_VITALS', 120.0)",
        (day.isoformat(), hr_avg, hr_avg - 5, hr_avg + 5, br_avg, br_avg - 2, br_avg + 2),
    )
    conn.commit()
    conn.close()


class TestVitalsTrendsTool:
    @pytest.mark.asyncio
    async def test_returns_summary(self, store: VitalsStore, analyzer: TrendAnalyzer, db_path: Path) -> None:
        for d in range(1, 8):
            _insert_daily(db_path, d, hr_avg=72.0, br_avg=16.0)

        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.vitals_trends import (
            VitalsTrendsTool,
        )

        tool = VitalsTrendsTool()
        deps = MagicMock()
        deps.trend_analyzer = analyzer
        result = await tool(deps, days=7)
        assert result["status"] == "ok"
        assert "summary" in result
        assert result["summary"]["days_of_data"] >= 1

    @pytest.mark.asyncio
    async def test_returns_ok_without_data(self, store: VitalsStore, analyzer: TrendAnalyzer) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.vitals_trends import (
            VitalsTrendsTool,
        )

        tool = VitalsTrendsTool()
        deps = MagicMock()
        deps.trend_analyzer = analyzer
        result = await tool(deps, days=7)
        assert result["status"] == "ok"
        assert result["summary"]["days_of_data"] == 0

    @pytest.mark.asyncio
    async def test_no_analyzer_returns_error(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.vitals_trends import (
            VitalsTrendsTool,
        )

        tool = VitalsTrendsTool()
        deps = MagicMock()
        deps.trend_analyzer = None
        result = await tool(deps)
        assert "error" in result

    @pytest.mark.asyncio
    async def test_includes_recent_insights(self, store: VitalsStore, db_path: Path) -> None:
        analyzer = TrendAnalyzer(db_path=db_path, cooldown_s=0)
        for d in range(1, 8):
            _insert_daily(db_path, d, hr_avg=70.0, br_avg=16.0)
        # Vary one day to get stddev > 0
        _insert_daily(db_path, 3, hr_avg=72.0, br_avg=16.0)
        analyzer.analyze_session(current_hr=105.0, current_br=16.0)

        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.vitals_trends import (
            VitalsTrendsTool,
        )

        tool = VitalsTrendsTool()
        deps = MagicMock()
        deps.trend_analyzer = analyzer
        result = await tool(deps, days=7)
        assert len(result.get("recent_insights", [])) >= 1
