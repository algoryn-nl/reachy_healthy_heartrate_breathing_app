"""Health trend analysis with anomaly detection and summaries."""

from __future__ import annotations
import time
import logging
import sqlite3
import statistics
from typing import Any
from pathlib import Path
from datetime import datetime, timezone, timedelta
from dataclasses import dataclass


logger = logging.getLogger(__name__)


@dataclass
class TrendInsight:
    """A single anomaly or notable trend detected in vitals data."""

    category: str
    message: str
    severity: str
    current_value: float
    baseline_mean: float
    baseline_stddev: float


@dataclass
class TrendSummary:
    """Aggregated vitals summary over a time window."""

    days_of_data: int
    hr_avg_7d: float | None
    br_avg_7d: float | None
    hr_trend: str
    br_trend: str
    resting_minutes_avg: float | None
    daily_series: list[dict[str, Any]]


class TrendAnalyzer:
    """Analyzes vitals trends, detects anomalies, and generates summaries."""

    def __init__(
        self,
        *,
        db_path: Path,
        deviation_threshold: float = 1.5,
        hr_high: float = 100.0,
        hr_low: float = 45.0,
        br_high: float = 25.0,
        br_low: float = 6.0,
        cooldown_s: float = 900.0,
    ) -> None:
        """Initialize with database path, thresholds, and cooldown."""
        self._db_path = db_path
        self._deviation_threshold = deviation_threshold
        self._hr_high = hr_high
        self._hr_low = hr_low
        self._br_high = br_high
        self._br_low = br_low
        self._cooldown_s = cooldown_s
        self._last_insight_time: float = 0.0

    def analyze_session(self, current_hr: float, current_br: float) -> TrendInsight | None:
        """Check current vitals against historical baseline and return an insight if anomalous."""
        now = time.monotonic()
        if self._cooldown_s > 0 and self._last_insight_time > 0 and (now - self._last_insight_time) < self._cooldown_s:
            return None

        daily = self._get_daily_stats(days=7)
        if len(daily) < 2:
            return None

        insight = self._check_anomalies(current_hr, current_br, daily)
        if insight is not None:
            self._persist_insight(insight)
            self._last_insight_time = time.monotonic()
        return insight

    def get_summary(self, days: int = 7) -> TrendSummary:
        """Compute an aggregated summary over the given number of days."""
        daily = self._get_daily_stats(days=days)
        if not daily:
            return TrendSummary(
                days_of_data=0,
                hr_avg_7d=None,
                br_avg_7d=None,
                hr_trend="stable",
                br_trend="stable",
                resting_minutes_avg=None,
                daily_series=[],
            )

        hr_values = [r["hr_avg"] for r in daily if r["hr_avg"] is not None]
        br_values = [r["br_avg"] for r in daily if r["br_avg"] is not None]
        resting = [r["resting_minutes"] for r in daily if r["resting_minutes"] is not None]

        return TrendSummary(
            days_of_data=len(daily),
            hr_avg_7d=statistics.mean(hr_values) if hr_values else None,
            br_avg_7d=statistics.mean(br_values) if br_values else None,
            hr_trend=self._detect_trend(hr_values),
            br_trend=self._detect_trend(br_values),
            resting_minutes_avg=statistics.mean(resting) if resting else None,
            daily_series=daily,
        )

    def get_daily_series(self, days: int = 7) -> list[dict[str, Any]]:
        """Return raw daily aggregation rows."""
        return self._get_daily_stats(days=days)

    def recent_insights(self, limit: int = 5) -> list[dict[str, Any]]:
        """Return the most recent trend insights from the database."""
        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                "SELECT timestamp, category, message, severity FROM trend_insights ORDER BY timestamp DESC LIMIT ?",
                (limit,),
            ).fetchall()
            conn.close()
            return [dict(r) for r in rows]
        except Exception:
            logger.warning("Failed to query trend insights", exc_info=True)
            return []

    def _check_anomalies(
        self, current_hr: float, current_br: float, daily: list[dict[str, Any]]
    ) -> TrendInsight | None:
        hr_values = [r["hr_avg"] for r in daily if r["hr_avg"] is not None]
        br_values = [r["br_avg"] for r in daily if r["br_avg"] is not None]

        hr_mean = statistics.mean(hr_values) if hr_values else 0.0
        br_mean = statistics.mean(br_values) if br_values else 0.0
        hr_std = statistics.stdev(hr_values) if len(hr_values) >= 2 else 0.0
        br_std = statistics.stdev(br_values) if len(br_values) >= 2 else 0.0

        if current_hr > self._hr_high:
            return TrendInsight(
                category="elevated_hr",
                message=f"Heart rate {current_hr:.0f} bpm exceeds safe threshold ({self._hr_high:.0f}).",
                severity="attention",
                current_value=current_hr,
                baseline_mean=hr_mean,
                baseline_stddev=hr_std,
            )
        if current_hr < self._hr_low:
            return TrendInsight(
                category="low_hr",
                message=f"Heart rate {current_hr:.0f} bpm is below safe threshold ({self._hr_low:.0f}).",
                severity="attention",
                current_value=current_hr,
                baseline_mean=hr_mean,
                baseline_stddev=hr_std,
            )
        if current_br > self._br_high:
            return TrendInsight(
                category="elevated_br",
                message=f"Breathing rate {current_br:.0f} bpm exceeds safe threshold ({self._br_high:.0f}).",
                severity="attention",
                current_value=current_br,
                baseline_mean=br_mean,
                baseline_stddev=br_std,
            )
        if current_br < self._br_low:
            return TrendInsight(
                category="low_br",
                message=f"Breathing rate {current_br:.0f} bpm is below safe threshold ({self._br_low:.0f}).",
                severity="attention",
                current_value=current_br,
                baseline_mean=br_mean,
                baseline_stddev=br_std,
            )

        if hr_std > 0 and abs(current_hr - hr_mean) > self._deviation_threshold * hr_std:
            direction = "higher" if current_hr > hr_mean else "lower"
            return TrendInsight(
                category=f"unusual_hr_{direction}",
                message=(
                    f"Heart rate {current_hr:.0f} bpm is {direction} than your 7-day average "
                    f"({hr_mean:.0f} +/- {hr_std:.1f})."
                ),
                severity="info",
                current_value=current_hr,
                baseline_mean=hr_mean,
                baseline_stddev=hr_std,
            )
        if br_std > 0 and abs(current_br - br_mean) > self._deviation_threshold * br_std:
            direction = "higher" if current_br > br_mean else "lower"
            return TrendInsight(
                category=f"unusual_br_{direction}",
                message=(
                    f"Breathing rate {current_br:.0f} bpm is {direction} than your 7-day average "
                    f"({br_mean:.0f} +/- {br_std:.1f})."
                ),
                severity="info",
                current_value=current_br,
                baseline_mean=br_mean,
                baseline_stddev=br_std,
            )

        return None

    def _detect_trend(self, values: list[float]) -> str:
        if len(values) < 4:
            return "stable"
        mid = len(values) // 2
        first_half = statistics.mean(values[:mid])
        second_half = statistics.mean(values[mid:])
        if first_half == 0:
            return "stable"
        change = (second_half - first_half) / first_half
        if change > 0.05:
            return "increasing"
        if change < -0.05:
            return "decreasing"
        return "stable"

    def _persist_insight(self, insight: TrendInsight) -> None:
        try:
            conn = sqlite3.connect(self._db_path)
            conn.execute(
                "INSERT INTO trend_insights (timestamp, category, message, severity) VALUES (?, ?, ?, ?)",
                (datetime.now(timezone.utc).isoformat(), insight.category, insight.message, insight.severity),
            )
            conn.commit()
            conn.close()
        except Exception:
            logger.warning("Failed to persist trend insight", exc_info=True)

    def _get_daily_stats(self, days: int) -> list[dict[str, Any]]:
        cutoff = (datetime.now(timezone.utc) - timedelta(days=days)).isoformat()
        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                "SELECT day_start, hr_avg, hr_min, hr_max, hr_count, br_avg, br_min, br_max, br_count, "
                "lux_avg, dominant_state, resting_minutes "
                "FROM vitals_daily WHERE day_start >= ? ORDER BY day_start ASC",
                (cutoff,),
            ).fetchall()
            conn.close()
            return [dict(r) for r in rows]
        except Exception:
            logger.warning("Failed to query daily stats", exc_info=True)
            return []
