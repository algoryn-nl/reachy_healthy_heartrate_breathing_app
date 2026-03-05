"""Periodic rollup of raw vitals into hourly and daily aggregates."""

from __future__ import annotations
import logging
import sqlite3
from pathlib import Path
from datetime import datetime, timezone, timedelta


logger = logging.getLogger(__name__)


class VitalsAggregator:
    """Roll up raw vitals_history rows into vitals_hourly and vitals_daily."""

    def __init__(
        self,
        *,
        db_path: Path,
        hourly_retention_days: int = 30,
        daily_retention_days: int = 90,
    ) -> None:
        """Initialize aggregator with database path and retention windows."""
        self._db_path = db_path
        self._hourly_retention_days = hourly_retention_days
        self._daily_retention_days = daily_retention_days
        self._last_rollup_hour: str | None = None

    def rollup_hourly(self) -> None:
        """Aggregate completed hours from vitals_history into vitals_hourly."""
        current_hour = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:00:00+00:00")

        # Skip if we already rolled up this hour (in-memory guard).
        if self._last_rollup_hour == current_hour:
            return

        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row

            # Find completed hours with data that haven't been aggregated yet.
            hours = conn.execute(
                "SELECT DISTINCT strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) AS hour_start "
                "FROM vitals_history "
                "WHERE strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) < ? "
                "AND strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) NOT IN "
                "    (SELECT hour_start FROM vitals_hourly)",
                (current_hour,),
            ).fetchall()

            for hour_row in hours:
                hs = hour_row["hour_start"]

                # Compute vitals averages (NULL-safe: COUNT(col) excludes NULLs).
                stats = conn.execute(
                    "SELECT "
                    "  AVG(heart_rate_bpm) AS hr_avg, "
                    "  MIN(heart_rate_bpm) AS hr_min, "
                    "  MAX(heart_rate_bpm) AS hr_max, "
                    "  COUNT(heart_rate_bpm) AS hr_count, "
                    "  AVG(breath_rate_bpm) AS br_avg, "
                    "  MIN(breath_rate_bpm) AS br_min, "
                    "  MAX(breath_rate_bpm) AS br_max, "
                    "  COUNT(breath_rate_bpm) AS br_count, "
                    "  AVG(lux) AS lux_avg, "
                    "  COUNT(*) AS total_count "
                    "FROM vitals_history "
                    "WHERE strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) = ?",
                    (hs,),
                ).fetchone()

                # Dominant state: most frequent device_state in the hour.
                dom_row = conn.execute(
                    "SELECT device_state FROM vitals_history "
                    "WHERE strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) = ? "
                    "AND device_state IS NOT NULL "
                    "GROUP BY device_state ORDER BY COUNT(*) DESC LIMIT 1",
                    (hs,),
                ).fetchone()
                dominant_state = dom_row["device_state"] if dom_row else None

                # Resting minutes: proportion of RESTING_VITALS rows * 60 minutes.
                total_count = stats["total_count"] or 0
                resting_count = 0
                if total_count > 0:
                    rc = conn.execute(
                        "SELECT COUNT(*) AS c FROM vitals_history "
                        "WHERE strftime('%Y-%m-%dT%H:00:00+00:00', timestamp) = ? "
                        "AND device_state = 'RESTING_VITALS'",
                        (hs,),
                    ).fetchone()
                    resting_count = rc["c"] if rc else 0
                resting_minutes = (resting_count / total_count * 60.0) if total_count > 0 else 0.0

                conn.execute(
                    "INSERT OR IGNORE INTO vitals_hourly "
                    "(hour_start, hr_avg, hr_min, hr_max, hr_count, "
                    "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (
                        hs,
                        stats["hr_avg"],
                        stats["hr_min"],
                        stats["hr_max"],
                        stats["hr_count"],
                        stats["br_avg"],
                        stats["br_min"],
                        stats["br_max"],
                        stats["br_count"],
                        stats["lux_avg"],
                        dominant_state,
                        resting_minutes,
                    ),
                )

            conn.commit()
            conn.close()
            self._last_rollup_hour = current_hour
        except Exception:
            logger.warning("Failed to rollup hourly vitals", exc_info=True)

    def rollup_daily(self) -> None:
        """Aggregate completed days from vitals_hourly into vitals_daily."""
        today = datetime.now(timezone.utc).strftime("%Y-%m-%d")

        try:
            conn = sqlite3.connect(self._db_path)
            conn.row_factory = sqlite3.Row

            # Find completed days with hourly data not yet aggregated.
            days = conn.execute(
                "SELECT DISTINCT substr(hour_start, 1, 10) AS day_start "
                "FROM vitals_hourly "
                "WHERE substr(hour_start, 1, 10) < ? "
                "AND substr(hour_start, 1, 10) NOT IN "
                "    (SELECT day_start FROM vitals_daily)",
                (today,),
            ).fetchall()

            for day_row in days:
                ds = day_row["day_start"]

                # Weighted averages from hourly rows.
                stats = conn.execute(
                    "SELECT "
                    "  SUM(hr_avg * hr_count) / NULLIF(SUM(hr_count), 0) AS hr_avg, "
                    "  MIN(hr_min) AS hr_min, "
                    "  MAX(hr_max) AS hr_max, "
                    "  SUM(hr_count) AS hr_count, "
                    "  SUM(br_avg * br_count) / NULLIF(SUM(br_count), 0) AS br_avg, "
                    "  MIN(br_min) AS br_min, "
                    "  MAX(br_max) AS br_max, "
                    "  SUM(br_count) AS br_count, "
                    "  AVG(lux_avg) AS lux_avg, "
                    "  SUM(resting_minutes) AS resting_minutes "
                    "FROM vitals_hourly "
                    "WHERE substr(hour_start, 1, 10) = ?",
                    (ds,),
                ).fetchone()

                # Dominant state: hourly row with most samples.
                dom_row = conn.execute(
                    "SELECT dominant_state FROM vitals_hourly "
                    "WHERE substr(hour_start, 1, 10) = ? "
                    "AND dominant_state IS NOT NULL "
                    "ORDER BY hr_count DESC LIMIT 1",
                    (ds,),
                ).fetchone()
                dominant_state = dom_row["dominant_state"] if dom_row else None

                conn.execute(
                    "INSERT OR IGNORE INTO vitals_daily "
                    "(day_start, hr_avg, hr_min, hr_max, hr_count, "
                    "br_avg, br_min, br_max, br_count, lux_avg, dominant_state, resting_minutes) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (
                        ds,
                        stats["hr_avg"],
                        stats["hr_min"],
                        stats["hr_max"],
                        stats["hr_count"],
                        stats["br_avg"],
                        stats["br_min"],
                        stats["br_max"],
                        stats["br_count"],
                        stats["lux_avg"],
                        dominant_state,
                        stats["resting_minutes"],
                    ),
                )

            conn.commit()
            conn.close()
        except Exception:
            logger.warning("Failed to rollup daily vitals", exc_info=True)

    def prune(self) -> None:
        """Remove old aggregate rows and trim trend_insights to 50."""
        try:
            conn = sqlite3.connect(self._db_path)

            # Prune hourly rows.
            hourly_cutoff = (datetime.now(timezone.utc) - timedelta(days=self._hourly_retention_days)).strftime(
                "%Y-%m-%dT%H:00:00+00:00"
            )
            conn.execute("DELETE FROM vitals_hourly WHERE hour_start < ?", (hourly_cutoff,))

            # Prune daily rows.
            daily_cutoff = (datetime.now(timezone.utc) - timedelta(days=self._daily_retention_days)).strftime(
                "%Y-%m-%d"
            )
            conn.execute("DELETE FROM vitals_daily WHERE day_start < ?", (daily_cutoff,))

            # Trim trend_insights to most recent 50.
            conn.execute(
                "DELETE FROM trend_insights WHERE id NOT IN "
                "(SELECT id FROM trend_insights ORDER BY timestamp DESC LIMIT 50)"
            )

            conn.commit()
            conn.close()
        except Exception:
            logger.warning("Failed to prune aggregates", exc_info=True)

    def rollup(self) -> None:
        """Run hourly rollup, daily rollup, and prune in sequence."""
        self.rollup_hourly()
        self.rollup_daily()
        self.prune()
