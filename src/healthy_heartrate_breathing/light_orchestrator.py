"""Light-context orchestration: lux delta tracking, analytics, auto-dispatch.

Reframed from ambient-light baseline tracking to proximity/occlusion context.
The BH1750 sensor sits behind the user — low lux means someone is close,
not that the room is dark.
"""

from __future__ import annotations
import json
import logging
import sqlite3
from typing import Any, Callable, Awaitable
from pathlib import Path
from datetime import datetime, timezone, timedelta

from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result


logger = logging.getLogger(__name__)

_ANALYTICS_SCHEMA_VERSION = 2


class LightOrchestrator:
    """Lux delta tracker and auto light_context dispatcher.

    Owns lux change detection and analytics. Dispatch is done via a callback,
    so this class is testable without OpenAI connections.
    """

    def __init__(  # noqa: D107
        self,
        *,
        enabled: bool = True,
        analytics_enabled: bool = False,
        user_id: str = "default",
        low_lux_threshold: float = 40.0,
        analytics_path: Path | None = None,
        analytics_max_age_days: int = 90,
    ) -> None:
        self.enabled = enabled
        self.analytics_enabled = analytics_enabled
        self.user_id = user_id
        self.low_lux_threshold = low_lux_threshold
        self.analytics_path = analytics_path
        self.analytics_max_age_days = analytics_max_age_days
        self._analytics_db_initialized: bool = False
        self._analytics_insert_count: int = 0

        # Mutable state
        self._last_lux: float | None = None
        self._last_lux_time: float | None = None

    # ---- Lux tracking ----

    def compute_lux_delta_60s(self, lux: float, now: float) -> float | None:
        """Estimate lux delta normalized to a 60s window."""
        if self._last_lux is None or self._last_lux_time is None:
            return None
        dt = now - self._last_lux_time
        if dt < 1.0 or dt > 600.0:
            return None
        raw_delta = lux - self._last_lux
        return float(raw_delta * (60.0 / max(1.0, dt)))

    # ---- Analytics ----

    def _init_analytics_db(self) -> None:
        """Create analytics SQLite schema and enable WAL mode (idempotent).

        Uses PRAGMA user_version to detect schema mismatches. If the existing
        DB has an older schema version, the table is dropped and recreated.
        """
        if self._analytics_db_initialized or self.analytics_path is None:
            return
        try:
            self.analytics_path.parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(self.analytics_path)
            conn.execute("PRAGMA journal_mode=WAL")

            # Check schema version
            (current_version,) = conn.execute("PRAGMA user_version").fetchone()
            if current_version != _ANALYTICS_SCHEMA_VERSION:
                conn.execute("DROP TABLE IF EXISTS light_events")
                conn.execute("DROP INDEX IF EXISTS idx_light_events_timestamp")
                conn.execute("DROP INDEX IF EXISTS idx_light_events_user_id")

            conn.executescript(f"""
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
                    obs_presence_detected      INTEGER,
                    obs_target_distance_cm     REAL
                );
                CREATE INDEX IF NOT EXISTS idx_light_events_timestamp ON light_events(timestamp);
                CREATE INDEX IF NOT EXISTS idx_light_events_user_id ON light_events(user_id);
                PRAGMA user_version={_ANALYTICS_SCHEMA_VERSION};
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
                    obs_lux_delta_60s, obs_presence_detected, obs_target_distance_cm
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
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
                    int(obs["presence_detected"])
                    if "presence_detected" in obs and obs["presence_detected"] is not None
                    else None,
                    obs.get("target_distance_cm"),
                ),
            )
            conn.commit()
            conn.close()
            self._analytics_insert_count += 1
            if self._analytics_insert_count % 100 == 0:
                self._prune_analytics()
        except Exception as e:
            logger.warning("Failed writing analytics event to %s: %s", self.analytics_path, e)

    # ---- Core orchestration ----

    async def run_from_mmwave(
        self,
        mmwave_result: dict[str, Any],
        *,
        is_idle: bool,
        has_tool: bool,
        dispatch_fn: Callable[[str, str], Awaitable[dict[str, Any]]],
    ) -> dict[str, Any] | None:
        """Auto-run light_context after mmWave when lux data is available.

        Args:
            mmwave_result: Raw mmWave tool result dict.
            is_idle: Whether this was an idle (background) tool call.
            has_tool: Whether the light_context tool is available.
            dispatch_fn: Callable(tool_name, args_json) -> result dict.

        """
        if not self.enabled:
            return None
        if not has_tool:
            return None

        lux = extract_lux_from_mmwave_result(mmwave_result)
        if lux is None:
            return None

        import asyncio

        now = asyncio.get_event_loop().time()
        lux_delta_60s = self.compute_lux_delta_60s(lux, now)

        # Detect presence from mmWave result
        has_target = False
        target_distance_cm: float | None = None
        scan = mmwave_result.get("scan")
        if isinstance(scan, dict):
            latest_target = scan.get("latest_target")
            if isinstance(latest_target, dict):
                has_target = True
                r = latest_target.get("distance_cm")
                if isinstance(r, (int, float)):
                    target_distance_cm = float(r)
            recent = scan.get("recent_targets")
            if isinstance(recent, list) and len(recent) > 0:
                has_target = True
        measure = mmwave_result.get("measure")
        if isinstance(measure, dict) and bool(measure.get("success")):
            has_target = True

        args: dict[str, Any] = {
            "lux": lux,
            "previous_lux": self._last_lux,
            "lux_delta_60s": lux_delta_60s,
            "presence_detected": has_target,
            "mmwave_result": mmwave_result,
        }
        if target_distance_cm is not None:
            args["target_distance_cm"] = target_distance_cm

        result = await dispatch_fn("light_context", json.dumps(args))
        if isinstance(result, dict):
            self._append_analytics_event(source_tool="mmWave", lux=lux, result=result)

        self._last_lux = lux
        self._last_lux_time = now
        return result if isinstance(result, dict) else None
