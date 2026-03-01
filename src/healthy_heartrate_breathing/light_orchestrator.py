"""Light-context orchestration: baseline tracking, analytics, auto-dispatch."""

from __future__ import annotations
import json
import sqlite3
import time
import logging
from typing import Any, Callable, Awaitable
from pathlib import Path
from datetime import datetime, timezone, timedelta

from healthy_heartrate_breathing.env_utils import extract_lux_from_mmwave_result


logger = logging.getLogger(__name__)


class LightOrchestrator:
    """Standalone lux baseline tracker and auto light_context dispatcher.

    Owns all light-related state previously spread across OpenaiRealtimeHandler.
    Testable without OpenAI connections — dispatch is done via a callback.
    """

    def __init__(  # noqa: D107
        self,
        *,
        enabled: bool = True,
        analytics_enabled: bool = False,
        user_id: str = "default",
        prefers_dim: bool = False,
        light_sensitive: bool = False,
        allow_wellness_nudges: bool = True,
        day_start_hour: int = 7,
        night_start_hour: int = 20,
        low_lux_threshold: float = 40.0,
        baseline_alpha: float = 0.15,
        baseline_min_samples: int = 5,
        baseline_path: Path | None = None,
        analytics_path: Path | None = None,
        baseline_save_interval_s: float = 60.0,
        baseline_max_age_days: int = 90,
        analytics_max_age_days: int = 90,
    ) -> None:
        self.enabled = enabled
        self.analytics_enabled = analytics_enabled
        self.user_id = user_id
        self.prefers_dim = prefers_dim
        self.light_sensitive = light_sensitive
        self.allow_wellness_nudges = allow_wellness_nudges
        self.day_start_hour = day_start_hour
        self.night_start_hour = night_start_hour
        self.low_lux_threshold = low_lux_threshold
        self.baseline_alpha = baseline_alpha
        self.baseline_min_samples = baseline_min_samples
        self.baseline_path = baseline_path
        self.analytics_path = analytics_path
        self.baseline_save_interval_s = baseline_save_interval_s
        self.baseline_max_age_days = baseline_max_age_days
        self.analytics_max_age_days = analytics_max_age_days
        self._analytics_db_initialized: bool = False
        self._analytics_insert_count: int = 0

        # Mutable state
        self._baseline_state: dict[str, Any] = {"schema_version": 1, "users": {}}
        self._last_lux: float | None = None
        self._last_lux_time: float | None = None
        self._low_since_time: float | None = None
        self._baseline_dirty: bool = False
        self._last_save_time: float = 0.0

        if self.baseline_path is not None:
            self.load_baseline()

    # ---- Baseline persistence ----

    def load_baseline(self) -> None:
        """Load baseline JSON state from disk with safe fallback."""
        default: dict[str, Any] = {"schema_version": 1, "users": {}}
        if self.baseline_path is None:
            self._baseline_state = default
            return
        try:
            if not self.baseline_path.exists():
                self._baseline_state = default
                return
            parsed = json.loads(self.baseline_path.read_text(encoding="utf-8"))
            if not isinstance(parsed, dict):
                self._baseline_state = default
                return
            users = parsed.get("users")
            if not isinstance(users, dict):
                parsed["users"] = {}
            self._baseline_state = parsed
            if self._prune_stale_users():
                self._baseline_dirty = True
                self.save_baseline()
        except Exception as e:
            logger.warning("Failed loading light baseline from %s: %s", self.baseline_path, e)
            self._baseline_state = default

    def save_baseline(self) -> None:
        """Persist baseline JSON state to disk if dirty and interval has elapsed."""
        if self.baseline_path is None:
            return
        if not self._baseline_dirty:
            return
        now = time.monotonic()
        if now - self._last_save_time < self.baseline_save_interval_s:
            return
        self._write_baseline(now)

    def flush(self) -> None:
        """Force-write dirty baseline to disk (call at shutdown)."""
        if self.baseline_path is None or not self._baseline_dirty:
            return
        self._write_baseline(time.monotonic())

    def _write_baseline(self, now: float) -> None:
        """Write baseline state to disk unconditionally."""
        assert self.baseline_path is not None
        try:
            self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
            self.baseline_path.write_text(
                json.dumps(self._baseline_state, ensure_ascii=True, indent=2),
                encoding="utf-8",
            )
            self._baseline_dirty = False
            self._last_save_time = now
        except Exception as e:
            logger.warning("Failed writing light baseline to %s: %s", self.baseline_path, e)

    def _prune_stale_users(self) -> bool:
        """Remove user entries older than baseline_max_age_days. Return True if any removed."""
        users = self._baseline_state.get("users")
        if not isinstance(users, dict) or not users:
            return False
        cutoff = datetime.now(timezone.utc) - timedelta(days=self.baseline_max_age_days)
        stale = []
        for uid, entry in users.items():
            if not isinstance(entry, dict):
                stale.append(uid)
                continue
            ts = entry.get("updated_at")
            if not isinstance(ts, str):
                stale.append(uid)
                continue
            try:
                updated = datetime.fromisoformat(ts)
                if updated < cutoff:
                    stale.append(uid)
            except (ValueError, TypeError):
                stale.append(uid)
        for uid in stale:
            del users[uid]
            logger.info("Pruned stale light baseline for user %r", uid)
        return len(stale) > 0

    # ---- Time helpers ----

    def is_daytime_hour(self, hour: int) -> bool:
        """Return True for daytime hours using configured day/night cutoffs."""
        if self.day_start_hour == self.night_start_hour:
            return True
        if self.day_start_hour < self.night_start_hour:
            return self.day_start_hour <= hour < self.night_start_hour
        return hour >= self.day_start_hour or hour < self.night_start_hour

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

    def _get_or_create_user_entry(self) -> dict[str, Any]:
        """Get mutable baseline entry for the configured user id."""
        users = self._baseline_state.setdefault("users", {})
        if not isinstance(users, dict):
            users = {}
            self._baseline_state["users"] = users
        entry = users.get(self.user_id)
        if not isinstance(entry, dict):
            entry = {"hours": {}, "updated_at": None}
            users[self.user_id] = entry
        hours = entry.get("hours")
        if not isinstance(hours, dict):
            entry["hours"] = {}
        return entry

    def update_baseline(self, lux: float, local_hour: int) -> None:
        """Update per-user rolling lux baseline for daytime behavior."""
        if not self.is_daytime_hour(local_hour):
            return
        entry = self._get_or_create_user_entry()
        hours: dict[str, Any] = entry["hours"]
        bucket_key = str(local_hour)
        bucket = hours.get(bucket_key)
        if not isinstance(bucket, dict):
            bucket = {"ema_lux": None, "samples": 0}
            hours[bucket_key] = bucket

        prev_ema = bucket.get("ema_lux")
        prev_val = float(prev_ema) if isinstance(prev_ema, (int, float)) else None
        next_ema = lux if prev_val is None else (self.baseline_alpha * lux) + ((1.0 - self.baseline_alpha) * prev_val)
        samples = int(bucket.get("samples", 0)) + 1
        bucket["ema_lux"] = round(float(next_ema), 3)
        bucket["samples"] = samples
        entry["updated_at"] = datetime.now(timezone.utc).isoformat()
        self._baseline_dirty = True
        self.save_baseline()

    def get_typical_day_low_lux(self, local_hour: int) -> float | None:
        """Return a user baseline lux value for personalization."""
        entry = self._get_or_create_user_entry()
        hours = entry.get("hours")
        if not isinstance(hours, dict):
            return None

        bucket = hours.get(str(local_hour))
        if isinstance(bucket, dict):
            samples = int(bucket.get("samples", 0))
            ema = bucket.get("ema_lux")
            if samples >= self.baseline_min_samples and isinstance(ema, (int, float)):
                return float(ema)

        fallback_values: list[float] = []
        for hour_str, raw_bucket in hours.items():
            if not isinstance(raw_bucket, dict):
                continue
            try:
                hour = int(hour_str)
            except Exception:
                continue
            if not self.is_daytime_hour(hour):
                continue
            ema = raw_bucket.get("ema_lux")
            samples = int(raw_bucket.get("samples", 0))
            if samples > 0 and isinstance(ema, (int, float)):
                fallback_values.append(float(ema))
        if not fallback_values:
            return None
        return float(sum(fallback_values) / len(fallback_values))

    # ---- Analytics ----

    def _init_analytics_db(self) -> None:
        """Create analytics SQLite schema and enable WAL mode (idempotent)."""
        if self._analytics_db_initialized or self.analytics_path is None:
            return
        try:
            self.analytics_path.parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(self.analytics_path)
            conn.execute("PRAGMA journal_mode=WAL")
            conn.executescript("""
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
                    obs_local_hour             INTEGER,
                    obs_is_night               INTEGER,
                    obs_presence_detected      INTEGER,
                    obs_active_interaction     INTEGER,
                    obs_low_light_duration_min REAL,
                    obs_prefers_dim            INTEGER,
                    obs_light_sensitive        INTEGER,
                    obs_allow_wellness_nudges  INTEGER
                );
                CREATE INDEX IF NOT EXISTS idx_light_events_timestamp ON light_events(timestamp);
                CREATE INDEX IF NOT EXISTS idx_light_events_user_id ON light_events(user_id);
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
                    obs_lux_delta_60s, obs_local_hour, obs_is_night, obs_presence_detected,
                    obs_active_interaction, obs_low_light_duration_min, obs_prefers_dim,
                    obs_light_sensitive, obs_allow_wellness_nudges
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
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
                    obs.get("local_hour"),
                    int(obs["is_night"]) if "is_night" in obs else None,
                    int(obs["presence_detected"]) if "presence_detected" in obs else None,
                    int(obs["active_interaction"]) if "active_interaction" in obs else None,
                    obs.get("low_light_duration_min"),
                    int(obs["prefers_dim"]) if "prefers_dim" in obs else None,
                    int(obs["light_sensitive"]) if "light_sensitive" in obs else None,
                    int(obs["allow_wellness_nudges"]) if "allow_wellness_nudges" in obs else None,
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
        local_hour = datetime.now().hour
        lux_delta_60s = self.compute_lux_delta_60s(lux, now)

        if lux <= self.low_lux_threshold:
            if self._low_since_time is None:
                self._low_since_time = now
        else:
            self._low_since_time = None

        low_light_duration_min = ((now - self._low_since_time) / 60.0) if self._low_since_time is not None else 0.0
        user_typical = self.get_typical_day_low_lux(local_hour)

        # Detect presence from mmWave result
        has_target = False
        scan = mmwave_result.get("scan")
        if isinstance(scan, dict):
            if isinstance(scan.get("latest_target"), dict):
                has_target = True
            recent = scan.get("recent_targets")
            if isinstance(recent, list) and len(recent) > 0:
                has_target = True
        measure = mmwave_result.get("measure")
        if isinstance(measure, dict) and bool(measure.get("success")):
            has_target = True

        args = {
            "lux": lux,
            "previous_lux": self._last_lux,
            "lux_delta_60s": lux_delta_60s,
            "presence_detected": has_target,
            "active_interaction": not is_idle,
            "low_light_duration_min": low_light_duration_min,
            "local_hour": local_hour,
            "prefers_dim": self.prefers_dim,
            "light_sensitive": self.light_sensitive,
            "allow_wellness_nudges": self.allow_wellness_nudges,
            "user_typical_day_low_lux": user_typical,
            "mmwave_result": mmwave_result,
        }
        result = await dispatch_fn("light_context", json.dumps(args))
        if isinstance(result, dict):
            self._append_analytics_event(source_tool="mmWave", lux=lux, result=result)

        # Baseline update comes after decision so current reading does not bias its own classification.
        self.update_baseline(lux=lux, local_hour=local_hour)
        self._last_lux = lux
        self._last_lux_time = now
        return result if isinstance(result, dict) else None
