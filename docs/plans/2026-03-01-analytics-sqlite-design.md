# Analytics Storage: JSONL to SQLite Migration

**Date**: 2026-03-01
**Status**: Approved
**Replaces**: PY-MED-8 JSONL rotation (commit 9c7985c)

## Problem

JSONL analytics in `LightOrchestrator` has three issues:
1. **Corruption risk** — crash mid-write leaves partial JSON lines
2. **Query difficulty** — no way to slice, filter, or aggregate without external tools
3. **Disk inefficiency** — field names repeated on every line

In-app queries are a desired use case, ruling out flat-file formats.

## Decision

Replace JSONL with SQLite (`sqlite3` stdlib). Single `.db` file with one fully flattened table. Time-based retention replaces file-size rotation.

## Schema

```sql
CREATE TABLE IF NOT EXISTS light_events (
    id                         INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp                  TEXT    NOT NULL,  -- ISO 8601 UTC
    user_id                    TEXT    NOT NULL,
    source_tool                TEXT    NOT NULL,
    context_state              TEXT,
    recommended_mode           TEXT,
    recommended_actions        TEXT,   -- comma-separated tags
    confidence                 REAL,
    cooldown_hint_s            REAL,
    reason_codes               TEXT,   -- comma-separated tags
    lux                        REAL,
    obs_lux_delta_60s          REAL,
    obs_local_hour             INTEGER,
    obs_is_night               INTEGER,  -- 0/1
    obs_presence_detected      INTEGER,  -- 0/1
    obs_active_interaction     INTEGER,  -- 0/1
    obs_low_light_duration_min REAL,
    obs_prefers_dim            INTEGER,  -- 0/1
    obs_light_sensitive        INTEGER,  -- 0/1
    obs_allow_wellness_nudges  INTEGER   -- 0/1
);

CREATE INDEX IF NOT EXISTS idx_light_events_timestamp ON light_events(timestamp);
CREATE INDEX IF NOT EXISTS idx_light_events_user_id ON light_events(user_id);
```

Array fields (`recommended_actions`, `reason_codes`) stored as comma-separated TEXT. Observations dict flattened to `obs_*` columns. No JSON-in-TEXT blobs.

## API Changes

### LightOrchestrator

- **Remove**: `_maybe_rotate_analytics()`, `analytics_max_bytes` parameter
- **Add**: `analytics_max_age_days: int = 90` parameter
- **Add**: `_init_analytics_db()` — lazy schema creation + WAL mode on first write
- **Add**: `_prune_analytics()` — `DELETE WHERE timestamp < cutoff`, called on init and every 100 inserts
- **Change**: `_append_analytics_event()` — `INSERT` instead of file append
- **Change**: `analytics_path` points to `.db` instead of `.jsonl`

### Connection Management

Open/close per write (no persistent connection). Writes are infrequent (~1 per mmWave probe, every 15-40s). WAL journal mode set on init for crash safety.

### Error Handling

Same pattern as current: `try/except` around all DB operations, log warning, never raise.

## Environment Variables

- **Remove**: `HEALTHY_LIGHT_ANALYTICS_MAX_BYTES`
- **Add**: `HEALTHY_LIGHT_ANALYTICS_MAX_AGE_DAYS` (default 90)

## Testing

Replace 4 JSONL rotation tests with ~5 SQLite tests:
1. Event inserted and queryable
2. Retention prunes old rows
3. DB created lazily on first write
4. Disabled analytics doesn't create DB
5. DB write failure doesn't raise

## Migration

No migration needed. Analytics is write-only diagnostic data (not user state). Old `.jsonl` files can be ignored or manually deleted.
