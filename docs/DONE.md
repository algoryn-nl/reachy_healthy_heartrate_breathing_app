# Completed Work Archive

For detailed descriptions of each item, see `docs/20260223_roadmap.md` (Resolved section + Phase 1–4 + Future Vision > Completed).

## Features (2026-03-05)

- **PY-FEAT-1**: Gradio sensor dashboard (vitals card, radar canvas, vitals history, WebSocket push, VitalsStore)
- **PY-FEAT-2**: Health trend analysis (two-tier aggregation, TrendAnalyzer, anomaly detection, dashboard trends panel, `vitals_trends` tool)
- **PY-REFACTOR-1**: Reframe BH1750 lux from ambient light to proximity/occlusion
- **FW-FEAT-1**: CMD_RESET mmWave radar recovery (0x07, ACK/ERR, Python auto-recovery)
- **FW-REFACTOR-1**: Decouple state machine from sensor polling (poll/tick, 10 Hz fixed tick)

## Phase 1: Critical Fixes (2026-02-28)

- **FW-CRIT-1**: Fix vitals hysteresis bypass in `emitBio()`
- **FW-CRIT-2**: Add expiry to cached vitals
- **FW-CRIT-3**: Fix race condition in presence detection
- **PY-CRIT-1**: Replace `sys.exit()` with `ConfigError` exceptions
- **PY-CRIT-2**: Add path traversal validation
- **PY-CRIT-3**: Fix falsy-zero threshold bug

## Phase 2: Stability & Thread Safety (2026-02-28 – 2026-03-05)

- **PY-HIGH-1** – **PY-HIGH-6**: Audio decode guard, registry locking, light_context timeout, HeadWobbler TOCTOU, MovementManager thread leak, type annotation fix
- **FW-HIGH-1** – **FW-HIGH-3**: COBS overflow counter, sensor error telemetry, EVT_STATE rate-limit

## Phase 3: Code Quality (2026-03-01 – 2026-03-05)

- **PY-MED-1** – **PY-MED-10**: REST cleanup, duplicate assignments, deferred init, float equality, private queue, event-based API key, launch decomposition, SQLite analytics, platform-aware globs, tool error standardization
- **FW-MED-1** – **FW-MED-5**: BH1750 re-init, struct encapsulation, buffer sizing, CMD_SET_GUARD_RAILS, guessPose hysteresis
- **PY-LOW-1** – **PY-LOW-5**: IdlePolicy validation, safe_load_obj logging, sys.modules cleanup, sweep_look error handling, endpoint consolidation
- **FW-LOW-1** – **FW-LOW-2**: FRAME_DELIMITER constant, firmware unit test harness (41 tests)

## Test Coverage Milestones

- 551 total tests as of 2026-03-05
- AudioRouter, TranscriptHandler, ToolDispatcher, IdlePolicy, LightOrchestrator, HeadWobbler, MovementManager, mmWave protocol, openai_realtime, VitalsStore, SensorBroadcaster, path traversal security, firmware codec cross-validation

## Infrastructure (2026-02-23 – 2026-02-24)

- Handler decomposition (5 classes from `_run_realtime_session()`)
- Non-blocking tool dispatch (asyncio.create_task + Semaphore + wait_for)
- Protocol version handshake rejection
- Graceful mmWave disconnection handling
- Multi-person tracking (IdlePolicy multi-target awareness)
- Device state context (`build_device_context()`)
- Headless sensor dashboard
- env_utils.py consolidation
