# Decouple State Machine from Sensor Polling Rate

## Goal

Run the firmware state machine at a fixed 10 Hz wall-clock frequency, independent of mmWave radar polling success, so that state transitions are predictable and the state machine never freezes when the radar misses frames.

## Problem

The state machine in `loop()` only runs when `mmWave.update()` returns a complete frame. Three frame-count thresholds (`VITALS_CONFIRM`, `ABSENT_CONFIRM`, `HUMAN_STABLE_FALLBACK_CONFIRM`) translate to variable wall-clock durations depending on the radar's actual frame rate (~3–10 Hz). When the radar fails, the state machine freezes entirely.

## Design

### New `SensorSnapshot` struct

Buffers the latest radar + lux readings. The poll phase writes it; the tick phase reads it.

```cpp
struct SensorSnapshot {
  uint32_t radarMs       = 0;     // millis() of last successful mmWave.update()
  bool     humanDetected = false;
  uint8_t  nTargets      = 0;
  TargetInfo targets[MAX_TARGETS_WIRE];
  float    dist_cm       = NAN;
  float    breathRate    = NAN;
  float    heartRate     = NAN;
  float    lux           = NAN;
  uint32_t luxMs         = 0;     // millis() of last lux read
  bool     radarFresh    = false; // set true on poll success, cleared after tick consumes it
};
```

### Loop restructure: poll + tick

```cpp
void loop() {
  uint32_t now = millis();

  // === POLL PHASE (every iteration) ===
  pollHostUsbSerial();
  pollRadar(now);        // mmWave.update() → writes SensorSnapshot
  pollLightSensor(now);  // BH1750 read, throttled to 1s
  emitDiagnostics(now);  // EVT_DIAG, throttled to 10s

  // === TICK PHASE (fixed 10 Hz) ===
  if (now - lastTickMs < TICK_INTERVAL_MS) return;
  lastTickMs = now;
  tickStateMachine(now); // reads SensorSnapshot, runs state logic, emits telemetry
}
```

- `pollRadar()` wraps `mmWave.update(100)`. On success, populates `SensorSnapshot` and sets `radarFresh = true`. On failure, increments diag counters; snapshot stays stale.
- `tickStateMachine()` is the extracted state machine (current lines 1044–1240). Runs at 10 Hz regardless of radar success.
- `TICK_INTERVAL_MS = 100`.

### Frame-count → wall-clock conversion

| Old (frame count) | Old value | New constant | New value | Rationale |
|---|---|---|---|---|
| `VITALS_CONFIRM` | 2 frames | `VITALS_CONFIRM_MS` | 600 | ~600ms at implied ~3 Hz |
| `ABSENT_CONFIRM` | 8 frames | *(removed)* | — | `ABSENT_HOLD_MS` (1200ms) already covers this |
| `HUMAN_STABLE_FALLBACK_CONFIRM` | 3 frames | `STABLE_FALLBACK_MS` | 1000 | ~1000ms at implied ~3 Hz |

### PresenceState struct changes

```cpp
struct PresenceState {
  uint32_t lastMs              = 0;     // last presence signal
  uint32_t vitalsValidSinceMs  = 0;     // 0 = not valid; >0 = when validity started
  uint32_t humanStableSinceMs  = 0;     // 0 = not stable; >0 = when stability started
  uint32_t lastSingleTargetMs  = 0;
  bool     seenSingleTarget    = false;
  // REMOVED: absentStreak, vitalsStreak, humanStableStreak
};
```

Tick phase logic:
- **Vitals**: Set `vitalsValidSinceMs = now` when vitals first become valid, reset to `0` when invalid. Transition to `RESTING_VITALS` when `(now - vitalsValidSinceMs) >= VITALS_CONFIRM_MS`.
- **Absence**: `(now - presence.lastMs) >= ABSENT_HOLD_MS` — already time-based, `absentStreak` removed.
- **Stable fallback**: Track `humanStableSinceMs` instead of `humanStableStreak`. Applies when `(now - humanStableSinceMs) >= STABLE_FALLBACK_MS`.

### Radar staleness detection

New constant: `RADAR_STALE_MS = 1500`.

When `(now - snap.radarMs) > RADAR_STALE_MS`, the tick forces `humanDetected = false` and `nTargets = 0`, driving the state machine toward NO_TARGET even without new frames.

### What doesn't change

- Telemetry emit throttles (already `millis()`-based)
- Vitals cache expiry (`VITALS_CACHE_EXPIRY_MS`)
- Guard rails, target picking, `guessPose()` hysteresis
- Host commands (handled in `pollHostUsbSerial()`)
- Python side — no changes; protocol unchanged
- `reachy_codec.h` — pure functions, untouched

### Testing

Firmware-only change. No new Python tests needed. Verify:
1. Full `pytest` suite still passes (no Python changes)
2. `ruff check` and `mypy` still pass
3. Manual hardware verification of state transition timing

## Approach chosen over alternatives

- **Convert frame counts to millis() in-place**: Rejected — state machine still only runs on successful `mmWave.update()`, so it freezes when radar stops. Bolting on a timeout escape hatch is messy.
- **RTOS timer task**: Rejected — overkill for Arduino single-threaded context. Seeed mmWave library isn't thread-safe. Adds concurrency complexity for no real benefit.
- **Split loop into poll + tick**: Chosen — clean separation, all timing is wall-clock, state machine never freezes, simple mental model.
