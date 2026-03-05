# Decouple State Machine from Sensor Polling — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Run the firmware state machine at a fixed 10 Hz tick rate, independent of mmWave radar polling, so state transitions use wall-clock timing and never freeze when the radar misses frames.

**Architecture:** Split `loop()` into a poll phase (every iteration: radar, light, host commands) and a tick phase (fixed 10 Hz: state machine + telemetry). Replace all frame-count thresholds (`VITALS_CONFIRM`, `ABSENT_CONFIRM`, `HUMAN_STABLE_FALLBACK_CONFIRM`) with `millis()`-based durations. Introduce a `SensorSnapshot` struct as the boundary between polling and state logic.

**Tech Stack:** Arduino C++ (ESP32/XIAO), Seeed mmWave library, BH1750 library

**Design doc:** `docs/plans/2026-03-05-decouple-state-machine-design.md`

---

### Task 1: Add `SensorSnapshot` struct and `TICK_INTERVAL_MS` constant

**Files:**
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino:261-321` (constants section), `:306-398` (global state section)

**Context:** The firmware groups all mutable state into structs (lines 306-398). We need a new struct to buffer radar readings between the poll phase and the tick phase, plus the new timing constants.

**Step 1: Add new timing constants**

After the existing telemetry throttle constants (line 300), add:

```cpp
// --- State machine tick rate ---
// The state machine runs at a fixed wall-clock frequency, independent of
// radar polling success. This prevents state transitions from freezing
// when mmWave.update() misses frames.
static const uint32_t TICK_INTERVAL_MS = 100;  // 10 Hz state machine tick
static const uint32_t RADAR_STALE_MS  = 1500;  // radar data considered stale after this
```

**Step 2: Convert frame-count constants to wall-clock durations**

Replace the three frame-count constants (lines 271-280):

Before:
```cpp
static const uint8_t VITALS_CONFIRM = 2;
static const uint8_t HUMAN_STABLE_FALLBACK_CONFIRM = 3;
```

After:
```cpp
static const uint32_t VITALS_CONFIRM_MS = 600;           // ~2 frames @ 3 Hz → 600 ms of valid vitals
static const uint32_t STABLE_FALLBACK_MS = 1000;          // ~3 frames @ 3 Hz → 1000 ms of stable human signal
```

Also remove `ABSENT_CONFIRM` (line 269). The existing `ABSENT_HOLD_MS = 1200` already provides the time-based absence gate. Update the comment block (lines 261-269) to reflect this.

**Step 3: Add `SensorSnapshot` struct**

After the `PrevState` struct (line 387), add:

```cpp
// Buffered sensor readings. The poll phase writes this every iteration;
// the tick phase reads it at a fixed 10 Hz rate. This decouples the
// state machine from the radar's variable frame rate.
struct SensorSnapshot {
  uint32_t   radarMs       = 0;      // millis() of last successful mmWave.update()
  bool       humanDetected = false;
  uint8_t    nTargets      = 0;
  PeopleCounting targetInfo;          // full target array from radar
  FocusTarget focus;                  // picked target for vitals
  float      dist_cm       = NAN;
  float      breathRate    = NAN;
  float      heartRate     = NAN;
  bool       dist_ok       = false;
  bool       br_ok         = false;
  bool       hr_ok         = false;
  bool       radarFresh    = false;   // true when poll produced new data; cleared after tick
};
static SensorSnapshot snap;
```

**Step 4: Update `PresenceState` struct**

Replace the existing struct (lines 334-341):

Before:
```cpp
struct PresenceState {
  uint32_t lastMs           = 0;
  uint8_t  absentStreak     = 0;
  uint8_t  vitalsStreak     = 0;
  uint8_t  humanStableStreak = 0;
  uint32_t lastSingleTargetMs = 0;
  bool     seenSingleTarget = false;
};
```

After:
```cpp
struct PresenceState {
  uint32_t lastMs              = 0;     // last millis() when any presence signal was detected
  uint32_t vitalsValidSinceMs  = 0;     // 0 = not valid; >0 = millis() when validity started
  uint32_t humanStableSinceMs  = 0;     // 0 = not stable; >0 = millis() when stability started
  uint32_t lastSingleTargetMs  = 0;     // last millis() when exactly 1 target was seen
  bool     seenSingleTarget    = false; // true once we've seen at least one single-target frame
};
```

**Step 5: Add tick timer global**

After `static PrevState prev;` (line 387), add:

```cpp
static uint32_t lastTickMs = 0;  // millis() of last state machine tick
```

**Step 6: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy-sensor.ino
git commit -m "refactor(fw): add SensorSnapshot struct and wall-clock timing constants"
```

---

### Task 2: Extract `pollRadar()` function

**Files:**
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino:993-1112` (loop step 3 + 3a + 3b)

**Context:** The current `loop()` does radar polling, sensor reading, fallback cache logic, and state machine all inline. We need to extract the polling + cache logic into a standalone function that writes to `SensorSnapshot`.

**Step 1: Create `pollRadar()` function**

Add this function before `loop()` (before line 993). It combines the current mmWave.update() call (lines 1037-1042), target reading (lines 1048-1066), distance/vitals reading with fallback cache (lines 1068-1112), and writes results to `snap`.

```cpp
// ---------------------------------------------------------------------------
// pollRadar() — Poll mmWave sensor and update SensorSnapshot
// ---------------------------------------------------------------------------
// Called every loop iteration. If the radar returns a complete frame,
// populates the snapshot with fresh data and sets radarFresh = true.
// On failure, the snapshot retains previous values (staleness detected
// by the tick phase via snap.radarMs).
void pollRadar(uint32_t now) {
  if (!mmWave.update(100)) {
    diag.mmwaveFails++;
    diag.mmwaveConsecFails++;
    return;
  }
  diag.mmwaveConsecFails = 0;

  snap.radarMs = now;
  snap.radarFresh = true;

  // 3a: Read presence and target data
  snap.humanDetected = mmWave.isHumanDetected();

  bool haveTargets = mmWave.getPeopleCountingTargetInfo(snap.targetInfo);
  snap.nTargets = haveTargets ? (uint8_t)snap.targetInfo.targets.size() : 0;

  // Pick focus target
  snap.focus = FocusTarget{};
  if (haveTargets && snap.nTargets > 0) {
    if (host.focusCluster >= 0) {
      snap.focus = pickForcedCluster(snap.targetInfo, host.focusCluster);
      if (!snap.focus.valid) snap.focus = pickClosestTarget(snap.targetInfo);
    } else {
      snap.focus = pickClosestTarget(snap.targetInfo);
    }
  }

  // 3b: Read distance and vitals with fallback cache
  float dist_cm = NAN, br = NAN, hr = NAN;
  snap.dist_ok = mmWave.getDistance(dist_cm);
  snap.br_ok = mmWave.getBreathRate(br);
  snap.hr_ok = mmWave.getHeartRate(hr);

  // Distance fallback (no expiry — distance is stable)
  if (snap.dist_ok && isfinite(dist_cm)) {
    vitals.dist = dist_cm;
  } else {
    dist_cm = vitals.dist;
  }
  snap.dist_ok = isFinitePositive(dist_cm);
  snap.dist_cm = dist_cm;

  // Breathing rate fallback (2-second expiry)
  if (snap.br_ok && isfinite(br)) {
    vitals.br = br;
    vitals.brUpdateMs = now;
  } else if ((now - vitals.brUpdateMs) <= VITALS_CACHE_EXPIRY_MS) {
    br = vitals.br;
  } else {
    vitals.br = NAN;
    br = NAN;
  }
  snap.breathRate = br;

  // Heart rate fallback (2-second expiry)
  if (snap.hr_ok && isfinite(hr)) {
    vitals.hr = hr;
    vitals.hrUpdateMs = now;
  } else if ((now - vitals.hrUpdateMs) <= VITALS_CACHE_EXPIRY_MS) {
    hr = vitals.hr;
  } else {
    vitals.hr = NAN;
    hr = NAN;
  }
  snap.heartRate = hr;
}
```

**Step 2: Verify it compiles**

Mentally verify: `pollRadar()` uses the same globals as the original inline code (`mmWave`, `diag`, `host`, `vitals`, `snap`). All are file-scope statics. No new dependencies.

**Step 3: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy-sensor.ino
git commit -m "refactor(fw): extract pollRadar() function for sensor snapshot buffering"
```

---

### Task 3: Extract `tickStateMachine()` function

**Files:**
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino:1114-1241` (loop steps 3c through 3h)

**Context:** This is the core refactor. Extract the state machine (presence detection, movement detection, vitals gating, state decision, telemetry emission) into `tickStateMachine(now)`. Convert all three frame-count thresholds to wall-clock durations. Add radar staleness detection.

**Step 1: Create `tickStateMachine()` function**

Add this function after `pollRadar()`, before `loop()`.

```cpp
// ---------------------------------------------------------------------------
// tickStateMachine() — Fixed-rate state machine tick (10 Hz)
// ---------------------------------------------------------------------------
// Reads from SensorSnapshot (written by pollRadar) and runs the full
// state decision + telemetry emission. All hysteresis uses wall-clock
// durations, not frame counts.
void tickStateMachine(uint32_t now) {

  // Snapshot the host-controlled head-moving flag (consistent for this tick).
  bool headMoving = host.headMoving;

  // --- Radar staleness override ---
  // If the radar hasn't delivered a frame in RADAR_STALE_MS, force
  // the snapshot to "nobody here" so the state machine doesn't freeze
  // in a stale state.
  bool radarStale = (snap.radarMs == 0) || ((now - snap.radarMs) > RADAR_STALE_MS);
  bool humanDetected = radarStale ? false : snap.humanDetected;
  uint8_t nTargets   = radarStale ? 0     : snap.nTargets;
  float dist_cm      = radarStale ? NAN   : snap.dist_cm;
  bool dist_ok       = radarStale ? false : snap.dist_ok;
  float br           = radarStale ? NAN   : snap.breathRate;
  bool br_ok         = radarStale ? false : snap.br_ok;
  float hr           = radarStale ? NAN   : snap.heartRate;
  bool hr_ok         = radarStale ? false : snap.hr_ok;
  FocusTarget focus  = radarStale ? FocusTarget{} : snap.focus;

  // Clear the fresh flag after consuming
  snap.radarFresh = false;

  // ── Presence detection ──────────────────────────────────────────────────
  bool present_now = humanDetected || (nTargets > 0) ||
                     (dist_ok && isFinitePositive(dist_cm)) ||
                     (br_ok && isFinitePositive(br)) ||
                     (hr_ok && isFinitePositive(hr));

  if (present_now) {
    presence.lastMs = now;
  }
  bool presence_recent = (now - presence.lastMs) < ABSENT_HOLD_MS;

  // ── Movement detection ──────────────────────────────────────────────────
  bool targetMoving = focus.valid && isfinite(focus.speed_cm_s) &&
                      fabsf(focus.speed_cm_s) >= MOVING_CM_S;
  bool moving = headMoving || targetMoving;

  // ── Near zone check ─────────────────────────────────────────────────────
  bool near = (!isnan(dist_cm) && dist_cm >= NEAR_MIN_DIST_CM &&
               dist_cm <= NEAR_MAX_DIST_CM);

  // ── Vitals validity gating (wall-clock durations) ───────────────────────
  bool singleTarget = (nTargets == 1);
  if (singleTarget) {
    presence.seenSingleTarget = true;
    presence.lastSingleTargetMs = now;
  }

  // Track time with stable human signal (replaces humanStableStreak)
  if (humanDetected && !headMoving) {
    if (presence.humanStableSinceMs == 0) presence.humanStableSinceMs = now;
  } else {
    presence.humanStableSinceMs = 0;
  }

  // Fallback lock: allow vitals to continue briefly when target tracking drops
  bool singleTargetRecent = presence.seenSingleTarget &&
                            ((now - presence.lastSingleTargetMs) <= TARGET_LOSS_GRACE_MS);
  bool humanStableLongEnough = (presence.humanStableSinceMs > 0) &&
                               ((now - presence.humanStableSinceMs) >= STABLE_FALLBACK_MS);
  bool fallbackTargetLock = (!singleTarget) && (nTargets == 0) &&
                            singleTargetRecent && humanStableLongEnough;

  // Guard rail check
  bool br_valid = br_ok && isfinite(br) && (br >= host.brMin) && (br <= host.brMax);
  bool hr_valid = hr_ok && isfinite(hr) && (hr >= host.hrMin) && (hr <= host.hrMax);

  // Three-level vitals gate (wall-clock duration replaces frame count)
  bool vitals_allowed = !headMoving && (singleTarget || fallbackTargetLock);
  bool vitals_valid = vitals_allowed && br_valid && hr_valid;

  // Track time with valid vitals (replaces vitalsStreak)
  if (vitals_valid) {
    if (presence.vitalsValidSinceMs == 0) presence.vitalsValidSinceMs = now;
  } else {
    presence.vitalsValidSinceMs = 0;
  }
  bool vitalsConfirmed = (presence.vitalsValidSinceMs > 0) &&
                         ((now - presence.vitalsValidSinceMs) >= VITALS_CONFIRM_MS);

  // ── State decision ──────────────────────────────────────────────────────
  PersonState s;
  if (!presence_recent) {
    s = PersonState::NO_TARGET;
    presence.vitalsValidSinceMs = 0;
  } else if (nTargets > 1) {
    s = PersonState::MULTI_TARGET;
    presence.vitalsValidSinceMs = 0;
  } else if (moving) {
    s = PersonState::MOVING;
    presence.vitalsValidSinceMs = 0;
  } else if (near && vitalsConfirmed) {
    s = PersonState::RESTING_VITALS;
  } else if (near) {
    s = PersonState::STILL_NEAR;
  } else {
    s = PersonState::PRESENT_FAR;
  }

  PoseGuess p = guessPose(s, dist_cm);
  uint32_t t_ms = now - t0;

  // ── Emit telemetry ──────────────────────────────────────────────────────

  // EVT_TARGETS
  bool haveTargets = (nTargets > 0);
  if (haveTargets && (now - emitTimers.targets >= host.targetsMs)) {
    emitTimers.targets = now;
    emitTargets(t_ms, snap.targetInfo, snap.focus);
  }

  // EVT_STATE
  bool stateChanged = (s != prev.s) || (p != prev.p) ||
                      (headMoving != prev.hm) || (nTargets != prev.n);
  bool intervalOk = (now - emitTimers.state >= STATE_MIN_INTERVAL_MS);
  if ((stateChanged && intervalOk) || (now - emitTimers.state > 1000)) {
    emitTimers.state = now;
    emitState(t_ms, s, p, headMoving, humanDetected, nTargets, dist_cm, dist_ok);
    prev.s = s;
    prev.p = p;
    prev.hm = headMoving;
    prev.n = nTargets;
  }

  // EVT_BIO
  if (now - emitTimers.bio >= host.bioMs) {
    emitTimers.bio = now;
    bool br_emit_ok = vitals_allowed && br_valid;
    bool hr_emit_ok = vitals_allowed && hr_valid;
    emitBio(t_ms, vitals_allowed, vitals_valid, br, br_emit_ok, hr, hr_emit_ok);
  }
}
```

**Key differences from original:**
- `presence.absentStreak` removed — `!presence_recent` alone gates NO_TARGET (already time-based via `ABSENT_HOLD_MS`)
- `presence.vitalsStreak` replaced by `vitalsValidSinceMs` duration check
- `presence.humanStableStreak` replaced by `humanStableSinceMs` duration check
- Radar staleness forces snapshot to "nobody" after `RADAR_STALE_MS`
- `haveTargets` is derived from `nTargets > 0` instead of from `mmWave.getPeopleCountingTargetInfo()` return value (already captured in snapshot)

**Step 2: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy-sensor.ino
git commit -m "refactor(fw): extract tickStateMachine() with wall-clock timing"
```

---

### Task 4: Rewrite `loop()` with poll + tick structure

**Files:**
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino:993-1241` (entire `loop()` function)

**Context:** Now that `pollRadar()` and `tickStateMachine()` are extracted, replace the entire `loop()` body with the clean poll + tick structure.

**Step 1: Replace `loop()` body**

Replace everything from line 993 to the end of the file with:

```cpp
// ============================================================================
// Main loop — poll sensors + fixed-rate state machine tick
// ============================================================================
// The loop runs two independent phases:
//
//   POLL PHASE (every iteration):
//     - Host serial commands (always responsive)
//     - mmWave radar → writes SensorSnapshot
//     - BH1750 light sensor (1-second throttle)
//     - Diagnostics (10-second throttle)
//
//   TICK PHASE (fixed 10 Hz via TICK_INTERVAL_MS):
//     - State machine reads SensorSnapshot
//     - All hysteresis uses wall-clock durations (not frame counts)
//     - Emits telemetry (EVT_STATE, EVT_TARGETS, EVT_BIO)
//
// This decoupling ensures state transitions are predictable even when
// the radar's frame rate varies or the sensor temporarily fails.
void loop() {
  uint32_t now = millis();

  // ── Poll phase (every iteration) ────────────────────────────────────────
  pollHostUsbSerial();
  pollRadar(now);

  // Light sensor (1-second throttle, independent of radar)
  if (!host.lightReady && (now - emitTimers.lightRetry >= LIGHT_RETRY_MS)) {
    emitTimers.lightRetry = now;
    host.lightReady = lightSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR, &Wire);
  }
  if (now - emitTimers.light >= LIGHT_MS_DEFAULT) {
    emitTimers.light = now;
    float lux = NAN;
    bool lightValid = false;
    if (host.lightReady) {
      lux = lightSensor.readLightLevel();
      lightValid = isfinite(lux) && lux >= 0.0f;
    }
    emitLight(now - t0, lightValid, lux);
  }

  // Diagnostics (10-second throttle)
  if (now - emitTimers.diag >= DIAG_MS) {
    emitTimers.diag = now;
    emitDiag(now - t0, diag.mmwaveFails, diag.mmwaveConsecFails, diag.txDrops);
  }

  // ── Tick phase (fixed 10 Hz) ────────────────────────────────────────────
  if (now - lastTickMs < TICK_INTERVAL_MS) return;
  lastTickMs = now;
  tickStateMachine(now);
}
```

**Step 2: Remove old inline state machine code**

The old code (lines 1033-1241) is completely replaced by the new `loop()`. All state machine logic now lives in `tickStateMachine()` (Task 3). All radar polling logic lives in `pollRadar()` (Task 2).

**Step 3: Update file header comments**

Update the module docstring (near line 14-17) to mention the poll/tick architecture:

```cpp
// The firmware processes raw sensor data into a higher-level "person state"
// (nobody here, someone far away, someone sitting still with good vitals, etc.)
// and sends periodic telemetry events to the host. A fixed 10 Hz state machine
// tick ensures predictable transitions independent of the radar's frame rate.
```

**Step 4: Compile and verify**

If you have the Arduino IDE or `arduino-cli` configured:

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 hardware/arduino/reachy-sensor/
```

If not, verify by visual code review that:
- All globals referenced in `pollRadar()` and `tickStateMachine()` are file-scope statics
- No variables are used before being written in the new flow
- `SensorSnapshot` fields cover everything the state machine needs

**Step 5: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy-sensor.ino
git commit -m "refactor(fw): rewrite loop() with poll/tick architecture"
```

---

### Task 5: Update documentation

**Files:**
- Modify: `CLAUDE.md`
- Modify: `docs/TODO.md`
- Modify: `docs/20260223_roadmap.md`
- Modify: `hardware/README.md`

**Step 1: Update `docs/20260223_roadmap.md`**

Strike through Phase 4 item 5:

Before:
```
5. Decouple state machine from sensor polling rate — run at fixed wall-clock frequency
```

After:
```
5. ~~Decouple state machine from sensor polling rate — run at fixed wall-clock frequency~~ (done: poll/tick architecture, 10 Hz fixed tick, frame-count thresholds converted to millis() durations, radar staleness detection; 2026-03-05)
```

Add to the Completed section in Future Vision:
```
- Decoupled state machine from sensor polling: poll/tick architecture with 10 Hz fixed tick, wall-clock hysteresis, radar staleness detection (2026-03-05)
```

**Step 2: Update `docs/TODO.md`**

Add to Done (recent):
```
- [x] **FW-REFACTOR-1**: Decouple state machine from sensor polling — poll/tick architecture in `reachy-sensor.ino`: `pollRadar()` writes `SensorSnapshot`, `tickStateMachine()` runs at fixed 10 Hz; frame-count thresholds (`VITALS_CONFIRM`, `ABSENT_CONFIRM`, `HUMAN_STABLE_FALLBACK_CONFIRM`) converted to wall-clock durations (`VITALS_CONFIRM_MS`, `STABLE_FALLBACK_MS`); `RADAR_STALE_MS` (1500ms) forces NO_TARGET when radar is dead (2026-03-05)
```

**Step 3: Update `CLAUDE.md`**

In the "Hardware: mmWave Sensor Module" section, find the firmware description and add a note about the poll/tick architecture. After "Implements a state machine with person states":

```
- Poll/tick architecture: `pollRadar()` writes `SensorSnapshot` every iteration; `tickStateMachine()` runs at fixed 10 Hz with wall-clock hysteresis (no frame-count coupling)
```

Update the test count if any Python tests were added (unlikely for this firmware-only change — verify by running `uv run pytest tests/ --ignore=tests/vision -q`).

**Step 4: Update `hardware/README.md`**

In the firmware description section, add a note about the architecture:

```
The firmware uses a poll/tick architecture: sensor polling runs every iteration,
while the state machine ticks at a fixed 10 Hz rate. All hysteresis thresholds
use wall-clock durations (millis()), not frame counts, ensuring predictable
state transitions regardless of radar frame rate.
```

**Step 5: Commit**

```bash
git add CLAUDE.md docs/TODO.md docs/20260223_roadmap.md hardware/README.md
git commit -m "docs: sync all surfaces with state machine decoupling refactor"
```

---

### Task 6: Full verification

**Files:** None (read-only checks)

**Step 1: Run Python test suite**

```bash
uv run pytest tests/ --ignore=tests/vision -q
```

Expected: Same pass count as before (508 passed, 2 known failures). This is a firmware-only change so no Python tests should break.

**Step 2: Run lint and type check**

```bash
uv run ruff check .
uv run mypy src/ tests/
```

Expected: No new errors (firmware changes don't affect Python lint/types).

**Step 3: Visual code review**

Review the final `reachy-sensor.ino` for:
- No dangling references to removed fields (`absentStreak`, `vitalsStreak`, `humanStableStreak`)
- No references to removed constants (`ABSENT_CONFIRM`, `VITALS_CONFIRM`, `HUMAN_STABLE_FALLBACK_CONFIRM`)
- `SensorSnapshot` fields cover all data the tick phase needs
- `pollRadar()` and `tickStateMachine()` don't reference each other's local variables

```bash
# Quick grep for removed symbols
grep -n 'absentStreak\|vitalsStreak\|humanStableStreak\|ABSENT_CONFIRM\|VITALS_CONFIRM\b\|HUMAN_STABLE_FALLBACK_CONFIRM' hardware/arduino/reachy-sensor/reachy-sensor.ino
```

Expected: No matches (all replaced by wall-clock equivalents).

**Step 4: Push**

```bash
git push origin main
```
