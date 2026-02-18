// Reachy-ready MR60BHA2 demo:
// - mmWave sensor over HW UART (mmWaveSerial)
// - Host control + telemetry over USB Serial (Serial)
// - Machine-readable JSONL output (one JSON object per line)
// - Host commands via Serial: HM=0/1, FOCUS=-1 or FOCUS=<cluster>, BIO_MS=<ms>, PING

#include <Arduino.h>
#include <math.h>
#include "Seeed_Arduino_mmWave.h"

#ifdef ESP32
  #include <HardwareSerial.h>
  // mmWave UART (change pins/serial instance to match your wiring if needed)
  HardwareSerial mmWaveSerial(0);
#else
  #define mmWaveSerial Serial1
#endif

#ifndef RANGE_STEP
  #define RANGE_STEP 1.0f
#endif

SEEED_MR60BHA2 mmWave;

// -----------------------------
// Types
// -----------------------------
enum class PersonState : uint8_t {
  NO_TARGET,
  MULTI_TARGET,
  PRESENT_FAR,
  MOVING,
  STILL_NEAR,
  RESTING_VITALS
};

enum class PoseGuess : uint8_t {
  UNKNOWN,
  SITTING,
  STANDING
};

struct FocusTarget {
  bool valid = false;
  int index = -1;
  int cluster = -1;
  float x_m = NAN;
  float y_m = NAN;
  float r_m = NAN;
  float bearing_deg = NAN;
  float speed_cm_s = NAN;
};

// -----------------------------
// Tuning
// -----------------------------
static const float NEAR_MIN_DIST_CM = 35.0f;
static const float NEAR_MAX_DIST_CM = 150.0f;

static const float MOVING_CM_S = 8.0f;

// Vitals guard rails (bpm)
static const float BR_MIN = 4.0f,  BR_MAX = 30.0f;
static const float HR_MIN = 35.0f, HR_MAX = 200.0f;

// Hysteresis
static const uint32_t ABSENT_HOLD_MS = 1200;
static const uint8_t  ABSENT_CONFIRM = 8;
static const uint8_t  VITALS_CONFIRM = 5;

// Pose heuristic (distance-based demo)
static const float SIT_STAND_THRESHOLD_CM = 55.0f;

// Output throttles
static const uint32_t TARGETS_JSON_MS_DEFAULT = 250;   // multi-target locations cadence
static const uint32_t BIO_JSON_MS_DEFAULT     = 1000;  // how often to output breath/heart (when allowed)

// -----------------------------
// Globals
// -----------------------------
static uint32_t t0 = 0;

static float lastDist = NAN, lastBR = NAN, lastHR = NAN;
static uint32_t lastPresenceMs = 0;
static uint8_t  absentStreak = 0;
static uint8_t  vitalsStreak = 0;

static uint32_t lastTargetsJson = 0;
static uint32_t lastBioJson     = 0;

static bool hostHeadMoving = false;   // HM=0/1 from host
static int  forcedFocusCluster = -1;  // FOCUS=-1 (auto) or FOCUS=<cluster>

// Runtime-configurable output rates:
static uint32_t TARGETS_JSON_MS = TARGETS_JSON_MS_DEFAULT;
static uint32_t BIO_JSON_MS     = BIO_JSON_MS_DEFAULT;

// -----------------------------
// Small helpers
// -----------------------------
static bool isFinitePositive(float v) { return isfinite(v) && v > 0.0f; }

static const char* stateName(PersonState s) {
  switch (s) {
    case PersonState::NO_TARGET:      return "NO_TARGET";
    case PersonState::MULTI_TARGET:   return "MULTI_TARGET";
    case PersonState::PRESENT_FAR:    return "PRESENT_FAR";
    case PersonState::MOVING:         return "MOVING";
    case PersonState::STILL_NEAR:     return "STILL_NEAR";
    case PersonState::RESTING_VITALS: return "RESTING_VITALS";
    default:                          return "UNKNOWN";
  }
}

static const char* poseName(PoseGuess p) {
  switch (p) {
    case PoseGuess::UNKNOWN:  return "UNKNOWN";
    case PoseGuess::SITTING:  return "SITTING";
    case PoseGuess::STANDING: return "STANDING";
    default:                  return "UNKNOWN";
  }
}

static PoseGuess guessPose(PersonState s, float dist_cm) {
  if (s == PersonState::NO_TARGET || isnan(dist_cm) || dist_cm <= 0.0f) return PoseGuess::UNKNOWN;
  return (dist_cm < SIT_STAND_THRESHOLD_CM) ? PoseGuess::SITTING : PoseGuess::STANDING;
}

// -----------------------------
// Host command parsing over USB Serial
// Commands (newline-terminated):
//   HM=0 / HM=1
//   FOCUS=-1 or FOCUS=<cluster>
//   BIO_MS=<ms>          (controls vitals JSON cadence)
//   TARGETS_MS=<ms>      (controls targets JSON cadence)
//   PING
// -----------------------------
static void applyCommand(const char* line) {
  while (*line == ' ' || *line == '\t') line++;

  if (strncmp(line, "HM=", 3) == 0) {
    hostHeadMoving = (atoi(line + 3) != 0);
    Serial.printf("{\"type\":\"ack\",\"hm\":%d}\n", hostHeadMoving ? 1 : 0);
    return;
  }

  if (strncmp(line, "FOCUS=", 6) == 0) {
    forcedFocusCluster = atoi(line + 6);
    Serial.printf("{\"type\":\"ack\",\"focus\":%d}\n", forcedFocusCluster);
    return;
  }

  if (strncmp(line, "BIO_MS=", 7) == 0) {
    uint32_t v = (uint32_t)max(50, atoi(line + 7)); // clamp
    BIO_JSON_MS = v;
    Serial.printf("{\"type\":\"ack\",\"bio_ms\":%lu}\n", (unsigned long)BIO_JSON_MS);
    return;
  }

  if (strncmp(line, "TARGETS_MS=", 11) == 0) {
    uint32_t v = (uint32_t)max(50, atoi(line + 11)); // clamp
    TARGETS_JSON_MS = v;
    Serial.printf("{\"type\":\"ack\",\"targets_ms\":%lu}\n", (unsigned long)TARGETS_JSON_MS);
    return;
  }

  if (strcmp(line, "PING") == 0) {
    Serial.printf("{\"type\":\"pong\",\"t_ms\":%lu}\n", (unsigned long)(millis() - t0));
    return;
  }

  // Unknown command
  Serial.printf("{\"type\":\"err\",\"msg\":\"unknown_cmd\"}\n");
}

static void pollHostUsbSerial() {
  static char buf[128];
  static size_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[n] = 0;
      if (n > 0) applyCommand(buf);
      n = 0;
      continue;
    }

    if (n < sizeof(buf) - 1) buf[n++] = c;
    else n = 0; // overflow -> reset line
  }
}

// -----------------------------
// Focus picking
// -----------------------------
static FocusTarget pickClosestTarget(const PeopleCounting& info) {
  FocusTarget best;
  float best_r = 1e9f;

  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x*x + y*y);
    if (!isfinite(r)) continue;

    if (r < best_r) {
      best_r = r;
      best.valid = true;
      best.index = (int)i;
      best.cluster = t.cluster_index;
      best.x_m = x;
      best.y_m = y;
      best.r_m = r;
      best.bearing_deg = atan2f(x, y) * 180.0f / PI;
      best.speed_cm_s = t.dop_index * RANGE_STEP;
    }
  }
  return best;
}

static FocusTarget pickForcedCluster(const PeopleCounting& info, int cluster) {
  FocusTarget out;
  if (cluster < 0) return out;

  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    if (t.cluster_index != cluster) continue;

    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x*x + y*y);

    out.valid = true;
    out.index = (int)i;
    out.cluster = t.cluster_index;
    out.x_m = x;
    out.y_m = y;
    out.r_m = r;
    out.bearing_deg = atan2f(x, y) * 180.0f / PI;
    out.speed_cm_s = t.dop_index * RANGE_STEP;
    return out;
  }
  return out;
}

// -----------------------------
// JSON output helpers (JSON Lines)
// -----------------------------
static void jsonTargets(uint32_t t_ms, const PeopleCounting& info, const FocusTarget& focus) {
  Serial.print("{\"type\":\"targets\",\"t_ms\":");
  Serial.print(t_ms);
  Serial.print(",\"n\":");
  Serial.print((unsigned)info.targets.size());

  if (forcedFocusCluster >= 0) {
    Serial.print(",\"forced_focus\":");
    Serial.print(forcedFocusCluster);
  } else {
    Serial.print(",\"forced_focus\":-1");
  }

  if (focus.valid) {
    Serial.print(",\"focus\":{");
    Serial.print("\"cluster\":"); Serial.print(focus.cluster);
    Serial.print(",\"x\":");      Serial.print(focus.x_m, 3);
    Serial.print(",\"y\":");      Serial.print(focus.y_m, 3);
    Serial.print(",\"r\":");      Serial.print(focus.r_m, 3);
    Serial.print(",\"bearing\":");Serial.print(focus.bearing_deg, 1);
    Serial.print(",\"v\":");      Serial.print(focus.speed_cm_s, 2);
    Serial.print("}");
  } else {
    Serial.print(",\"focus\":null");
  }

  Serial.print(",\"targets\":[");
  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    float x = t.x_point, y = t.y_point;
    float r = sqrtf(x*x + y*y);
    float bearing = atan2f(x, y) * 180.0f / PI;
    float v = t.dop_index * RANGE_STEP;

    if (i) Serial.print(",");
    Serial.print("{\"cluster\":"); Serial.print(t.cluster_index);
    Serial.print(",\"x\":");       Serial.print(x, 3);
    Serial.print(",\"y\":");       Serial.print(y, 3);
    Serial.print(",\"r\":");       Serial.print(r, 3);
    Serial.print(",\"bearing\":"); Serial.print(bearing, 1);
    Serial.print(",\"v\":");       Serial.print(v, 2);
    Serial.print("}");
  }
  Serial.print("]}");
  Serial.print("\n");
}

static void jsonState(uint32_t t_ms,
                      PersonState s,
                      PoseGuess p,
                      bool headMoving,
                      bool human,
                      int nTargets,
                      float dist_cm,
                      bool dist_ok) {
  Serial.print("{\"type\":\"state\",\"t_ms\":");
  Serial.print(t_ms);
  Serial.print(",\"state\":\"");
  Serial.print(stateName(s));
  Serial.print("\",\"pose\":\"");
  Serial.print(poseName(p));
  Serial.print("\",\"head_moving\":");
  Serial.print(headMoving ? 1 : 0);
  Serial.print(",\"human\":");
  Serial.print(human ? 1 : 0);
  Serial.print(",\"n_targets\":");
  Serial.print(nTargets);
  Serial.print(",\"dist_cm\":");
  if (isfinite(dist_cm)) Serial.print(dist_cm, 2); else Serial.print("null");
  Serial.print(",\"dist_new\":");
  Serial.print(dist_ok ? 1 : 0);
  Serial.print("}\n");
}

static void jsonBio(uint32_t t_ms,
                    bool allowed,
                    bool valid,
                    float br,
                    bool br_ok,
                    float hr,
                    bool hr_ok) {
  Serial.print("{\"type\":\"bio\",\"t_ms\":");
  Serial.print(t_ms);
  Serial.print(",\"allowed\":");
  Serial.print(allowed ? 1 : 0);
  Serial.print(",\"valid\":");
  Serial.print(valid ? 1 : 0);

  Serial.print(",\"br\":");
  if (isfinite(br)) Serial.print(br, 2); else Serial.print("null");
  Serial.print(",\"br_new\":");
  Serial.print(br_ok ? 1 : 0);

  Serial.print(",\"hr\":");
  if (isfinite(hr)) Serial.print(hr, 2); else Serial.print("null");
  Serial.print(",\"hr_new\":");
  Serial.print(hr_ok ? 1 : 0);

  Serial.print("}\n");
}

// -----------------------------
// Setup / Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  mmWave.begin(&mmWaveSerial);

  t0 = millis();
  Serial.println("{\"type\":\"hello\",\"msg\":\"MR60BHA2 ready\",\"cmds\":[\"HM=0/1\",\"FOCUS=-1/<cluster>\",\"BIO_MS=<ms>\",\"TARGETS_MS=<ms>\",\"PING\"]}");
}

void loop() {
  if (!mmWave.update(100)) return;

  // Read host commands on USB Serial
  pollHostUsbSerial();

  bool headMoving = hostHeadMoving;

  // Presence / targets
  bool human = mmWave.isHumanDetected();

  PeopleCounting info;
  bool haveTargets = mmWave.getPeopleCountingTargetInfo(info);
  int nTargets = haveTargets ? (int)info.targets.size() : 0;

  // Focus target: forced cluster if requested, else closest
  FocusTarget focus;
  if (haveTargets && nTargets > 0) {
    if (forcedFocusCluster >= 0) {
      focus = pickForcedCluster(info, forcedFocusCluster);
      if (!focus.valid) focus = pickClosestTarget(info);
    } else {
      focus = pickClosestTarget(info);
    }
  }

  // Distance / vitals
  float dist_cm = NAN, br = NAN, hr = NAN;
  bool dist_ok = mmWave.getDistance(dist_cm);
  bool br_ok   = mmWave.getBreathRate(br);
  bool hr_ok   = mmWave.getHeartRate(hr);

  // Keep last-good values (do not turn missing values into zeros)
  if (dist_ok && isfinite(dist_cm)) lastDist = dist_cm; else dist_cm = lastDist;
  if (br_ok   && isfinite(br))      lastBR   = br;      else br      = lastBR;
  if (hr_ok   && isfinite(hr))      lastHR   = hr;      else hr      = lastHR;

  // Presence (any signal)
  bool present_now =
      human ||
      (nTargets > 0) ||
      (dist_ok && isFinitePositive(dist_cm)) ||
      (br_ok   && isFinitePositive(br)) ||
      (hr_ok   && isFinitePositive(hr));

  uint32_t now = millis();
  if (present_now) {
    lastPresenceMs = now;
    absentStreak = 0;
  } else {
    absentStreak++;
  }
  bool presence_recent = (now - lastPresenceMs) < ABSENT_HOLD_MS;

  // Movement: target doppler + head motion
  bool targetMoving = focus.valid && isfinite(focus.speed_cm_s) && fabsf(focus.speed_cm_s) >= MOVING_CM_S;
  bool moving = headMoving || targetMoving;

  // Near zone (for vitals)
  bool near = (!isnan(dist_cm) && dist_cm >= NEAR_MIN_DIST_CM && dist_cm <= NEAR_MAX_DIST_CM);

  // Vitals validity gate: only trust when single target AND head stable
  bool singleTarget = (nTargets == 1);

  bool br_valid = br_ok && isfinite(br) && (br >= BR_MIN) && (br <= BR_MAX);
  bool hr_valid = hr_ok && isfinite(hr) && (hr >= HR_MIN) && (hr <= HR_MAX);

  bool vitals_allowed = singleTarget && !headMoving;
  bool vitals_valid = vitals_allowed && br_valid && hr_valid;

  vitalsStreak = vitals_valid ? (uint8_t)min<int>(255, vitalsStreak + 1) : 0;

  // Decide state
  PersonState s;
  if (!presence_recent && absentStreak >= ABSENT_CONFIRM) {
    s = PersonState::NO_TARGET;
    vitalsStreak = 0;
  } else if (nTargets > 1) {
    s = PersonState::MULTI_TARGET;
    vitalsStreak = 0;
  } else if (moving) {
    s = PersonState::MOVING;
    vitalsStreak = 0;
  } else if (near && vitalsStreak >= VITALS_CONFIRM) {
    s = PersonState::RESTING_VITALS;
  } else if (near) {
    s = PersonState::STILL_NEAR;
  } else {
    s = PersonState::PRESENT_FAR;
  }

  PoseGuess p = guessPose(s, dist_cm);
  uint32_t t_ms = now - t0;

  // Emit JSON: targets list (throttled)
  if (haveTargets && nTargets > 0 && (now - lastTargetsJson >= TARGETS_JSON_MS)) {
    lastTargetsJson = now;
    jsonTargets(t_ms, info, focus);
  }

  // Emit JSON: state (whenever it changes materially, plus once per second as a keepalive)
  static uint32_t lastStateJson = 0;
  static PersonState prevS = PersonState::NO_TARGET;
  static PoseGuess prevP = PoseGuess::UNKNOWN;
  static bool prevHM = false;
  static int prevN = -1;

  bool stateChanged = (s != prevS) || (p != prevP) || (headMoving != prevHM) || (nTargets != prevN);
  if (stateChanged || (now - lastStateJson > 1000)) {
    lastStateJson = now;
    jsonState(t_ms, s, p, headMoving, human, nTargets, dist_cm, dist_ok);
    prevS = s; prevP = p; prevHM = headMoving; prevN = nTargets;
  }

  // Emit JSON: bio (throttled and only meaningful when allowed/valid)
  // This is the knob you asked about:
  //   BIO_JSON_MS controls how often bio messages are output.
  if (now - lastBioJson >= BIO_JSON_MS) {
    lastBioJson = now;
    jsonBio(t_ms, vitals_allowed, vitals_valid, br, br_ok, hr, hr_ok);
  }
}