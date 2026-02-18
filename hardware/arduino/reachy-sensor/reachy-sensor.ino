#include <Arduino.h>
#include <math.h>
#include "Seeed_Arduino_mmWave.h"

#ifdef ESP32
  #include <HardwareSerial.h>
  HardwareSerial mmWaveSerial(0);

  // Host control UART (set pins below)
  HardwareSerial ctrlSerial(1);
#else
  #define mmWaveSerial Serial1
  #define ctrlSerial   Serial  // fallback
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
// Name helpers
// -----------------------------
const char* stateName(PersonState s) {
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

const char* poseName(PoseGuess p) {
  switch (p) {
    case PoseGuess::UNKNOWN:  return "UNKNOWN";
    case PoseGuess::SITTING:  return "SITTING";
    case PoseGuess::STANDING: return "STANDING";
    default:                  return "UNKNOWN";
  }
}

// -----------------------------
// Tuning
// -----------------------------
static const float NEAR_MIN_DIST_CM = 35.0f;
static const float NEAR_MAX_DIST_CM = 150.0f;

static const float MOVING_CM_S   = 8.0f;

// Vitals guard rails
static const float BR_MIN = 4.0f,  BR_MAX = 30.0f;
static const float HR_MIN = 35.0f, HR_MAX = 200.0f;

// Hysteresis
static const uint32_t ABSENT_HOLD_MS = 1200;
static const uint8_t  ABSENT_CONFIRM = 8;
static const uint8_t  VITALS_CONFIRM = 5;

// Pose demo threshold (distance-based)
static const float SIT_STAND_THRESHOLD_CM = 55.0f;

// Print throttles
static const uint32_t TARGETS_PRINT_MS = 500;

// -----------------------------
// Globals
// -----------------------------
static uint32_t t0 = 0;

static PersonState lastState = PersonState::NO_TARGET;
static PoseGuess   lastPose  = PoseGuess::UNKNOWN;

static float lastDist = NAN, lastBR = NAN, lastHR = NAN;
static uint32_t lastPresenceMs = 0;
static uint8_t  absentStreak = 0;
static uint8_t  vitalsStreak = 0;

static uint32_t lastTargetsPrint = 0;
static int lastFocusCluster = -999;

// Host control state:
static bool hostHeadMoving = false;
static int  forcedFocusCluster = -1; // -1 => no forced focus

// -----------------------------
// Helpers
// -----------------------------
static bool isFinitePositive(float v) {
  return isfinite(v) && v > 0.0f;
}

static PoseGuess guessPose(PersonState s, float dist_cm) {
  if (s == PersonState::NO_TARGET || isnan(dist_cm) || dist_cm <= 0.0f) return PoseGuess::UNKNOWN;
  if (dist_cm < SIT_STAND_THRESHOLD_CM) return PoseGuess::SITTING;
  return PoseGuess::STANDING;
}

// -------- Host UART parsing --------
// Commands (newline-terminated):
//   HM=0 / HM=1
//   FOCUS=-1 (auto) or FOCUS=<cluster_id>
static void applyCommand(const char* line) {
  // Trim leading spaces
  while (*line == ' ' || *line == '\t') line++;

  if (strncmp(line, "HM=", 3) == 0) {
    int v = atoi(line + 3);
    hostHeadMoving = (v != 0);
    return;
  }

  if (strncmp(line, "FOCUS=", 6) == 0) {
    forcedFocusCluster = atoi(line + 6); // -1 means auto
    return;
  }
}

static void pollHostControlUart() {
  static char buf[64];
  static size_t n = 0;

  while (ctrlSerial.available()) {
    char c = (char)ctrlSerial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = 0;
      if (n > 0) applyCommand(buf);
      n = 0;
      continue;
    }

    if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      // overflow: reset
      n = 0;
    }
  }
}

// -------- Focus picking --------
static FocusTarget pickClosestTarget(const PeopleCounting& info) {
  FocusTarget best;
  if (info.targets.empty()) return best;

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

// -------- Printing --------
static void printTargets(const PeopleCounting& info, const FocusTarget& focus) {
  Serial.printf("Targets=%u", (unsigned)info.targets.size());
  if (focus.valid) {
    Serial.printf("  focus(cluster=%d r=%.2fm ang=%.1fdeg)\n", focus.cluster, focus.r_m, focus.bearing_deg);
  } else {
    Serial.println();
  }

  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x*x + y*y);
    float ang = atan2f(x, y) * 180.0f / PI;
    float v = t.dop_index * RANGE_STEP;
    bool isFocus = focus.valid && ((int)i == focus.index);

    Serial.printf("  %s#%u cluster=%d  x=%.2fm y=%.2fm r=%.2fm ang=%.1fdeg  v=%.2fcm/s\n",
                  isFocus ? ">" : " ",
                  (unsigned)(i + 1),
                  t.cluster_index,
                  x, y, r, ang,
                  v);
  }
}

static void printState(PersonState s,
                       PoseGuess p,
                       const FocusTarget& focus,
                       float dist_cm,
                       float br,
                       float hr,
                       int targets,
                       bool human,
                       bool dist_ok,
                       bool br_ok,
                       bool hr_ok,
                       bool headMoving) {
  Serial.printf("[%lu ms] state=%s pose=%s headMoving=%d human=%d targets=%d "
                "dist=%.2fcm(%c) breath=%.2f(%c) heart=%.2f(%c)",
                millis() - t0,
                stateName(s),
                poseName(p),
                headMoving ? 1 : 0,
                human ? 1 : 0,
                targets,
                dist_cm, dist_ok ? 'Y' : 'N',
                br,     br_ok   ? 'Y' : 'N',
                hr,     hr_ok   ? 'Y' : 'N');

  if (forcedFocusCluster >= 0) {
    Serial.printf(" forcedFocus=%d", forcedFocusCluster);
  } else {
    Serial.printf(" forcedFocus=auto");
  }

  if (focus.valid) {
    Serial.printf("  focus(cluster=%d r=%.2fm ang=%.1fdeg v=%.2fcm/s)",
                  focus.cluster, focus.r_m, focus.bearing_deg, focus.speed_cm_s);
  }
  Serial.println();
}

// -----------------------------
// Setup / Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

#ifdef ESP32
  // IMPORTANT: set these pins for your wiring from Reachy host -> XIAO ESP32C6
  // Replace with actual GPIO numbers on the XIAO ESP32C6.
  const int CTRL_RX = 20;  // <-- TODO
  const int CTRL_TX = 21;  // <-- TODO
  ctrlSerial.begin(115200, SERIAL_8N1, CTRL_RX, CTRL_TX);
#endif

  mmWave.begin(&mmWaveSerial);

  t0 = millis();
  Serial.println("= MR60BHA2 start (with host UART control) ===================");
  Serial.println("Host cmds: HM=0/1, FOCUS=-1 or FOCUS=<cluster>");
}

void loop() {
  if (!mmWave.update(100)) return;

  // Read host commands (head motion + optional forced focus)
  pollHostControlUart();

  bool headMoving = hostHeadMoving;

  // Presence / targets
  bool human = mmWave.isHumanDetected();

  PeopleCounting info;
  bool haveTargets = mmWave.getPeopleCountingTargetInfo(info);
  int nTargets = haveTargets ? (int)info.targets.size() : 0;

  // Pick focus target: forced cluster if requested, else closest
  FocusTarget focus;
  if (haveTargets && nTargets > 0) {
    if (forcedFocusCluster >= 0) {
      focus = pickForcedCluster(info, forcedFocusCluster);
      if (!focus.valid) {
        // fallback to closest if cluster isn't present this frame
        focus = pickClosestTarget(info);
      }
    } else {
      focus = pickClosestTarget(info);
    }
  }

  // Distance / vitals
  float dist_cm = 0, br = 0, hr = 0;
  bool dist_ok = mmWave.getDistance(dist_cm);

  bool br_ok = mmWave.getBreathRate(br);
  bool hr_ok = mmWave.getHeartRate(hr);

  // Keep last-good values
  if (dist_ok) lastDist = dist_cm; else dist_cm = lastDist;
  if (br_ok)   lastBR   = br;      else br      = lastBR;
  if (hr_ok)   lastHR   = hr;      else hr      = lastHR;

  // Presence
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

  // Movement (target doppler + head motion)
  bool targetMoving = focus.valid && isfinite(focus.speed_cm_s) && fabsf(focus.speed_cm_s) >= MOVING_CM_S;
  bool moving = headMoving || targetMoving;

  // Near
  bool near = (!isnan(dist_cm) && dist_cm >= NEAR_MIN_DIST_CM && dist_cm <= NEAR_MAX_DIST_CM);

  // Vitals validity gate: only trust when focused on a single target AND head stable
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
  } else if (nTargets > 1) {
    s = PersonState::MULTI_TARGET;     // keep counting/positions here
    vitalsStreak = 0;                 // avoid carrying vitals across multi-target scenes
  } else if (moving) {
    s = PersonState::MOVING;          // head moving => no vitals
    vitalsStreak = 0;
  } else if (near && vitalsStreak >= VITALS_CONFIRM) {
    s = PersonState::RESTING_VITALS;
  } else if (near) {
    s = PersonState::STILL_NEAR;
  } else {
    s = PersonState::PRESENT_FAR;
  }

  PoseGuess p = guessPose(s, dist_cm);

  // Print target list occasionally
  if (haveTargets && nTargets > 0 && (now - lastTargetsPrint > TARGETS_PRINT_MS)) {
    lastTargetsPrint = now;
    printTargets(info, focus);
  }

  // Print state on change OR when focus changes
  bool focusChanged = focus.valid && (focus.cluster != lastFocusCluster);
  if (s != lastState || p != lastPose || focusChanged) {
    printState(s, p, focus, dist_cm, br, hr, nTargets, human, dist_ok, br_ok, hr_ok, headMoving);
    lastState = s;
    lastPose  = p;
    lastFocusCluster = focus.valid ? focus.cluster : -999;
  }
}