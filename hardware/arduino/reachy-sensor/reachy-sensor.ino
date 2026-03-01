// ============================================================================
// reachy-sensor.ino — Firmware for Reachy Mini's health-sensing module
// ============================================================================
//
// WHAT THIS DOES
// --------------
// This firmware reads two sensors and streams their data to a host computer
// (e.g. a Raspberry Pi inside the Reachy Mini robot) over USB serial:
//
//   1. Seeed MR60BHA2 60 GHz mmWave radar  — detects people, measures distance,
//      heart rate, and breathing rate without physical contact.
//   2. BH1750 ambient light sensor (I2C)   — measures room brightness in lux.
//
// The firmware processes raw sensor data into a higher-level "person state"
// (nobody here, someone far away, someone sitting still with good vitals, etc.)
// and sends periodic telemetry events to the host. The host can also send
// commands to tune behavior (e.g. "the robot's head is moving, suppress vitals").
//
// HARDWARE WIRING
// ---------------
//   Microcontroller: Seeed XIAO (ESP32-S3 or similar)
//   mmWave sensor:   connected to hardware UART (Serial1 / mmWaveSerial)
//   BH1750:          connected to I2C (SDA/SCL), address 0x23
//   Host:            USB CDC serial at 115200 baud (Arduino "Serial")
//
// COMMUNICATION PROTOCOL (MMWAVE_PROTO_V1)
// ----------------------------------------
// All communication uses a compact binary protocol — no JSON, no text.
// Each message is framed with COBS encoding (Consistent Overhead Byte Stuffing)
// and terminated by a 0x00 delimiter byte. Inside the COBS frame, the packet
// contains a version byte, message type, sequence number, payload, and a
// CRC-16/CCITT-FALSE checksum for integrity.
//
// See hardware/README.md for the full protocol specification.
// Use hardware/tools/mmwave_decode.py for human-readable serial monitoring.
//
// PERSON STATE MACHINE
// --------------------
// The firmware classifies what the radar sees into one of six states:
//
//   NO_TARGET ──────> PRESENT_FAR ──────> MOVING
//       ^                  |                  |
//       |                  v                  v
//       |             STILL_NEAR ───> RESTING_VITALS
//       |                  ^                  |
//       |                  |                  |
//   MULTI_TARGET <─────────┴──────────────────┘
//
//   NO_TARGET       — nobody detected for ABSENT_HOLD_MS + ABSENT_CONFIRM frames
//   MULTI_TARGET    — more than one person in view (vitals unreliable)
//   PRESENT_FAR     — someone detected but outside the vitals zone (35–150 cm)
//   MOVING          — person or robot head is moving (vitals unreliable)
//   STILL_NEAR      — person in vitals zone, still, but vitals not yet confirmed
//   RESTING_VITALS  — stable vitals confirmed for VITALS_CONFIRM consecutive frames
//
// VITALS GATING
// -------------
// Heart rate and breathing rate are only reported when ALL of these are true:
//   - Single target (or brief gap covered by fallback lock)
//   - Robot head not moving (host signals this via CMD_SET_HM)
//   - Person in the near zone (35–150 cm)
//   - Both rates within physiological guard rails (HR 35–200, BR 4–30 bpm)
//   - Rates stable for VITALS_CONFIRM consecutive frames (hysteresis)
//   - Cached vitals not older than VITALS_CACHE_EXPIRY_MS (2 seconds)
//
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <Wire.h>
#include <BH1750.h>

#include "Seeed_Arduino_mmWave.h"

// --- Platform-specific UART setup ---
// The mmWave sensor communicates over a hardware UART. On ESP32 boards this
// is explicitly constructed; on XIAO/SAMD/etc. it's the built-in Serial1.
#ifdef ESP32
  #include <HardwareSerial.h>
  HardwareSerial mmWaveSerial(0);  // change pins if your wiring differs
#else
  #define mmWaveSerial Serial1
#endif

// --- Compile-time configuration ---
// RANGE_STEP: Doppler velocity scale factor from the mmWave library.
// Override at build time if your sensor variant uses a different value.
#ifndef RANGE_STEP
  #define RANGE_STEP 1.0f
#endif

// MMWAVE_DEBUG_TEXT: Set to 1 to enable human-readable debug prints on Serial.
// WARNING: This interferes with the binary protocol — use only for development.
#ifndef MMWAVE_DEBUG_TEXT
  #define MMWAVE_DEBUG_TEXT 0
#endif

#if MMWAVE_DEBUG_TEXT
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINTLN(x) ((void)0)
#endif

// --- Sensor instances ---
SEEED_MR60BHA2 mmWave;      // 60 GHz radar (presence, distance, heart/breath rate)
BH1750 lightSensor;          // ambient light sensor (lux)

// ============================================================================
// Binary protocol constants (MMWAVE_PROTO_V1)
// ============================================================================
// Message IDs are split into two ranges:
//   0x01–0x7F : commands sent FROM the host TO this device
//   0x80–0xFF : events sent FROM this device TO the host
// See hardware/README.md for full packet layout and payload schemas.

static const uint8_t PROTO_VERSION = 1;  // bump when wire format changes

// --- Host -> Device commands ---
static const uint8_t CMD_SET_HM         = 0x01;  // set head-moving flag (1 byte: 0 or 1)
static const uint8_t CMD_SET_FOCUS      = 0x02;  // lock focus to a cluster index (2 bytes: i16, -1 = auto)
static const uint8_t CMD_SET_BIO_MS     = 0x03;  // set vitals emission interval (2 bytes: u16 ms, min 50)
static const uint8_t CMD_SET_TARGETS_MS = 0x04;  // set targets emission interval (2 bytes: u16 ms, min 50)
static const uint8_t CMD_PING           = 0x05;  // health check (no payload), replies with EVT_PONG

// --- Device -> Host events ---
static const uint8_t EVT_ACK     = 0x81;  // command acknowledged (status + value)
static const uint8_t EVT_ERR     = 0x82;  // command rejected (error code)
static const uint8_t EVT_PONG    = 0x83;  // reply to CMD_PING (uptime in ms)
static const uint8_t EVT_HELLO   = 0x90;  // sent once at boot (protocol version + feature bits)
static const uint8_t EVT_STATE   = 0x91;  // person state snapshot (on change or every 1 s)
static const uint8_t EVT_TARGETS = 0x92;  // radar target list with focus info (throttled)
static const uint8_t EVT_BIO     = 0x93;  // heart rate + breathing rate (throttled)
static const uint8_t EVT_LIGHT   = 0x94;  // ambient light reading in lux (every 1 s)
static const uint8_t EVT_DIAG    = 0x95;  // diagnostic counters (every 10 s)

// --- ACK status codes (in EVT_ACK payload) ---
static const uint8_t ACK_OK      = 0;  // command applied as requested
static const uint8_t ACK_CLAMPED = 1;  // value was out of range and clamped to nearest valid
static const uint8_t ACK_IGNORED = 2;  // command acknowledged but had no effect

// --- Error codes (in EVT_ERR payload) ---
static const uint8_t ERR_UNKNOWN_CMD        = 1;  // unrecognized message type
static const uint8_t ERR_BAD_LEN            = 2;  // payload length doesn't match expected
static const uint8_t ERR_BAD_VALUE          = 3;  // payload value out of allowed range
static const uint8_t ERR_CRC_FAIL           = 4;  // CRC-16 mismatch (data corruption)
static const uint8_t ERR_UNSUPPORTED_VERSION = 5;  // protocol version mismatch

// --- EVT_TARGETS flags bitfield ---
static const uint8_t FLAG_FOCUS_VALID      = 1 << 0;  // focus target fields are populated
static const uint8_t FLAG_TARGETS_TRUNCATED = 1 << 1;  // more targets exist than fit in packet

// Maximum number of individual targets sent per EVT_TARGETS packet.
// The radar can see more, but we cap the wire payload to keep packets small.
static const uint8_t MAX_TARGETS_WIRE = 8;

// --- EVT_HELLO feature bits ---
// The HELLO message advertises which optional hardware is available.
// The host can use these bits to decide which telemetry to expect.
static const uint16_t FEAT_LIGHT_SENSOR = 0x0001;  // BH1750 light sensor is present

// ============================================================================
// Types
// ============================================================================

// PersonState: the firmware's high-level classification of what the radar sees.
// Sent to the host in every EVT_STATE event. The state machine in loop()
// determines which state applies on each sensor cycle.
//
// Transition priority (highest to lowest):
//   1. NO_TARGET       — no presence for ABSENT_HOLD_MS + ABSENT_CONFIRM frames
//   2. MULTI_TARGET    — more than one person detected (vitals unreliable)
//   3. MOVING          — person or robot head is moving
//   4. RESTING_VITALS  — near zone, still, vitals confirmed for VITALS_CONFIRM frames
//   5. STILL_NEAR      — near zone, still, but vitals not yet confirmed
//   6. PRESENT_FAR     — someone detected but outside the near zone
enum class PersonState : uint8_t {
  NO_TARGET       = 0,  // nobody in view
  MULTI_TARGET    = 1,  // multiple people — can't isolate vitals
  PRESENT_FAR     = 2,  // single person, outside 35–150 cm vitals zone
  MOVING          = 3,  // person or head is moving — vitals suppressed
  STILL_NEAR      = 4,  // in vitals zone, still, warming up hysteresis counter
  RESTING_VITALS  = 5   // in vitals zone, still, vitals confirmed stable
};

// PoseGuess: a rough sitting/standing estimate based on distance alone.
// This is a simple heuristic (closer = sitting, farther = standing) and
// should be treated as a hint, not a reliable classification.
enum class PoseGuess : uint8_t {
  UNKNOWN  = 0,  // no target or invalid distance
  SITTING  = 1,  // distance < SIT_STAND_LOWER_CM (50 cm), with hysteresis
  STANDING = 2   // distance >= SIT_STAND_UPPER_CM (60 cm), with hysteresis
};

// FocusTarget: the single radar target that the firmware has selected for
// vitals measurement. Either the host-requested cluster (CMD_SET_FOCUS)
// or the closest target (auto-focus). Populated by pickClosestTarget()
// or pickForcedCluster() each cycle.
struct FocusTarget {
  bool valid = false;        // true if a target was found
  int index = -1;            // index into the radar's target array
  int cluster = -1;          // radar cluster ID (stable across frames)
  float x_m = NAN;           // x position in meters (lateral)
  float y_m = NAN;           // y position in meters (depth / range axis)
  float r_m = NAN;           // radial distance in meters (sqrt(x^2 + y^2))
  float bearing_deg = NAN;   // angle from boresight in degrees (atan2(x, y))
  float speed_cm_s = NAN;    // radial velocity in cm/s (from Doppler)
};

// ============================================================================
// Tuning constants
// ============================================================================
// These control the sensitivity, timing, and guard rails of the state machine.
// All values are compile-time constants. Adjust them to match your deployment
// environment (sensor placement, expected user distance, etc.).

// --- Near zone: the distance range where vitals measurement is reliable ---
// The MR60BHA2 can detect presence at longer range, but heart/breath rate
// accuracy degrades outside this window. Values are in centimeters.
static const float NEAR_MIN_DIST_CM = 35.0f;   // closer than this = too close / noise
static const float NEAR_MAX_DIST_CM = 150.0f;   // farther than this = signal too weak

// --- Movement threshold ---
// If the focus target's radial velocity exceeds this, the person is considered
// "moving" and vitals are suppressed (body motion corrupts the vital signal).
static const float MOVING_CM_S = 8.0f;  // cm/s

// --- Vitals guard rails (beats/breaths per minute) ---
// Values outside these ranges are rejected as sensor noise or artifacts.
// These match the MR60BHA2's practical accuracy range.
static const float BR_MIN = 4.0f,  BR_MAX = 30.0f;   // breathing rate
static const float HR_MIN = 35.0f, HR_MAX = 200.0f;   // heart rate

// --- Hysteresis and debouncing ---
// These prevent flickering between states when the sensor signal is noisy.
//
// ABSENT_HOLD_MS: after losing all presence signals, keep reporting "present"
//   for this many milliseconds (covers brief radar dropouts).
// ABSENT_CONFIRM: additionally require this many consecutive "absent" frames
//   before transitioning to NO_TARGET.
static const uint32_t ABSENT_HOLD_MS = 1200;
static const uint8_t ABSENT_CONFIRM = 8;

// VITALS_CONFIRM: number of consecutive frames where both heart rate AND
//   breathing rate are valid before we transition to RESTING_VITALS and
//   start emitting vitals to the host. Prevents reporting transient spikes.
static const uint8_t VITALS_CONFIRM = 5;

// HUMAN_STABLE_FALLBACK_CONFIRM: when the radar briefly loses target tracking
//   (nTargets drops to 0) but still detects a human, this many consecutive
//   "human detected + head not moving" frames allow vitals to continue.
//   Covers momentary tracking dropouts without resetting the vitals streak.
static const uint8_t HUMAN_STABLE_FALLBACK_CONFIRM = 3;

// TARGET_LOSS_GRACE_MS: maximum duration (ms) that the fallback lock remains
//   active after the last single-target frame. After this, the fallback expires
//   and vitals require a fresh single-target lock.
static const uint32_t TARGET_LOSS_GRACE_MS = 1200;

// --- Pose heuristic ---
// Distance-based guess with hysteresis to prevent flicker near the boundary.
// SITTING→STANDING requires crossing the upper threshold (60 cm).
// STANDING→SITTING requires dropping below the lower threshold (50 cm).
// In the dead zone (50–60 cm) the previous pose is held.
static const float SIT_STAND_UPPER_CM = 60.0f;  // must exceed to transition to STANDING
static const float SIT_STAND_LOWER_CM = 50.0f;  // must drop below to transition to SITTING

// --- Telemetry emission throttles (milliseconds) ---
// These set the minimum interval between consecutive events of each type.
// The host can override BIO_MS and TARGETS_MS via commands (minimum 50 ms).
static const uint32_t TARGETS_MS_DEFAULT = 250;   // radar target list
static const uint32_t BIO_MS_DEFAULT     = 1000;  // heart/breath rate
static const uint32_t LIGHT_MS_DEFAULT   = 1000;  // ambient light

// --- Light sensor I2C address and recovery ---
static const uint8_t BH1750_I2C_ADDR = 0x23;        // ADDR pin LOW (default)
static const uint32_t LIGHT_RETRY_MS = 10000;        // retry BH1750 init every 10 s on failure

// ============================================================================
// Global state
// ============================================================================
// All mutable state lives here. The firmware is single-threaded (Arduino
// superloop), so no locking is needed. Variables are grouped by function.

// --- Timing ---
static uint32_t t0 = 0;       // millis() at boot — all event timestamps are relative to this
static uint16_t txSeq = 0;    // monotonically increasing sequence number for outgoing packets

// --- Sensor value caching ---
// The radar doesn't always return fresh readings every cycle. When a reading
// is missing, we fall back to the last known good value — but only within
// an expiry window to avoid reporting stale data.
static float lastDist = NAN;  // last good distance (cm) — no expiry (distance is stable)
static float lastBR = NAN;    // last good breathing rate (bpm) — expires after VITALS_CACHE_EXPIRY_MS
static float lastHR = NAN;    // last good heart rate (bpm) — expires after VITALS_CACHE_EXPIRY_MS
static uint32_t lastBRUpdateMs = 0;  // timestamp of last fresh BR reading
static uint32_t lastHRUpdateMs = 0;  // timestamp of last fresh HR reading
static const uint32_t VITALS_CACHE_EXPIRY_MS = 2000;  // max age (ms) before cached vitals are invalidated
static const uint32_t STATE_MIN_INTERVAL_MS = 200;    // minimum ms between change-triggered EVT_STATE

// --- Presence tracking ---
static uint32_t lastPresenceMs = 0;    // last millis() when any presence signal was detected
static uint8_t absentStreak = 0;       // consecutive frames with no presence signal
static uint8_t vitalsStreak = 0;       // consecutive frames with valid vitals (both HR and BR)
static uint8_t humanStableStreak = 0;  // consecutive frames with human detected + head still
static uint32_t lastSingleTargetMs = 0;  // last millis() when exactly 1 target was seen
static bool seenSingleTarget = false;    // true once we've seen at least one single-target frame

// --- Emission throttle timestamps ---
// Each telemetry type has its own emission interval. These track when the
// last packet of each type was sent to enforce the throttle.
static uint32_t lastTargetsEmitMs = 0;
static uint32_t lastBioEmitMs = 0;
static uint32_t lastStateEmitMs = 0;
static uint32_t lastLightEmitMs = 0;

// --- Diagnostic counters ---
// Cumulative health metrics emitted periodically via EVT_DIAG so the host
// can detect firmware-side failures (FW-HIGH-1, FW-HIGH-2).
static uint32_t mmwaveFailCount = 0;        // total mmWave.update() failures since boot
static uint16_t mmwaveConsecutiveFails = 0;  // current consecutive failure streak
static uint16_t txDropCount = 0;             // total COBS/overflow packet drops since boot
static uint32_t lastDiagEmitMs = 0;
static const uint32_t DIAG_MS = 10000;       // diagnostic emission interval (10 s)

// --- Host-controlled settings ---
// These are modified by host commands (CMD_SET_HM, CMD_SET_FOCUS, etc.)
static bool hostHeadMoving = false;     // true when the robot head is in motion (suppresses vitals)
static bool lightSensorReady = false;   // true when BH1750 is initialized; retried periodically if false
static uint32_t lastLightRetryMs = 0;  // last millis() when BH1750 re-init was attempted
static int forcedFocusCluster = -1;     // host-requested cluster index (-1 = auto-select closest)

// --- Configurable emission intervals (host can change via CMD_SET_BIO_MS / CMD_SET_TARGETS_MS) ---
static uint32_t TARGETS_MS = TARGETS_MS_DEFAULT;
static uint32_t BIO_MS = BIO_MS_DEFAULT;

// --- Previous-state tracking for change detection ---
// EVT_STATE is emitted on state change OR every 1 second. These track the
// previous values so we can detect changes.
static PersonState prevS = PersonState::NO_TARGET;
static PoseGuess prevP = PoseGuess::UNKNOWN;
static bool prevHM = false;  // previous head-moving flag
static int prevN = -1;       // previous target count

// --- Serial protocol buffers ---
// COBS encoding expands data by at most 1 byte per 254, plus a delimiter.
//
// Buffer sizing rationale (worst case = EVT_TARGETS at MAX_TARGETS_WIRE):
//   Payload:  22 header bytes + MAX_TARGETS_WIRE(8) * 12 bytes/target = 118 bytes
//   Packet:   1 (version) + 1 (type) + 2 (seq) + 118 (payload) + 2 (CRC) = 124 bytes
//   COBS:     124 + ceil(124/254) + 1 (delimiter) = 126 bytes
//
// All buffers use 384+ bytes, giving >3x headroom over worst case.
// If MAX_TARGETS_WIRE increases, the static_assert below will catch overflow.
static uint8_t rxEncodedBuf[384];   // incoming COBS-encoded bytes (accumulated until 0x00 delimiter)
static size_t rxEncodedLen = 0;     // current fill level of rxEncodedBuf
static bool rxOverflow = false;     // true if incoming frame exceeded buffer capacity
static uint8_t rxDecodedBuf[384];   // decoded packet (after COBS removal)

static uint8_t txPacketBuf[512];    // outgoing packet before COBS encoding (header + payload + CRC)
static uint8_t txEncodedBuf[640];   // COBS-encoded output (txPacketBuf expands slightly)
static uint8_t txPayloadBuf[384];   // scratch space for building event payloads

// Compile-time check: txPayloadBuf must fit the largest possible payload (EVT_TARGETS).
static const size_t EVT_TARGETS_HEADER_BYTES = 22;   // t_ms(4) + forcedCluster(2) + focus(12) + flags(1) + nWire(1)
static const size_t EVT_TARGETS_PER_TARGET   = 12;   // cluster(2) + x(2) + y(2) + r(2) + bearing(2) + v(2)
static_assert(
  sizeof(txPayloadBuf) >= EVT_TARGETS_HEADER_BYTES + MAX_TARGETS_WIRE * EVT_TARGETS_PER_TARGET,
  "txPayloadBuf too small for MAX_TARGETS_WIRE targets — increase buffer or reduce MAX_TARGETS_WIRE"
);

// ============================================================================
// Small helpers
// ============================================================================

// Returns true if v is a real positive number (not NaN, not Inf, not zero/negative).
static bool isFinitePositive(float v) { return isfinite(v) && v > 0.0f; }

// Sitting/standing heuristic with hysteresis to prevent flicker.
// Uses prevP to hold the current pose inside the dead zone (50–60 cm).
// Returns UNKNOWN if there's no target or the distance is invalid.
static PoseGuess guessPose(PersonState s, float dist_cm) {
  if (s == PersonState::NO_TARGET || isnan(dist_cm) || dist_cm <= 0.0f) return PoseGuess::UNKNOWN;
  if (dist_cm >= SIT_STAND_UPPER_CM) return PoseGuess::STANDING;
  if (dist_cm < SIT_STAND_LOWER_CM) return PoseGuess::SITTING;
  // Inside dead zone: hold previous pose, default to SITTING if unknown
  return (prevP == PoseGuess::STANDING) ? PoseGuess::STANDING : PoseGuess::SITTING;
}

// CRC-16/CCITT-FALSE: the checksum algorithm used for packet integrity.
// Poly=0x1021, Init=0xFFFF, no reflection, no final XOR.
// Must match the host-side implementation (Python: mmwave_protocol.py).
static uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

// ============================================================================
// Buffer serialization helpers (little-endian)
// ============================================================================
// These write typed values into a byte buffer at the current index (*idx)
// and advance the index. Used to build packet payloads field by field.

static void appendU8(uint8_t* buf, size_t* idx, uint8_t value) { buf[(*idx)++] = value; }

static void appendU16LE(uint8_t* buf, size_t* idx, uint16_t value) {
  buf[(*idx)++] = (uint8_t)(value & 0xFF);
  buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
}

static void appendI16LE(uint8_t* buf, size_t* idx, int16_t value) {
  appendU16LE(buf, idx, (uint16_t)value);
}

static void appendU32LE(uint8_t* buf, size_t* idx, uint32_t value) {
  buf[(*idx)++] = (uint8_t)(value & 0xFF);
  buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
  buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFF);
  buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFF);
}

static void appendI32LE(uint8_t* buf, size_t* idx, int32_t value) {
  appendU32LE(buf, idx, (uint32_t)value);
}

// Write a 32-bit float as its raw IEEE-754 bytes (little-endian).
static void appendF32LE(uint8_t* buf, size_t* idx, float value) {
  uint32_t raw = 0;
  memcpy(&raw, &value, sizeof(raw));
  appendU32LE(buf, idx, raw);
}

// Read helpers for parsing incoming host commands.
static uint16_t readU16LE(const uint8_t* buf) {
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static int16_t readI16LE(const uint8_t* buf) {
  return (int16_t)readU16LE(buf);
}

// ============================================================================
// COBS codec (Consistent Overhead Byte Stuffing)
// ============================================================================
// COBS eliminates 0x00 bytes from the data so that 0x00 can be used as an
// unambiguous frame delimiter. Overhead is at most 1 byte per 254 input bytes.
//
// Wire format:  [COBS-encoded payload bytes] [0x00 delimiter]
//
// The host and device both use this same encoding. The Python counterpart
// is in mmwave_protocol.py.

// Encode `input` (may contain 0x00 bytes) into `output` (no 0x00 bytes).
// Returns the encoded length, or 0 on overflow.
static size_t cobsEncode(const uint8_t* input, size_t inputLen, uint8_t* output, size_t outputCap) {
  if (outputCap == 0) return 0;

  size_t readIdx = 0;
  size_t writeIdx = 1;
  size_t codeIdx = 0;
  uint8_t code = 1;
  output[0] = 0;

  while (readIdx < inputLen) {
    uint8_t byte = input[readIdx++];
    if (byte == 0) {
      if (codeIdx >= outputCap) return 0;
      output[codeIdx] = code;
      code = 1;
      codeIdx = writeIdx++;
      if (codeIdx >= outputCap) return 0;
      continue;
    }

    if (writeIdx >= outputCap) return 0;
    output[writeIdx++] = byte;
    code++;
    if (code == 0xFF) {
      if (codeIdx >= outputCap) return 0;
      output[codeIdx] = code;
      code = 1;
      codeIdx = writeIdx++;
      if (codeIdx >= outputCap) return 0;
    }
  }

  if (codeIdx >= outputCap) return 0;
  output[codeIdx] = code;
  return writeIdx;
}

// Decode COBS-encoded `input` back into original data (with 0x00 bytes restored).
// Returns true on success, false on malformed input or overflow.
static bool cobsDecode(const uint8_t* input, size_t inputLen, uint8_t* output, size_t* outLen, size_t outputCap) {
  if (inputLen == 0) return false;

  size_t readIdx = 0;
  size_t writeIdx = 0;

  while (readIdx < inputLen) {
    uint8_t code = input[readIdx++];
    if (code == 0) return false;

    size_t next = readIdx + (size_t)code - 1;
    if (next > inputLen) return false;

    while (readIdx < next) {
      if (writeIdx >= outputCap) return false;
      output[writeIdx++] = input[readIdx++];
    }

    if (code < 0xFF && readIdx < inputLen) {
      if (writeIdx >= outputCap) return false;
      output[writeIdx++] = 0;
    }
  }

  *outLen = writeIdx;
  return true;
}

// ============================================================================
// Numeric scaling for wire format
// ============================================================================
// The protocol sends most values as scaled integers to save bandwidth.
// For example, a distance of 1.234 m is sent as 1234 mm (i16, scale=1000).
// NaN/Inf values are mapped to sentinel values (0 or 0xFFFF) so the host
// can distinguish "no data" from "zero".

// Scale a float to a signed 16-bit integer, clamping to [-32768, 32767].
// NaN/Inf maps to 0 (not a sentinel — use toU16ScaledOrNull for nullable fields).
static int16_t toI16Scaled(float value, float scale) {
  if (!isfinite(value)) return 0;
  float scaled = value * scale;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)lroundf(scaled);
}

// Scale a float to an unsigned 16-bit integer, clamping to [0, 65534].
// NaN/Inf maps to 0xFFFF (the null sentinel). The host interprets 0xFFFF as "no data".
static uint16_t toU16ScaledOrNull(float value, float scale) {
  if (!isfinite(value)) return 0xFFFF;
  float scaled = value * scale;
  if (scaled < 0.0f) scaled = 0.0f;
  if (scaled > 65534.0f) scaled = 65534.0f;
  return (uint16_t)lroundf(scaled);
}

// ============================================================================
// Packet framing and transmission
// ============================================================================

// Build a complete packet (header + payload + CRC), COBS-encode it, and send
// it over USB serial followed by the 0x00 frame delimiter.
//
// Packet layout before COBS encoding:
//   [version:u8] [msgType:u8] [seq:u16] [payloadLen:u16] [payload...] [crc:u16]
//
// Drops the packet if payload exceeds buffer capacity or COBS encoding
// overflows, incrementing txDropCount so the host can detect lost frames
// via EVT_DIAG.
static void sendFrame(uint8_t msgType, const uint8_t* payload, size_t payloadLen) {
  if (payloadLen > sizeof(txPayloadBuf)) { txDropCount++; return; }

  size_t pktLen = 0;
  appendU8(txPacketBuf, &pktLen, PROTO_VERSION);
  appendU8(txPacketBuf, &pktLen, msgType);
  appendU16LE(txPacketBuf, &pktLen, txSeq++);
  appendU16LE(txPacketBuf, &pktLen, (uint16_t)payloadLen);

  if (payloadLen > 0) {
    memcpy(&txPacketBuf[pktLen], payload, payloadLen);
    pktLen += payloadLen;
  }

  uint16_t crc = crc16CcittFalse(txPacketBuf, pktLen);
  appendU16LE(txPacketBuf, &pktLen, crc);

  size_t encodedLen = cobsEncode(txPacketBuf, pktLen, txEncodedBuf, sizeof(txEncodedBuf));
  if (encodedLen == 0) { txDropCount++; return; }

  Serial.write(txEncodedBuf, encodedLen);
  Serial.write((uint8_t)0x00);
}

// ============================================================================
// Event emitters
// ============================================================================
// Each emitXxx() function builds a payload for one event type and sends it.
// These are called from loop() (telemetry) or applyBinaryCommand() (responses).

// EVT_ACK: confirm a host command was processed.
//   cmdId      — which command this acknowledges
//   statusCode — ACK_OK, ACK_CLAMPED, or ACK_IGNORED
//   value      — the applied value (e.g. the new BIO_MS interval)
static void emitAck(uint8_t cmdId, uint8_t statusCode, int32_t value) {
  size_t n = 0;
  appendU8(txPayloadBuf, &n, cmdId);
  appendU8(txPayloadBuf, &n, statusCode);
  appendI32LE(txPayloadBuf, &n, value);
  sendFrame(EVT_ACK, txPayloadBuf, n);
}

// EVT_ERR: reject a host command (bad length, bad value, CRC failure, etc.)
static void emitErr(uint8_t cmdId, uint8_t errCode) {
  size_t n = 0;
  appendU8(txPayloadBuf, &n, cmdId);
  appendU8(txPayloadBuf, &n, errCode);
  sendFrame(EVT_ERR, txPayloadBuf, n);
}

// EVT_PONG: reply to CMD_PING with the device's uptime in milliseconds.
// Used by the host to verify the serial link is alive and measure round-trip latency.
static void emitPong(uint32_t t_ms) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  sendFrame(EVT_PONG, txPayloadBuf, n);
}

// EVT_HELLO: sent once at boot (and on DTR reset) to announce protocol version
// and available hardware features. The host uses this to verify compatibility
// and discover which sensors are present.
static void emitHello() {
  uint16_t features = 0;
  if (lightSensorReady) features |= FEAT_LIGHT_SENSOR;
  size_t n = 0;
  appendU8(txPayloadBuf, &n, PROTO_VERSION);
  appendU16LE(txPayloadBuf, &n, features);
  sendFrame(EVT_HELLO, txPayloadBuf, n);
}

// EVT_STATE: snapshot of the current person state. Sent on every state change
// and at least once per second as a heartbeat. This is the primary event the
// host uses to understand what the sensor sees.
//
// Wire payload (12 bytes):
//   u32 t_ms, u8 state, u8 pose, u8 head_moving, u8 human,
//   u8 n_targets, u8 dist_ok, u16 dist_mm (0xFFFF = null)
static void emitState(
    uint32_t t_ms,
    PersonState s,
    PoseGuess p,
    bool headMoving,
    bool human,
    int nTargets,
    float dist_cm,
    bool dist_ok) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  appendU8(txPayloadBuf, &n, (uint8_t)s);
  appendU8(txPayloadBuf, &n, (uint8_t)p);
  appendU8(txPayloadBuf, &n, headMoving ? 1 : 0);
  appendU8(txPayloadBuf, &n, human ? 1 : 0);
  appendU8(txPayloadBuf, &n, (uint8_t)max(0, min(255, nTargets)));
  appendU8(txPayloadBuf, &n, dist_ok ? 1 : 0);
  appendU16LE(txPayloadBuf, &n, toU16ScaledOrNull(dist_cm, 10.0f));
  sendFrame(EVT_STATE, txPayloadBuf, n);
}

// EVT_BIO: heart rate and breathing rate telemetry.
//   allowed — conditions permit vitals (single target, head still)
//   valid   — both HR and BR passed all gates (allowed + in range + hysteresis)
//   br_ok   — breathing rate specifically is trustworthy and confirmed
//   hr_ok   — heart rate specifically is trustworthy and confirmed
//   br/hr   — the raw rate values in bpm (sent as centi-bpm on wire; 0xFFFF = null)
//
// The host should only display vitals when br_ok/hr_ok are true.
// The "allowed" and "valid" flags provide context for why vitals may be absent.
static void emitBio(uint32_t t_ms, bool allowed, bool valid, float br, bool br_ok, float hr, bool hr_ok) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  appendU8(txPayloadBuf, &n, allowed ? 1 : 0);
  appendU8(txPayloadBuf, &n, valid ? 1 : 0);
  appendU8(txPayloadBuf, &n, br_ok ? 1 : 0);
  appendU8(txPayloadBuf, &n, hr_ok ? 1 : 0);
  uint16_t brWire = br_ok ? toU16ScaledOrNull(br, 100.0f) : 0xFFFF;
  uint16_t hrWire = hr_ok ? toU16ScaledOrNull(hr, 100.0f) : 0xFFFF;
  appendU16LE(txPayloadBuf, &n, brWire);
  appendU16LE(txPayloadBuf, &n, hrWire);
  sendFrame(EVT_BIO, txPayloadBuf, n);
}

// EVT_LIGHT: ambient light reading from the BH1750 sensor.
// Sent every LIGHT_MS_DEFAULT (1 second), independent of radar timing.
// When the sensor is absent or returns an error, valid=false and lux=NaN.
static void emitLight(uint32_t t_ms, bool valid, float lux) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  appendU8(txPayloadBuf, &n, valid ? 1 : 0);
  appendF32LE(txPayloadBuf, &n, valid ? lux : NAN);
  sendFrame(EVT_LIGHT, txPayloadBuf, n);
}

// EVT_DIAG: periodic diagnostic counters so the host can detect firmware-side
// failures (mmWave sensor errors, dropped TX packets).
// Emitted every DIAG_MS (10 s) regardless of sensor state.
static void emitDiag(uint32_t t_ms, uint32_t mmwFails, uint16_t mmwConsecFails, uint16_t txDrops) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  appendU32LE(txPayloadBuf, &n, mmwFails);
  appendU16LE(txPayloadBuf, &n, mmwConsecFails);
  appendU16LE(txPayloadBuf, &n, txDrops);
  sendFrame(EVT_DIAG, txPayloadBuf, n);
}

// EVT_TARGETS: the list of radar targets (people/objects) currently detected,
// plus the selected focus target's detailed position. Sent at TARGETS_MS interval
// (default 250 ms, configurable via CMD_SET_TARGETS_MS).
//
// Wire layout:
//   Header: t_ms, forced_focus_cluster, focus fields (cluster/x/y/r/bearing/velocity),
//           flags, n_targets
//   Then n_targets entries of: cluster, x_mm, y_mm, r_mm, bearing_cdeg, velocity
//
// Targets with NaN coordinates are skipped. If the radar reports more targets
// than MAX_TARGETS_WIRE (8), only the first 8 are sent and the TRUNCATED flag is set.
static void emitTargets(uint32_t t_ms, const PeopleCounting& info, const FocusTarget& focus) {
  uint8_t flags = 0;
  if (focus.valid) flags |= FLAG_FOCUS_VALID;

  int nTargets = (int)info.targets.size();
  uint8_t nWire = (uint8_t)min(nTargets, (int)MAX_TARGETS_WIRE);
  if (nTargets > (int)MAX_TARGETS_WIRE) flags |= FLAG_TARGETS_TRUNCATED;

  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  appendI16LE(txPayloadBuf, &n, (int16_t)forcedFocusCluster);

  int16_t focusCluster = -1;
  int16_t focusXmm = 0;
  int16_t focusYmm = 0;
  uint16_t focusRmm = 0;
  int16_t focusBearingCdeg = 0;
  int16_t focusVx10 = 0;

  if (focus.valid) {
    focusCluster = (int16_t)focus.cluster;
    focusXmm = toI16Scaled(focus.x_m, 1000.0f);
    focusYmm = toI16Scaled(focus.y_m, 1000.0f);
    focusRmm = toU16ScaledOrNull(focus.r_m, 1000.0f);
    focusBearingCdeg = toI16Scaled(focus.bearing_deg, 100.0f);
    focusVx10 = toI16Scaled(focus.speed_cm_s, 10.0f);
  }

  appendI16LE(txPayloadBuf, &n, focusCluster);
  appendI16LE(txPayloadBuf, &n, focusXmm);
  appendI16LE(txPayloadBuf, &n, focusYmm);
  appendU16LE(txPayloadBuf, &n, focusRmm);
  appendI16LE(txPayloadBuf, &n, focusBearingCdeg);
  appendI16LE(txPayloadBuf, &n, focusVx10);
  appendU8(txPayloadBuf, &n, flags);
  // Reserve space for nWire count — we'll fill it after filtering
  size_t nWireOffset = n;
  appendU8(txPayloadBuf, &n, 0);  // placeholder

  uint8_t nEncoded = 0;
  for (uint8_t i = 0; i < nWire; i++) {
    const auto& t = info.targets[i];
    float x = t.x_point;
    float y = t.y_point;
    if (!isfinite(x) || !isfinite(y)) continue;

    float r = sqrtf(x * x + y * y);
    float bearing = atan2f(x, y) * 180.0f / PI;
    float v = t.dop_index * RANGE_STEP;

    appendI16LE(txPayloadBuf, &n, (int16_t)t.cluster_index);
    appendI16LE(txPayloadBuf, &n, toI16Scaled(x, 1000.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(y, 1000.0f));
    appendU16LE(txPayloadBuf, &n, toU16ScaledOrNull(r, 1000.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(bearing, 100.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(v, 10.0f));
    nEncoded++;
  }

  // Patch the actual count into the reserved slot
  txPayloadBuf[nWireOffset] = nEncoded;

  sendFrame(EVT_TARGETS, txPayloadBuf, n);
}

// ============================================================================
// Focus target selection
// ============================================================================
// The firmware picks one "focus" target for vitals measurement. Two strategies:
//   1. Auto-focus (default): pick the closest target by radial distance.
//   2. Forced cluster: the host locks onto a specific cluster ID via CMD_SET_FOCUS.
//      If that cluster disappears, we fall back to auto-focus.
//
// The focus target's position and velocity are included in every EVT_TARGETS
// packet so the host knows who the vitals belong to.

// Auto-focus: find the target with the smallest radial distance.
// Skips targets with non-finite coordinates (NaN from sensor noise).
static FocusTarget pickClosestTarget(const PeopleCounting& info) {
  FocusTarget best;
  float best_r = 1e9f;

  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x * x + y * y);
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

// Forced focus: find the target matching the host-requested cluster ID.
// Returns an invalid FocusTarget if the cluster is not currently visible
// (the caller falls back to pickClosestTarget in that case).
static FocusTarget pickForcedCluster(const PeopleCounting& info, int cluster) {
  FocusTarget out;
  if (cluster < 0) return out;

  for (size_t i = 0; i < info.targets.size(); i++) {
    const auto& t = info.targets[i];
    if (t.cluster_index != cluster) continue;

    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x * x + y * y);

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

// ============================================================================
// Host command processing
// ============================================================================
// The host sends commands to control firmware behavior. Each command is
// validated (length, value range) and either acknowledged (EVT_ACK) or
// rejected (EVT_ERR). Commands take effect immediately.

// Dispatch a validated command to the appropriate handler.
// Called from handleDecodedPacket() after version, length, and CRC checks pass.
static void applyBinaryCommand(uint8_t msgType, const uint8_t* payload, size_t payloadLen) {
  // CMD_SET_HM: tell the firmware whether the robot's head is currently moving.
  // When head-moving=true, vitals are suppressed (physical motion corrupts the
  // radar's micro-motion detection used for heart/breath measurement).
  if (msgType == CMD_SET_HM) {
    if (payloadLen != 1) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    uint8_t hm = payload[0];
    if (hm != 0 && hm != 1) {
      emitErr(msgType, ERR_BAD_VALUE);
      return;
    }
    hostHeadMoving = (hm == 1);
    emitAck(msgType, ACK_OK, hostHeadMoving ? 1 : 0);
    return;
  }

  // CMD_SET_FOCUS: lock the focus target to a specific radar cluster index.
  // Use -1 to return to auto-focus (closest target). The host typically sets
  // this after identifying which person to measure via the target list.
  if (msgType == CMD_SET_FOCUS) {
    if (payloadLen != 2) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    forcedFocusCluster = (int)readI16LE(payload);
    emitAck(msgType, ACK_OK, (int32_t)forcedFocusCluster);
    return;
  }

  // CMD_SET_BIO_MS: set the minimum interval (ms) between EVT_BIO emissions.
  // Clamped to a minimum of 50 ms to prevent flooding the serial link.
  // ACK_CLAMPED is returned if the requested value was below the minimum.
  if (msgType == CMD_SET_BIO_MS) {
    if (payloadLen != 2) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    uint16_t req = readU16LE(payload);
    uint16_t applied = (req < 50) ? 50 : req;
    uint8_t status = (applied == req) ? ACK_OK : ACK_CLAMPED;
    BIO_MS = applied;
    emitAck(msgType, status, (int32_t)BIO_MS);
    return;
  }

  // CMD_SET_TARGETS_MS: same as CMD_SET_BIO_MS but for EVT_TARGETS emission interval.
  if (msgType == CMD_SET_TARGETS_MS) {
    if (payloadLen != 2) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    uint16_t req = readU16LE(payload);
    uint16_t applied = (req < 50) ? 50 : req;
    uint8_t status = (applied == req) ? ACK_OK : ACK_CLAMPED;
    TARGETS_MS = applied;
    emitAck(msgType, status, (int32_t)TARGETS_MS);
    return;
  }

  // CMD_PING: health check. The host sends this to verify the link is alive.
  // Also used at connection startup for protocol version handshake (the host
  // sends PING and checks that the PONG comes in a valid MMWAVE_PROTO_V1 frame).
  if (msgType == CMD_PING) {
    if (payloadLen != 0) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    emitPong(millis() - t0);
    return;
  }

  emitErr(msgType, ERR_UNKNOWN_CMD);
}

// Validate a decoded packet's structure (version, length, CRC) and dispatch
// to applyBinaryCommand(). Sends EVT_ERR for any validation failure.
//
// Expected packet layout:
//   [0] version  [1] msgType  [2-3] seq  [4-5] payloadLen  [6..] payload  [..] crc16
//   Minimum packet size: 6 (header) + 0 (payload) + 2 (CRC) = 8 bytes.
static void handleDecodedPacket(const uint8_t* packet, size_t packetLen) {
  if (packetLen < 8) {
    emitErr(0, ERR_BAD_LEN);
    return;
  }

  uint8_t version = packet[0];
  uint8_t msgType = packet[1];
  uint16_t payloadLen = readU16LE(&packet[4]);
  if (version != PROTO_VERSION) {
    emitErr(msgType, ERR_UNSUPPORTED_VERSION);
    return;
  }

  size_t expectedLen = 6 + (size_t)payloadLen + 2;
  if (packetLen != expectedLen) {
    emitErr(msgType, ERR_BAD_LEN);
    return;
  }

  const uint8_t* payload = &packet[6];
  uint16_t crcExpected = readU16LE(&packet[6 + payloadLen]);
  uint16_t crcActual = crc16CcittFalse(packet, 6 + payloadLen);
  if (crcExpected != crcActual) {
    emitErr(msgType, ERR_CRC_FAIL);
    return;
  }

  applyBinaryCommand(msgType, payload, payloadLen);
}

// Read bytes from USB serial and assemble them into COBS frames.
//
// Bytes are accumulated in rxEncodedBuf until a 0x00 delimiter arrives,
// which marks the end of a frame. The frame is then COBS-decoded and
// passed to handleDecodedPacket() for validation and dispatch.
//
// Overflow handling: if a frame exceeds the buffer size, the rxOverflow
// flag is set and all bytes are discarded until the next 0x00 delimiter.
static void pollHostUsbSerial() {
  while (Serial.available()) {
    int byteRead = Serial.read();
    if (byteRead < 0) break;
    uint8_t byte = (uint8_t)byteRead;

    if (byte == 0x00) {
      if (rxOverflow) {
        rxOverflow = false;
        rxEncodedLen = 0;
        emitErr(0, ERR_BAD_LEN);
        continue;
      }

      if (rxEncodedLen == 0) {
        continue;
      }

      size_t decodedLen = 0;
      bool ok = cobsDecode(rxEncodedBuf, rxEncodedLen, rxDecodedBuf, &decodedLen, sizeof(rxDecodedBuf));
      rxEncodedLen = 0;
      if (!ok) {
        emitErr(0, ERR_BAD_LEN);
        continue;
      }

      handleDecodedPacket(rxDecodedBuf, decodedLen);
      continue;
    }

    if (rxOverflow) continue;
    if (rxEncodedLen < sizeof(rxEncodedBuf)) {
      rxEncodedBuf[rxEncodedLen++] = byte;
    } else {
      rxOverflow = true;
    }
  }
}

// ============================================================================
// Arduino entry points: setup() and loop()
// ============================================================================

// setup() runs once at power-on (or USB DTR reset).
// Initializes all peripherals and announces the firmware to the host.
void setup() {
  Serial.begin(115200);  // USB CDC serial to host
  delay(200);            // let USB enumerate before sending data

  Wire.begin();          // I2C bus for BH1750 light sensor
  lightSensorReady = lightSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR, &Wire);

  mmWave.begin(&mmWaveSerial);  // start mmWave radar on hardware UART

  t0 = millis();   // record boot time — all event timestamps are relative to this
  emitHello();     // announce protocol version and feature bits to host
}

// loop() runs repeatedly as fast as possible (Arduino superloop).
//
// Each iteration does three things:
//   1. Process any pending host commands from USB serial.
//   2. Read the BH1750 light sensor (every 1 second).
//   3. Read the mmWave radar, run the state machine, and emit telemetry.
//
// Step 3 only runs when the radar has new data (mmWave.update() returns true).
// Steps 1 and 2 run every iteration regardless, so the host link stays responsive
// and light readings continue even when the radar is slow.
void loop() {

  // ── Step 1: Process host commands ──────────────────────────────────────────
  // This runs every iteration so commands (PING, SET_HM, etc.) are handled
  // promptly, even when the radar is still processing.
  pollHostUsbSerial();

  // ── Step 2: Read ambient light sensor ──────────────────────────────────────
  // The BH1750 is read on its own 1-second timer, independent of the radar.
  // If the sensor failed to initialize (at boot or due to I2C glitch), we
  // periodically retry begin() every LIGHT_RETRY_MS. On success, normal
  // readings resume immediately. On failure, EVT_LIGHT with valid=false
  // continues so the host knows the sensor is absent.
  uint32_t lightNow = millis();
  if (!lightSensorReady && (lightNow - lastLightRetryMs >= LIGHT_RETRY_MS)) {
    lastLightRetryMs = lightNow;
    lightSensorReady = lightSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR, &Wire);
  }
  if (lightNow - lastLightEmitMs >= LIGHT_MS_DEFAULT) {
    lastLightEmitMs = lightNow;
    float lux = NAN;
    bool lightValid = false;
    if (lightSensorReady) {
      lux = lightSensor.readLightLevel();
      lightValid = isfinite(lux) && lux >= 0.0f;
    }
    emitLight(lightNow - t0, lightValid, lux);
  }

  // ── Step 2b: Emit diagnostics ──────────────────────────────────────────────
  // Periodic health counters so the host can detect firmware-side failures.
  // Runs before mmWave.update() so it fires even during sensor failure streaks.
  {
    uint32_t diagNow = millis();
    if (diagNow - lastDiagEmitMs >= DIAG_MS) {
      lastDiagEmitMs = diagNow;
      emitDiag(diagNow - t0, mmwaveFailCount, mmwaveConsecutiveFails, txDropCount);
    }
  }

  // ── Step 3: Read radar and run state machine ───────────────────────────────
  // mmWave.update() polls the radar's UART. Returns true when a complete
  // data frame has been received and parsed. If false, skip the rest —
  // the previous state remains valid and we'll try again next iteration.
  if (!mmWave.update(100)) {
    mmwaveFailCount++;
    mmwaveConsecutiveFails++;
    return;
  }
  mmwaveConsecutiveFails = 0;  // reset streak on success

  // Snapshot the host-controlled head-moving flag (may change mid-cycle
  // via pollHostUsbSerial, but we use a consistent value for this frame).
  bool headMoving = hostHeadMoving;

  // ── 3a: Read presence and target data ──────────────────────────────────────
  bool human = mmWave.isHumanDetected();  // binary: radar thinks a human is present

  PeopleCounting info;
  bool haveTargets = mmWave.getPeopleCountingTargetInfo(info);
  int nTargets = haveTargets ? (int)info.targets.size() : 0;

  // Pick the focus target for vitals measurement.
  // If the host has locked a cluster (CMD_SET_FOCUS), try that first;
  // fall back to the closest target if the locked cluster isn't visible.
  FocusTarget focus;
  if (haveTargets && nTargets > 0) {
    if (forcedFocusCluster >= 0) {
      focus = pickForcedCluster(info, forcedFocusCluster);
      if (!focus.valid) focus = pickClosestTarget(info);
    } else {
      focus = pickClosestTarget(info);
    }
  }

  // ── 3b: Read distance and vitals from radar ────────────────────────────────
  // The radar doesn't always return fresh values every cycle. When a reading
  // is missing (_ok = false), we fall back to the last known good value —
  // but only within an expiry window to avoid reporting stale data.
  float dist_cm = NAN, br = NAN, hr = NAN;
  bool dist_ok = mmWave.getDistance(dist_cm);
  bool br_ok = mmWave.getBreathRate(br);
  bool hr_ok = mmWave.getHeartRate(hr);

  uint32_t now = millis();

  // Distance fallback: use the last good distance if the current read failed.
  // Distance is relatively stable so there's no expiry — a person doesn't
  // teleport between frames.
  if (dist_ok && isfinite(dist_cm)) {
    lastDist = dist_cm;
  } else {
    dist_cm = lastDist;
  }
  // After fallback, dist_ok must reflect whether we actually have a usable
  // distance (not just whether this specific read succeeded).
  dist_ok = isFinitePositive(dist_cm);

  // Breathing rate fallback: use cached value if fresh, expire after 2 seconds.
  // Vitals change faster than distance, so stale values are dangerous.
  if (br_ok && isfinite(br)) {
    lastBR = br;
    lastBRUpdateMs = now;
  } else if ((now - lastBRUpdateMs) <= VITALS_CACHE_EXPIRY_MS) {
    br = lastBR;  // still fresh enough — use cached value
  } else {
    lastBR = NAN;  // expired — clear cache
    br = NAN;
  }

  // Heart rate fallback: same expiry logic as breathing rate.
  if (hr_ok && isfinite(hr)) {
    lastHR = hr;
    lastHRUpdateMs = now;
  } else if ((now - lastHRUpdateMs) <= VITALS_CACHE_EXPIRY_MS) {
    hr = lastHR;
  } else {
    lastHR = NAN;
    hr = NAN;
  }

  // ── 3c: Presence detection ─────────────────────────────────────────────────
  // A person is considered "present" if ANY of these signals are active.
  // This is deliberately inclusive to avoid premature NO_TARGET transitions.
  bool present_now = human || (nTargets > 0) || (dist_ok && isFinitePositive(dist_cm)) || (br_ok && isFinitePositive(br)) ||
                     (hr_ok && isFinitePositive(hr));

  if (present_now) {
    lastPresenceMs = now;
    absentStreak = 0;
  } else {
    absentStreak++;
  }
  // Don't transition to NO_TARGET until both a time window AND a frame count
  // have elapsed — this prevents flickering on momentary radar dropouts.
  bool presence_recent = (now - lastPresenceMs) < ABSENT_HOLD_MS;

  // ── 3d: Movement detection ─────────────────────────────────────────────────
  // Two sources of movement: the focus target's Doppler velocity (person moving)
  // and the host's head-moving flag (robot head rotating). Either one suppresses
  // vitals because body/sensor motion corrupts the micro-motion signal.
  bool targetMoving = focus.valid && isfinite(focus.speed_cm_s) && fabsf(focus.speed_cm_s) >= MOVING_CM_S;
  bool moving = headMoving || targetMoving;

  // ── 3e: Near zone check ────────────────────────────────────────────────────
  // Vitals are only reliable within the near zone (35–150 cm). Outside this
  // range the radar's micro-motion sensitivity is too low for heart/breath.
  bool near = (!isnan(dist_cm) && dist_cm >= NEAR_MIN_DIST_CM && dist_cm <= NEAR_MAX_DIST_CM);

  // ── 3f: Vitals validity gating ─────────────────────────────────────────────
  // This is the most complex part of the state machine. Vitals require:
  //   - Single target isolation (or a brief fallback when tracking drops out)
  //   - Robot head stationary
  //   - Both rates within physiological guard rails
  //   - Stability confirmed over VITALS_CONFIRM consecutive frames
  //
  // The "fallback target lock" handles a common scenario: the radar briefly
  // loses target tracking (nTargets drops to 0) while still detecting a human.
  // If this happens within TARGET_LOSS_GRACE_MS of the last single-target frame,
  // and the human signal has been stable, we keep vitals running instead of
  // resetting the hysteresis counter.
  bool singleTarget = (nTargets == 1);
  if (singleTarget) {
    seenSingleTarget = true;
    lastSingleTargetMs = now;
  }

  // Track consecutive frames with a stable human signal (for fallback lock).
  if (human && !headMoving) {
    humanStableStreak = (humanStableStreak < 255) ? (uint8_t)(humanStableStreak + 1) : (uint8_t)255;
  } else {
    humanStableStreak = 0;
  }

  // Fallback lock: allow vitals to continue briefly when target tracking drops.
  bool singleTargetRecent = seenSingleTarget && ((now - lastSingleTargetMs) <= TARGET_LOSS_GRACE_MS);
  bool fallbackTargetLock = (!singleTarget) && (nTargets == 0) && singleTargetRecent &&
                            (humanStableStreak >= HUMAN_STABLE_FALLBACK_CONFIRM);

  // Guard rail check: reject physiologically impossible values.
  bool br_valid = br_ok && isfinite(br) && (br >= BR_MIN) && (br <= BR_MAX);
  bool hr_valid = hr_ok && isfinite(hr) && (hr >= HR_MIN) && (hr <= HR_MAX);

  // The three-level vitals gate:
  //   allowed → conditions permit measurement (single target + head still)
  //   valid   → allowed AND both rates pass guard rails
  //   streak  → valid for VITALS_CONFIRM consecutive frames (hysteresis)
  bool vitals_allowed = !headMoving && (singleTarget || fallbackTargetLock);
  bool vitals_valid = vitals_allowed && br_valid && hr_valid;
  vitalsStreak = vitals_valid ? (vitalsStreak < 255 ? (uint8_t)(vitalsStreak + 1) : (uint8_t)255) : 0;

  // ── 3g: State machine decision ─────────────────────────────────────────────
  // Priority order matters: NO_TARGET > MULTI_TARGET > MOVING > RESTING_VITALS > STILL_NEAR > PRESENT_FAR
  PersonState s;
  if (!presence_recent && absentStreak >= ABSENT_CONFIRM) {
    s = PersonState::NO_TARGET;
    vitalsStreak = 0;    // reset hysteresis — next person starts fresh
  } else if (nTargets > 1) {
    s = PersonState::MULTI_TARGET;
    vitalsStreak = 0;    // can't isolate vitals with multiple people
  } else if (moving) {
    s = PersonState::MOVING;
    vitalsStreak = 0;    // motion corrupts vitals signal
  } else if (near && vitalsStreak >= VITALS_CONFIRM) {
    s = PersonState::RESTING_VITALS;  // stable vitals confirmed!
  } else if (near) {
    s = PersonState::STILL_NEAR;      // warming up — vitals not yet confirmed
  } else {
    s = PersonState::PRESENT_FAR;     // someone is there but too far for vitals
  }

  PoseGuess p = guessPose(s, dist_cm);
  uint32_t t_ms = now - t0;  // event timestamp relative to boot

  // ── 3h: Emit telemetry events ──────────────────────────────────────────────

  // EVT_TARGETS: radar target list (throttled by TARGETS_MS).
  // Only sent when there are actual targets to report.
  if (haveTargets && nTargets > 0 && (now - lastTargetsEmitMs >= TARGETS_MS)) {
    lastTargetsEmitMs = now;
    emitTargets(t_ms, info, focus);
  }

  // EVT_STATE: on state change (rate-limited to STATE_MIN_INTERVAL_MS) or
  // at least every 1 second as a heartbeat. The rate limit caps change-triggered
  // emissions to max 5/second to prevent host flooding on rapid oscillation.
  bool stateChanged = (s != prevS) || (p != prevP) || (headMoving != prevHM) || (nTargets != prevN);
  bool intervalOk = (now - lastStateEmitMs >= STATE_MIN_INTERVAL_MS);
  if ((stateChanged && intervalOk) || (now - lastStateEmitMs > 1000)) {
    lastStateEmitMs = now;
    emitState(t_ms, s, p, headMoving, human, nTargets, dist_cm, dist_ok);
    prevS = s;
    prevP = p;
    prevHM = headMoving;
    prevN = nTargets;
  }

  // EVT_BIO: vitals telemetry (throttled by BIO_MS).
  // Always emitted on schedule, even when vitals are unavailable — the
  // allowed/valid/br_ok/hr_ok flags tell the host why data is missing.
  if (now - lastBioEmitMs >= BIO_MS) {
    lastBioEmitMs = now;
    // Require hysteresis confirmation before marking individual vitals as OK.
    // This prevents emitting a single lucky reading as trustworthy.
    bool br_emit_ok = vitals_allowed && br_valid && (vitalsStreak >= VITALS_CONFIRM);
    bool hr_emit_ok = vitals_allowed && hr_valid && (vitalsStreak >= VITALS_CONFIRM);
    emitBio(t_ms, vitals_allowed, vitals_valid, br, br_emit_ok, hr, hr_emit_ok);
  }
}
