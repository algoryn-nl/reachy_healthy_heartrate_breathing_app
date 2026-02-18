// Reachy-ready MR60BHA2 demo:
// - mmWave sensor over HW UART (mmWaveSerial)
// - Host control + telemetry over USB Serial (Serial)
// - Binary COBS framed packets + CRC16

#include <Arduino.h>
#include <math.h>
#include <string.h>

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

#ifndef MMWAVE_DEBUG_TEXT
  #define MMWAVE_DEBUG_TEXT 0
#endif

#if MMWAVE_DEBUG_TEXT
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINTLN(x) ((void)0)
#endif

SEEED_MR60BHA2 mmWave;

// -----------------------------
// Binary protocol constants
// -----------------------------
static const uint8_t PROTO_VERSION = 1;

// Host -> device commands
static const uint8_t CMD_SET_HM = 0x01;
static const uint8_t CMD_SET_FOCUS = 0x02;
static const uint8_t CMD_SET_BIO_MS = 0x03;
static const uint8_t CMD_SET_TARGETS_MS = 0x04;
static const uint8_t CMD_PING = 0x05;

// Device -> host events
static const uint8_t EVT_ACK = 0x81;
static const uint8_t EVT_ERR = 0x82;
static const uint8_t EVT_PONG = 0x83;
static const uint8_t EVT_HELLO = 0x90;
static const uint8_t EVT_STATE = 0x91;
static const uint8_t EVT_TARGETS = 0x92;
static const uint8_t EVT_BIO = 0x93;

// ACK status codes
static const uint8_t ACK_OK = 0;
static const uint8_t ACK_CLAMPED = 1;
static const uint8_t ACK_IGNORED = 2;

// ERR codes
static const uint8_t ERR_UNKNOWN_CMD = 1;
static const uint8_t ERR_BAD_LEN = 2;
static const uint8_t ERR_BAD_VALUE = 3;
static const uint8_t ERR_CRC_FAIL = 4;
static const uint8_t ERR_UNSUPPORTED_VERSION = 5;

static const uint8_t FLAG_FOCUS_VALID = 1 << 0;
static const uint8_t FLAG_TARGETS_TRUNCATED = 1 << 1;

static const uint8_t MAX_TARGETS_WIRE = 8;

// -----------------------------
// Types
// -----------------------------
enum class PersonState : uint8_t {
  NO_TARGET = 0,
  MULTI_TARGET = 1,
  PRESENT_FAR = 2,
  MOVING = 3,
  STILL_NEAR = 4,
  RESTING_VITALS = 5
};

enum class PoseGuess : uint8_t {
  UNKNOWN = 0,
  SITTING = 1,
  STANDING = 2
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
static const float BR_MIN = 4.0f, BR_MAX = 30.0f;
static const float HR_MIN = 35.0f, HR_MAX = 200.0f;

// Hysteresis
static const uint32_t ABSENT_HOLD_MS = 1200;
static const uint8_t ABSENT_CONFIRM = 8;
static const uint8_t VITALS_CONFIRM = 5;
static const uint8_t HUMAN_STABLE_FALLBACK_CONFIRM = 3;
static const uint32_t TARGET_LOSS_GRACE_MS = 1200;

// Pose heuristic (distance-based demo)
static const float SIT_STAND_THRESHOLD_CM = 55.0f;

// Output throttles
static const uint32_t TARGETS_MS_DEFAULT = 250;
static const uint32_t BIO_MS_DEFAULT = 1000;

// -----------------------------
// Globals
// -----------------------------
static uint32_t t0 = 0;
static uint16_t txSeq = 0;

static float lastDist = NAN, lastBR = NAN, lastHR = NAN;
static uint32_t lastPresenceMs = 0;
static uint8_t absentStreak = 0;
static uint8_t vitalsStreak = 0;
static uint8_t humanStableStreak = 0;
static uint32_t lastSingleTargetMs = 0;
static bool seenSingleTarget = false;

static uint32_t lastTargetsEmitMs = 0;
static uint32_t lastBioEmitMs = 0;
static uint32_t lastStateEmitMs = 0;

static bool hostHeadMoving = false;
static int forcedFocusCluster = -1;

static uint32_t TARGETS_MS = TARGETS_MS_DEFAULT;
static uint32_t BIO_MS = BIO_MS_DEFAULT;

static PersonState prevS = PersonState::NO_TARGET;
static PoseGuess prevP = PoseGuess::UNKNOWN;
static bool prevHM = false;
static int prevN = -1;

// RX/TX buffers for framed protocol
static uint8_t rxEncodedBuf[384];
static size_t rxEncodedLen = 0;
static bool rxOverflow = false;
static uint8_t rxDecodedBuf[384];

static uint8_t txPacketBuf[512];
static uint8_t txEncodedBuf[640];
static uint8_t txPayloadBuf[384];

// -----------------------------
// Small helpers
// -----------------------------
static bool isFinitePositive(float v) { return isfinite(v) && v > 0.0f; }

static PoseGuess guessPose(PersonState s, float dist_cm) {
  if (s == PersonState::NO_TARGET || isnan(dist_cm) || dist_cm <= 0.0f) return PoseGuess::UNKNOWN;
  return (dist_cm < SIT_STAND_THRESHOLD_CM) ? PoseGuess::SITTING : PoseGuess::STANDING;
}

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

static uint16_t readU16LE(const uint8_t* buf) {
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static int16_t readI16LE(const uint8_t* buf) {
  return (int16_t)readU16LE(buf);
}

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

static int16_t toI16Scaled(float value, float scale) {
  if (!isfinite(value)) return 0;
  float scaled = value * scale;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)lroundf(scaled);
}

static uint16_t toU16ScaledOrNull(float value, float scale) {
  if (!isfinite(value)) return 0xFFFF;
  float scaled = value * scale;
  if (scaled < 0.0f) scaled = 0.0f;
  if (scaled > 65534.0f) scaled = 65534.0f;
  return (uint16_t)lroundf(scaled);
}

static void sendFrame(uint8_t msgType, const uint8_t* payload, size_t payloadLen) {
  if (payloadLen > sizeof(txPayloadBuf)) return;

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
  if (encodedLen == 0) return;

  Serial.write(txEncodedBuf, encodedLen);
  Serial.write((uint8_t)0x00);
}

static void emitAck(uint8_t cmdId, uint8_t statusCode, int32_t value) {
  size_t n = 0;
  appendU8(txPayloadBuf, &n, cmdId);
  appendU8(txPayloadBuf, &n, statusCode);
  appendI32LE(txPayloadBuf, &n, value);
  sendFrame(EVT_ACK, txPayloadBuf, n);
}

static void emitErr(uint8_t cmdId, uint8_t errCode) {
  size_t n = 0;
  appendU8(txPayloadBuf, &n, cmdId);
  appendU8(txPayloadBuf, &n, errCode);
  sendFrame(EVT_ERR, txPayloadBuf, n);
}

static void emitPong(uint32_t t_ms) {
  size_t n = 0;
  appendU32LE(txPayloadBuf, &n, t_ms);
  sendFrame(EVT_PONG, txPayloadBuf, n);
}

static void emitHello() {
  size_t n = 0;
  appendU8(txPayloadBuf, &n, PROTO_VERSION);
  appendU16LE(txPayloadBuf, &n, 0);  // feature bits
  sendFrame(EVT_HELLO, txPayloadBuf, n);
}

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
  appendU8(txPayloadBuf, &n, nWire);

  for (uint8_t i = 0; i < nWire; i++) {
    const auto& t = info.targets[i];
    float x = t.x_point;
    float y = t.y_point;
    float r = sqrtf(x * x + y * y);
    float bearing = atan2f(x, y) * 180.0f / PI;
    float v = t.dop_index * RANGE_STEP;

    appendI16LE(txPayloadBuf, &n, (int16_t)t.cluster_index);
    appendI16LE(txPayloadBuf, &n, toI16Scaled(x, 1000.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(y, 1000.0f));
    appendU16LE(txPayloadBuf, &n, toU16ScaledOrNull(r, 1000.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(bearing, 100.0f));
    appendI16LE(txPayloadBuf, &n, toI16Scaled(v, 10.0f));
  }

  sendFrame(EVT_TARGETS, txPayloadBuf, n);
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

// -----------------------------
// Host command processing
// -----------------------------
static void applyBinaryCommand(uint8_t msgType, const uint8_t* payload, size_t payloadLen) {
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

  if (msgType == CMD_SET_FOCUS) {
    if (payloadLen != 2) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    forcedFocusCluster = (int)readI16LE(payload);
    emitAck(msgType, ACK_OK, (int32_t)forcedFocusCluster);
    return;
  }

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

// -----------------------------
// Setup / Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  mmWave.begin(&mmWaveSerial);

  t0 = millis();
  emitHello();
}

void loop() {
  // Process host commands continuously, independent of sensor update timing.
  pollHostUsbSerial();
  if (!mmWave.update(100)) return;

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
  bool br_ok = mmWave.getBreathRate(br);
  bool hr_ok = mmWave.getHeartRate(hr);

  // Keep last-good values (do not turn missing values into zeros)
  if (dist_ok && isfinite(dist_cm))
    lastDist = dist_cm;
  else
    dist_cm = lastDist;

  if (br_ok && isfinite(br))
    lastBR = br;
  else
    br = lastBR;

  if (hr_ok && isfinite(hr))
    lastHR = hr;
  else
    hr = lastHR;

  // Presence (any signal)
  bool present_now = human || (nTargets > 0) || (dist_ok && isFinitePositive(dist_cm)) || (br_ok && isFinitePositive(br)) ||
                     (hr_ok && isFinitePositive(hr));

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

  // Vitals validity gate: trust single-target mode, and allow a short fallback
  // when target tracking briefly drops to nTargets=0.
  bool singleTarget = (nTargets == 1);
  if (singleTarget) {
    seenSingleTarget = true;
    lastSingleTargetMs = now;
  }

  if (human && !headMoving) {
    humanStableStreak = (humanStableStreak < 255) ? (uint8_t)(humanStableStreak + 1) : (uint8_t)255;
  } else {
    humanStableStreak = 0;
  }

  bool singleTargetRecent = seenSingleTarget && ((now - lastSingleTargetMs) <= TARGET_LOSS_GRACE_MS);
  bool fallbackTargetLock = (!singleTarget) && (nTargets == 0) && singleTargetRecent &&
                            (humanStableStreak >= HUMAN_STABLE_FALLBACK_CONFIRM);

  bool br_valid = br_ok && isfinite(br) && (br >= BR_MIN) && (br <= BR_MAX);
  bool hr_valid = hr_ok && isfinite(hr) && (hr >= HR_MIN) && (hr <= HR_MAX);

  bool vitals_allowed = !headMoving && (singleTarget || fallbackTargetLock);
  bool vitals_valid = vitals_allowed && br_valid && hr_valid;
  vitalsStreak = vitals_valid ? (vitalsStreak < 255 ? (uint8_t)(vitalsStreak + 1) : (uint8_t)255) : 0;

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

  if (haveTargets && nTargets > 0 && (now - lastTargetsEmitMs >= TARGETS_MS)) {
    lastTargetsEmitMs = now;
    emitTargets(t_ms, info, focus);
  }

  bool stateChanged = (s != prevS) || (p != prevP) || (headMoving != prevHM) || (nTargets != prevN);
  if (stateChanged || (now - lastStateEmitMs > 1000)) {
    lastStateEmitMs = now;
    emitState(t_ms, s, p, headMoving, human, nTargets, dist_cm, dist_ok);
    prevS = s;
    prevP = p;
    prevHM = headMoving;
    prevN = nTargets;
  }

  if (now - lastBioEmitMs >= BIO_MS) {
    lastBioEmitMs = now;
    bool br_emit_ok = vitals_allowed && br_valid;
    bool hr_emit_ok = vitals_allowed && hr_valid;
    emitBio(t_ms, vitals_allowed, vitals_valid, br, br_emit_ok, hr, hr_emit_ok);
  }
}
