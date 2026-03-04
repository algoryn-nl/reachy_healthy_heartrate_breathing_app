// reachy_codec.h — Pure protocol functions shared between firmware and host tests.
// Included by reachy-sensor.ino (Arduino) and reachy_codec_shim.c (host test build).
#ifndef REACHY_CODEC_H
#define REACHY_CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

// --- Frame delimiter ---
static const uint8_t RC_FRAME_DELIMITER = 0x00;

// --- Guards ---
static inline int rc_is_finite_positive(float v) {
    return isfinite(v) && v > 0.0f;
}

// --- CRC-16/CCITT-FALSE ---
// Poly=0x1021, Init=0xFFFF, no reflection, no final XOR.
// Must match the host-side Python implementation (mmwave_protocol.py).
static inline uint16_t rc_crc16_ccitt_false(const uint8_t* data, size_t len) {
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

// --- Buffer serialization (little-endian) ---
static inline void rc_append_u8(uint8_t* buf, size_t* idx, uint8_t value) {
    buf[(*idx)++] = value;
}

static inline void rc_append_u16le(uint8_t* buf, size_t* idx, uint16_t value) {
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void rc_append_i16le(uint8_t* buf, size_t* idx, int16_t value) {
    rc_append_u16le(buf, idx, (uint16_t)value);
}

static inline void rc_append_u32le(uint8_t* buf, size_t* idx, uint32_t value) {
    buf[(*idx)++] = (uint8_t)(value & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 16) & 0xFF);
    buf[(*idx)++] = (uint8_t)((value >> 24) & 0xFF);
}

static inline void rc_append_i32le(uint8_t* buf, size_t* idx, int32_t value) {
    rc_append_u32le(buf, idx, (uint32_t)value);
}

static inline void rc_append_f32le(uint8_t* buf, size_t* idx, float value) {
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(raw));
    rc_append_u32le(buf, idx, raw);
}

static inline uint16_t rc_read_u16le(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline int16_t rc_read_i16le(const uint8_t* buf) {
    return (int16_t)rc_read_u16le(buf);
}

// --- COBS codec (Consistent Overhead Byte Stuffing) ---
// Eliminates 0x00 bytes so 0x00 can be used as an unambiguous frame delimiter.
// Returns encoded length, or 0 on overflow.
static inline size_t rc_cobs_encode(const uint8_t* input, size_t inputLen,
                                     uint8_t* output, size_t outputCap) {
    if (outputCap == 0) return 0;
    size_t readIdx = 0, writeIdx = 1, codeIdx = 0;
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

// Returns 1 on success, 0 on malformed input or overflow.
static inline int rc_cobs_decode(const uint8_t* input, size_t inputLen,
                                  uint8_t* output, size_t* outLen, size_t outputCap) {
    if (inputLen == 0) return 0;
    size_t readIdx = 0, writeIdx = 0;

    while (readIdx < inputLen) {
        uint8_t code = input[readIdx++];
        if (code == 0) return 0;
        size_t next = readIdx + (size_t)code - 1;
        if (next > inputLen) return 0;
        while (readIdx < next) {
            if (writeIdx >= outputCap) return 0;
            output[writeIdx++] = input[readIdx++];
        }
        if (code < 0xFF && readIdx < inputLen) {
            if (writeIdx >= outputCap) return 0;
            output[writeIdx++] = 0;
        }
    }
    *outLen = writeIdx;
    return 1;
}

// --- Numeric scaling for wire format ---
// Scale float to i16, clamping to [-32768, 32767]. NaN/Inf → 0.
static inline int16_t rc_to_i16_scaled(float value, float scale) {
    if (!isfinite(value)) return 0;
    float scaled = value * scale;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)lroundf(scaled);
}

// Scale float to u16, clamping to [0, 65534]. NaN/Inf → 0xFFFF (null sentinel).
static inline uint16_t rc_to_u16_scaled_or_null(float value, float scale) {
    if (!isfinite(value)) return 0xFFFF;
    float scaled = value * scale;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 65534.0f) scaled = 65534.0f;
    return (uint16_t)lroundf(scaled);
}

#ifdef __cplusplus
}
#endif

#endif // REACHY_CODEC_H
