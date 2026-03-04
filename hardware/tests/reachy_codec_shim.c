// reachy_codec_shim.c — Thin wrappers exporting reachy_codec.h functions for ctypes.
// Compiled into a shared library (libreachy_codec.so/.dylib) for pytest cross-validation.
#include "../arduino/reachy-sensor/reachy_codec.h"

// Re-export each static inline as a real symbol so ctypes can find it.

int shim_is_finite_positive(float v) {
    return rc_is_finite_positive(v);
}

uint16_t shim_crc16_ccitt_false(const uint8_t* data, size_t len) {
    return rc_crc16_ccitt_false(data, len);
}

size_t shim_cobs_encode(const uint8_t* input, size_t inputLen,
                        uint8_t* output, size_t outputCap) {
    return rc_cobs_encode(input, inputLen, output, outputCap);
}

int shim_cobs_decode(const uint8_t* input, size_t inputLen,
                     uint8_t* output, size_t* outLen, size_t outputCap) {
    return rc_cobs_decode(input, inputLen, output, outLen, outputCap);
}

void shim_append_u8(uint8_t* buf, size_t* idx, uint8_t value) {
    rc_append_u8(buf, idx, value);
}

void shim_append_u16le(uint8_t* buf, size_t* idx, uint16_t value) {
    rc_append_u16le(buf, idx, value);
}

void shim_append_i16le(uint8_t* buf, size_t* idx, int16_t value) {
    rc_append_i16le(buf, idx, value);
}

void shim_append_u32le(uint8_t* buf, size_t* idx, uint32_t value) {
    rc_append_u32le(buf, idx, value);
}

void shim_append_i32le(uint8_t* buf, size_t* idx, int32_t value) {
    rc_append_i32le(buf, idx, value);
}

void shim_append_f32le(uint8_t* buf, size_t* idx, float value) {
    rc_append_f32le(buf, idx, value);
}

uint16_t shim_read_u16le(const uint8_t* buf) {
    return rc_read_u16le(buf);
}

int16_t shim_read_i16le(const uint8_t* buf) {
    return rc_read_i16le(buf);
}

int16_t shim_to_i16_scaled(float value, float scale) {
    return rc_to_i16_scaled(value, scale);
}

uint16_t shim_to_u16_scaled_or_null(float value, float scale) {
    return rc_to_u16_scaled_or_null(value, scale);
}

uint8_t shim_frame_delimiter(void) {
    return RC_FRAME_DELIMITER;
}
