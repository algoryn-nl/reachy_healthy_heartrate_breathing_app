"""Binary protocol helpers for Reachy mmWave transport."""

from __future__ import annotations
import math
import struct
from typing import Any


PROTO_VERSION = 1

# Host -> device commands
CMD_SET_HM = 0x01
CMD_SET_FOCUS = 0x02
CMD_SET_BIO_MS = 0x03
CMD_SET_TARGETS_MS = 0x04
CMD_PING = 0x05

# Device -> host events
EVT_ACK = 0x81
EVT_ERR = 0x82
EVT_PONG = 0x83
EVT_HELLO = 0x90
EVT_STATE = 0x91
EVT_TARGETS = 0x92
EVT_BIO = 0x93
EVT_LIGHT = 0x94

# ACK status codes
ACK_OK = 0
ACK_CLAMPED = 1
ACK_IGNORED = 2

# ERR codes
ERR_UNKNOWN_CMD = 1
ERR_BAD_LEN = 2
ERR_BAD_VALUE = 3
ERR_CRC_FAIL = 4
ERR_UNSUPPORTED_VERSION = 5

# Target event flags
FLAG_FOCUS_VALID = 1 << 0
FLAG_TARGETS_TRUNCATED = 1 << 1

STATE_ENUM_TO_NAME = {
    0: "NO_TARGET",
    1: "MULTI_TARGET",
    2: "PRESENT_FAR",
    3: "MOVING",
    4: "STILL_NEAR",
    5: "RESTING_VITALS",
}

POSE_ENUM_TO_NAME = {
    0: "UNKNOWN",
    1: "SITTING",
    2: "STANDING",
}


class ProtocolError(ValueError):
    """Raised when a frame or payload is invalid."""


def crc16_ccitt_false(data: bytes) -> int:
    """Compute CRC16/CCITT-FALSE."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """COBS encode payload bytes (delimiter byte excluded)."""
    if not data:
        return b"\x01"

    out = bytearray()
    code_index = 0
    out.append(0)
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue

        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(encoded: bytes) -> bytes:
    """COBS decode encoded bytes (without delimiter)."""
    if not encoded:
        raise ProtocolError("empty cobs frame")

    out = bytearray()
    index = 0
    length = len(encoded)

    while index < length:
        code = encoded[index]
        if code == 0:
            raise ProtocolError("invalid cobs code 0")
        index += 1

        end = index + code - 1
        if end > length:
            raise ProtocolError("cobs code exceeds frame length")

        out.extend(encoded[index:end])
        index = end
        if code < 0xFF and index < length:
            out.append(0)

    return bytes(out)


def encode_frame(msg_type: int, payload: bytes = b"", seq: int = 0, version: int = PROTO_VERSION) -> bytes:
    """Build one framed packet ready to write on serial."""
    payload_len = len(payload)
    header = struct.pack("<BBHH", version & 0xFF, msg_type & 0xFF, seq & 0xFFFF, payload_len & 0xFFFF)
    packet_wo_crc = header + payload
    crc = crc16_ccitt_false(packet_wo_crc)
    packet = packet_wo_crc + struct.pack("<H", crc)
    return cobs_encode(packet) + b"\x00"


def decode_frame(encoded_frame: bytes) -> tuple[int, int, int, bytes]:
    """Decode one framed packet payload (without trailing delimiter)."""
    raw = cobs_decode(encoded_frame)
    if len(raw) < 8:
        raise ProtocolError("packet too short")

    version, msg_type, seq, payload_len = struct.unpack_from("<BBHH", raw, 0)
    expected_len = 6 + payload_len + 2
    if len(raw) != expected_len:
        raise ProtocolError("payload length mismatch")

    payload = raw[6 : 6 + payload_len]
    crc_expected = struct.unpack_from("<H", raw, 6 + payload_len)[0]
    crc_actual = crc16_ccitt_false(raw[: 6 + payload_len])
    if crc_expected != crc_actual:
        raise ProtocolError("crc mismatch")

    return version, msg_type, seq, payload


def extract_encoded_frames(buffer: bytearray) -> list[bytes]:
    """Extract 0x00-delimited encoded frames from a mutable byte buffer."""
    frames: list[bytes] = []
    while True:
        idx = buffer.find(0)
        if idx < 0:
            break
        frame = bytes(buffer[:idx])
        del buffer[: idx + 1]
        if frame:
            frames.append(frame)
    return frames


def pack_cmd_set_hm(hm: bool | int) -> bytes:
    """Build payload for CMD_SET_HM."""
    return struct.pack("<B", 1 if hm else 0)


def pack_cmd_set_focus(cluster: int) -> bytes:
    """Build payload for CMD_SET_FOCUS."""
    return struct.pack("<h", int(cluster))


def pack_cmd_set_bio_ms(ms: int) -> bytes:
    """Build payload for CMD_SET_BIO_MS."""
    return struct.pack("<H", int(ms) & 0xFFFF)


def pack_cmd_set_targets_ms(ms: int) -> bytes:
    """Build payload for CMD_SET_TARGETS_MS."""
    return struct.pack("<H", int(ms) & 0xFFFF)


def decode_event(msg_type: int, payload: bytes) -> dict[str, Any]:
    """Decode payload into a normalized event dictionary."""
    if msg_type == EVT_ACK:
        if len(payload) != 6:
            raise ProtocolError("bad ack payload length")
        cmd_id, status_code, value = struct.unpack("<BBi", payload)
        return {"type": "ack", "cmd_id": cmd_id, "status_code": status_code, "value": value}

    if msg_type == EVT_ERR:
        if len(payload) != 2:
            raise ProtocolError("bad err payload length")
        cmd_id, err_code = struct.unpack("<BB", payload)
        return {"type": "err", "cmd_id": cmd_id, "err_code": err_code}

    if msg_type == EVT_PONG:
        if len(payload) != 4:
            raise ProtocolError("bad pong payload length")
        (t_ms,) = struct.unpack("<I", payload)
        return {"type": "pong", "t_ms": t_ms}

    if msg_type == EVT_HELLO:
        if len(payload) != 3:
            raise ProtocolError("bad hello payload length")
        proto_version, feature_bits = struct.unpack("<BH", payload)
        return {"type": "hello", "proto_version": proto_version, "feature_bits": feature_bits}

    if msg_type == EVT_STATE:
        if len(payload) != 12:
            raise ProtocolError("bad state payload length")
        t_ms, state_enum, pose_enum, head_moving, human, n_targets, dist_new, dist_mm = struct.unpack(
            "<IBBBBBBH", payload
        )
        dist_cm = None if dist_mm == 0xFFFF else dist_mm / 10.0
        return {
            "type": "state",
            "t_ms": t_ms,
            "state": STATE_ENUM_TO_NAME.get(state_enum, f"UNKNOWN_{state_enum}"),
            "pose": POSE_ENUM_TO_NAME.get(pose_enum, f"UNKNOWN_{pose_enum}"),
            "head_moving": int(head_moving),
            "human": int(human),
            "n_targets": int(n_targets),
            "dist_cm": dist_cm,
            "dist_new": int(dist_new),
        }

    if msg_type == EVT_BIO:
        if len(payload) != 12:
            raise ProtocolError("bad bio payload length")
        t_ms, allowed, valid, br_new, hr_new, br_centi, hr_centi = struct.unpack("<IBBBBHH", payload)
        br = None if br_centi == 0xFFFF else br_centi / 100.0
        hr = None if hr_centi == 0xFFFF else hr_centi / 100.0
        return {
            "type": "bio",
            "t_ms": t_ms,
            "allowed": int(allowed),
            "valid": int(valid),
            "br": br,
            "br_new": int(br_new),
            "hr": hr,
            "hr_new": int(hr_new),
        }

    if msg_type == EVT_LIGHT:
        if len(payload) != 9:
            raise ProtocolError("bad light payload length")
        t_ms, valid, lux = struct.unpack("<IBf", payload)
        lux_out: float | None
        if int(valid) == 0 or not math.isfinite(lux):
            lux_out = None
        else:
            lux_out = float(lux)
        return {
            "type": "light",
            "t_ms": t_ms,
            "valid": int(valid),
            "lux": lux_out,
        }

    if msg_type == EVT_TARGETS:
        header_len = 20
        target_len = 12
        if len(payload) < header_len:
            raise ProtocolError("bad targets payload length")
        t_ms, forced_focus, focus_cluster, focus_x_mm, focus_y_mm, focus_r_mm, focus_bearing_cdeg, focus_v_x10, flags, n_targets = struct.unpack(
            "<IhhhhHhhBB", payload[:header_len]
        )
        expected_len = header_len + int(n_targets) * target_len
        if len(payload) != expected_len:
            raise ProtocolError("targets payload length mismatch")

        focus = None
        if flags & FLAG_FOCUS_VALID:
            focus = {
                "cluster": int(focus_cluster),
                "x": focus_x_mm / 1000.0,
                "y": focus_y_mm / 1000.0,
                "r": focus_r_mm / 1000.0,
                "bearing": focus_bearing_cdeg / 100.0,
                "v": focus_v_x10 / 10.0,
            }

        targets = []
        offset = header_len
        for _ in range(int(n_targets)):
            cluster, x_mm, y_mm, r_mm, bearing_cdeg, v_x10 = struct.unpack("<hhhHhh", payload[offset : offset + target_len])
            offset += target_len
            targets.append(
                {
                    "cluster": int(cluster),
                    "x": x_mm / 1000.0,
                    "y": y_mm / 1000.0,
                    "r": r_mm / 1000.0,
                    "bearing": bearing_cdeg / 100.0,
                    "v": v_x10 / 10.0,
                }
            )

        return {
            "type": "targets",
            "t_ms": t_ms,
            "n": int(n_targets),
            "n_targets": int(n_targets),
            "forced_focus": int(forced_focus),
            "focus": focus,
            "targets": targets,
            "targets_truncated": bool(flags & FLAG_TARGETS_TRUNCATED),
        }

    return {"type": "unknown", "msg_type": msg_type, "payload": payload}
