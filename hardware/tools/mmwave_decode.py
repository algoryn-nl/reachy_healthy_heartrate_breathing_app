"""Decode mmWave binary protocol frames from serial or capture files."""

from __future__ import annotations
import re
import sys
import json
import argparse
from typing import Iterable
from pathlib import Path

from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
    PROTO_VERSION,
    ProtocolError,
    decode_event,
    decode_frame,
    extract_encoded_frames,
)


def _looks_like_hex_dump(raw: bytes) -> bool:
    if not raw:
        return False
    if b"\x00" in raw:
        return False
    text = raw.decode("utf-8", errors="ignore")
    tokens = re.findall(r"[0-9A-Fa-f]{2}", text)
    return len(tokens) >= max(8, len(text.strip().split()) // 2)


def _parse_hex_dump(raw: bytes) -> bytes:
    text = raw.decode("utf-8", errors="ignore")
    tokens = re.findall(r"[0-9A-Fa-f]{2}", text)
    return bytes(int(token, 16) for token in tokens)


def _fv(val: object, fmt: str = "{}", width: int = 0) -> str:
    """Format a value, replacing None with a right-aligned dash."""
    if val is None:
        return f"{'-':>{max(width, 1)}}"
    return f"{fmt.format(val):>{width}}" if width else fmt.format(val)


def _format_pretty(event: dict, msg_type: int, seq: int) -> str:
    evt_type = event.get("type", "unknown")
    prefix = f"{seq:>5} {evt_type:<7}"

    if evt_type == "state":
        state = event.get("state", "?")
        pose = event.get("pose", "?")
        dist = event.get("dist_cm")
        return (
            f"{prefix} {state:<15} {pose:<10}"
            f" h={event.get('human', '?')} n={event.get('n_targets', '?')}"
            f" d={_fv(dist, '{:.1f}', 5)}cm"
            f" hm={event.get('head_moving', '?')} new={event.get('dist_new', '?')}"
        )

    if evt_type == "targets":
        n = event.get("n", 0)
        focus = event.get("focus")
        trunc = event.get("targets_truncated", False)
        if focus:
            parts = (
                f"n={n}"
                f"  r={focus.get('r', 0):>5.2f}"
                f" θ={focus.get('bearing', 0):>6.1f}°"
                f" xy=({focus.get('x', 0):>5.3f},{focus.get('y', 0):>5.3f})"
                f" v={focus.get('v', 0):>4.1f}"
            )
        else:
            parts = f"n={n}  (no focus)"
        if trunc:
            parts += " [TRUNC]"
        return f"{prefix} {parts}"

    if evt_type == "bio":
        gate = "ok" if event.get("allowed") else "--"
        valid = "ok" if event.get("valid") else "--"
        flags = ""
        if event.get("br_new"):
            flags += " +br"
        if event.get("hr_new"):
            flags += " +hr"
        return (
            f"{prefix} {gate}/{valid:<2}"
            f" br={_fv(event.get('br'), '{:.1f}', 5)}"
            f" hr={_fv(event.get('hr'), '{:.1f}', 5)}"
            f"{flags}"
        )

    if evt_type == "light":
        mark = " " if event.get("valid") else "!"
        return f"{prefix}{mark}lux={_fv(event.get('lux'), '{:.1f}', 7)}"

    if evt_type == "diag":
        return (
            f"{prefix}"
            f" mmw_fail={event.get('mmwave_fail_count', '?')}"
            f" consec={event.get('consecutive_fails', '?')}"
            f" tx_drop={event.get('tx_drops', '?')}"
        )

    if evt_type == "pong":
        return f"{prefix} t={event.get('t_ms')}ms"

    if evt_type == "hello":
        return f"{prefix} v={event.get('proto_version')} feat=0x{event.get('feature_bits', 0):04X}"

    if evt_type == "ack":
        return (
            f"{prefix} cmd=0x{event.get('cmd_id', 0):02X} status={event.get('status_code')} val={event.get('value')}"
        )

    if evt_type == "err":
        return f"{prefix} cmd=0x{event.get('cmd_id', 0):02X} err={event.get('err_code')}"

    return f"{prefix} msg_type=0x{msg_type:02X} {event}"


def _emit_event(event: dict, msg_type: int, seq: int, fmt: str, allowed_types: set[str] | None) -> None:
    evt_type = event.get("type")
    if evt_type is None:
        return
    if allowed_types is not None and evt_type not in allowed_types:
        return
    if fmt == "json":
        print(json.dumps({"seq": seq, "msg_type": msg_type, "event": event}, ensure_ascii=True))
        return
    print(_format_pretty(event, msg_type, seq))


def _consume_bytes(
    buffer: bytearray,
    data: bytes,
    fmt: str,
    show_bad_frames: bool,
    allowed_types: set[str] | None,
) -> None:
    buffer.extend(data)
    for encoded in extract_encoded_frames(buffer):
        try:
            version, msg_type, seq, payload = decode_frame(encoded)
            event = decode_event(msg_type, payload)
        except ProtocolError as exc:
            if show_bad_frames:
                print(f"[bad-frame] {exc}", file=sys.stderr)
            continue

        if version != PROTO_VERSION:
            if show_bad_frames:
                print(f"[bad-frame] unsupported version {version}", file=sys.stderr)
            continue
        _emit_event(event, msg_type, seq, fmt, allowed_types)


def _iter_file_chunks(input_file: Path) -> Iterable[bytes]:
    raw = input_file.read_bytes()
    if _looks_like_hex_dump(raw):
        yield _parse_hex_dump(raw)
    else:
        yield raw


def _decode_file(input_file: Path, fmt: str, show_bad_frames: bool, allowed_types: set[str] | None) -> None:
    buffer = bytearray()
    for chunk in _iter_file_chunks(input_file):
        _consume_bytes(buffer, chunk, fmt, show_bad_frames, allowed_types)


def _decode_serial(port: str, baud: int, fmt: str, show_bad_frames: bool, allowed_types: set[str] | None) -> None:
    try:
        import serial
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime environment dependent
        raise SystemExit(f"missing dependency: {exc.name or 'pyserial'}")

    buffer = bytearray()
    with serial.Serial(port, baud, timeout=0.2) as ser:
        try:
            while True:
                waiting = getattr(ser, "in_waiting", 0)
                read_size = waiting if isinstance(waiting, int) and waiting > 0 else 1
                chunk = ser.read(read_size)
                if not chunk:
                    continue
                _consume_bytes(buffer, chunk, fmt, show_bad_frames, allowed_types)
        except KeyboardInterrupt:
            return


def main() -> int:
    """CLI entrypoint."""
    parser = argparse.ArgumentParser(description="Decode mmWave binary protocol frames.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Serial device path, e.g. /dev/cu.usbmodem1234")
    source.add_argument("--input-file", type=Path, help="Capture file path (raw bytes or hex dump text)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--format", choices=["pretty", "json"], default="pretty")
    parser.add_argument("--show-bad-frames", action="store_true", help="Print decode errors to stderr")
    parser.add_argument(
        "--filter",
        help="Comma-separated event types to show (e.g. bio,state,light). Default: all.",
    )
    args = parser.parse_args()

    allowed_types: set[str] | None = None
    if args.filter:
        allowed_types = {t.strip() for t in args.filter.split(",") if t.strip()}

    if args.input_file is not None:
        _decode_file(args.input_file, args.format, args.show_bad_frames, allowed_types)
        return 0

    _decode_serial(args.port, args.baud, args.format, args.show_bad_frames, allowed_types)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
