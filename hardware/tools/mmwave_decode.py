"""Decode mmWave binary protocol frames from serial or capture files."""

from __future__ import annotations
import re
import sys
import json
import argparse
from typing import Iterable
from pathlib import Path

from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
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


def _format_pretty(event: dict, msg_type: int, seq: int) -> str:
    evt_type = event.get("type", "unknown")
    if evt_type == "targets":
        return (
            f"seq={seq} type=targets n={event.get('n')} "
            f"focus={event.get('focus')} truncated={event.get('targets_truncated')}"
        )
    if evt_type == "bio":
        return (
            f"seq={seq} type=bio allowed={event.get('allowed')} valid={event.get('valid')} "
            f"br={event.get('br')} hr={event.get('hr')}"
        )
    if evt_type == "state":
        return (
            f"seq={seq} type=state state={event.get('state')} pose={event.get('pose')} "
            f"human={event.get('human')} n_targets={event.get('n_targets')} dist_cm={event.get('dist_cm')}"
        )
    if evt_type == "ack":
        return f"seq={seq} type=ack cmd={event.get('cmd_id')} status={event.get('status_code')} value={event.get('value')}"
    if evt_type == "err":
        return f"seq={seq} type=err cmd={event.get('cmd_id')} err={event.get('err_code')}"
    return f"seq={seq} msg_type=0x{msg_type:02X} event={event}"


def _emit_event(event: dict, msg_type: int, seq: int, fmt: str) -> None:
    if fmt == "json":
        print(json.dumps({"seq": seq, "msg_type": msg_type, "event": event}, ensure_ascii=True))
        return
    print(_format_pretty(event, msg_type, seq))


def _consume_bytes(buffer: bytearray, data: bytes, fmt: str, show_bad_frames: bool) -> None:
    buffer.extend(data)
    for encoded in extract_encoded_frames(buffer):
        try:
            version, msg_type, seq, payload = decode_frame(encoded)
            event = decode_event(msg_type, payload)
        except ProtocolError as exc:
            if show_bad_frames:
                print(f"[bad-frame] {exc}", file=sys.stderr)
            continue

        if version != 1:
            if show_bad_frames:
                print(f"[bad-frame] unsupported version {version}", file=sys.stderr)
            continue
        _emit_event(event, msg_type, seq, fmt)


def _iter_file_chunks(input_file: Path) -> Iterable[bytes]:
    raw = input_file.read_bytes()
    if _looks_like_hex_dump(raw):
        yield _parse_hex_dump(raw)
    else:
        yield raw


def _decode_file(input_file: Path, fmt: str, show_bad_frames: bool) -> None:
    buffer = bytearray()
    for chunk in _iter_file_chunks(input_file):
        _consume_bytes(buffer, chunk, fmt, show_bad_frames)


def _decode_serial(port: str, baud: int, fmt: str, show_bad_frames: bool) -> None:
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
                _consume_bytes(buffer, chunk, fmt, show_bad_frames)
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
    args = parser.parse_args()

    if args.input_file is not None:
        _decode_file(args.input_file, args.format, args.show_bad_frames)
        return 0

    _decode_serial(args.port, args.baud, args.format, args.show_bad_frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
