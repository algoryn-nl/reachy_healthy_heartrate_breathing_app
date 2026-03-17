# Development Log & TODOs

Completed work archived in `docs/DONE.md`. Detailed history in `docs/20260223_roadmap.md`.

## Tasks

### Current

(nothing actively in-progress)

### Done (recent)

- [x] TUI monitor for mmWave sensor (`hardware/tools/mmwave_monitor.py`) — Textual app with state, radar, vitals, log, diagnostics panels (2026-03-17)
- [x] Shared sensor data models (`sensor_models.py`) — dataclasses, EventBuffer, NotableFilter, BioAcceptanceTracker, read_events async iterator (2026-03-17)
- [x] `mmwave_decode.py` improvements — `--format tui` as default, `--filter` option, fixed-width pretty format, diag field name bug fix (2026-03-17)
- [x] 650 tests total (99 new: test_sensor_models.py, test_mmwave_monitor.py) (2026-03-17)

### Upcoming

See `docs/20260223_roadmap.md` > Aspirational (beyond v0.4) for future directions.

## Notes

### Development Decisions

- handleDecodedPacket() buffer validation already sufficient: exact length check + per-command payload validation — no additional guards needed (2026-02-23)
- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- scipy.signal.resample returns float64 from int16 input; fastrtc's audio_to_int16 only accepts int16/float32 — must cast to float32 after resample in receive() (2026-02-24)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Recurring Issues & Solutions

- **`or` vs `is not None`**: When accepting optional numeric parameters, always use `x if x is not None else default`. The `x or default` pattern silently replaces 0/0.0/False/"" with defaults.
- **Thread safety contracts**: Every module with multi-threaded access should document expected callers and synchronization requirements in its module docstring (see `moves.py` as the gold standard).
