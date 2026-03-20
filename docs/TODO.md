# Development Log & TODOs

Completed work archived in `docs/DONE.md`. Detailed history in `docs/20260223_roadmap.md`.

## Tasks

### Current

- [ ] Multi-person target tracking — verify with 2+ people in front of sensor (Seeed library frame limit fix applied, needs live validation)

### Done (recent)

- [x] Firmware: distance cache expiry — cached distance now expires after 2s, fixing NO_TARGET transition when everyone leaves (2026-03-20)
- [x] Firmware: Seeed library frame size limit — raised from 30 to FRAME_BUFFER_SIZE (512), enabling multi-target frames (2+ people) to flow through (2026-03-20)
- [x] TUI: target position smoothing — `TargetSmoother` EMA filter reduces x/y jitter on radar panel (2026-03-20)
- [x] TUI: radar x-axis mirror — display matches user perspective when screen faces same direction as sensor (2026-03-20)
- [x] TUI: multi-layer radar colors — range rings (#888888), vitals zone arcs (#c084fc), target glow (#444444) rendered on separate canvases (2026-03-20)
- [x] TUI: vitals label clarity — gate/quality acceptance labels (✓/✗), BR unit changed to rpm, state bar shows current state name (2026-03-20)
- [x] TUI: Help tab — full legend for device states, vitals, radar, state bar, keybindings (2026-03-20)
- [x] 659 tests total (9 new: TargetSmoother, acceptance label, tab cycling) (2026-03-20)
- [x] TUI monitor for mmWave sensor (`hardware/tools/mmwave_monitor.py`) — Textual app with state, radar, vitals, log, diagnostics panels (2026-03-17)
- [x] Shared sensor data models (`sensor_models.py`) — dataclasses, EventBuffer, NotableFilter, BioAcceptanceTracker, read_events async iterator (2026-03-17)
- [x] `mmwave_decode.py` improvements — `--format tui` as default, `--filter` option, fixed-width pretty format, diag field name bug fix (2026-03-17)

### Upcoming

- [ ] Sensor fusion: combine mmWave radar with camera vision for robust person tracking (bearing from camera, range/vitals from radar)
- See `docs/20260223_roadmap.md` > Aspirational (beyond v0.4) for future directions.

## Notes

### Development Decisions

- handleDecodedPacket() buffer validation already sufficient: exact length check + per-command payload validation — no additional guards needed (2026-02-23)
- Locked profile pattern: app always uses `_healthy_heartrate_breathing_locked_profile` (2026-02-23)
- scipy.signal.resample returns float64 from int16 input; fastrtc's audio_to_int16 only accepts int16/float32 — must cast to float32 after resample in receive() (2026-02-24)
- COBS + CRC-16 binary protocol chosen for mmWave: compact, checksummed, no framing ambiguity

### Recurring Issues & Solutions

- **`or` vs `is not None`**: When accepting optional numeric parameters, always use `x if x is not None else default`. The `x or default` pattern silently replaces 0/0.0/False/"" with defaults.
- **Thread safety contracts**: Every module with multi-threaded access should document expected callers and synchronization requirements in its module docstring (see `moves.py` as the gold standard).
