# CMD_RESET Design: mmWave Radar Recovery

## Goal

Allow the host to tell the firmware to re-initialize the mmWave radar module when it's stuck, without rebooting the entire microcontroller.

## Problem

When the Seeed mmWave radar module stops responding (`mmWave.update()` consistently fails), the firmware has no recovery path. The only reset mechanism today is DTR (full MCU reboot), which wipes all state and is slow. The host can detect the failure via `EVT_DIAG` counters but cannot act on it.

## Design

### Firmware: `CMD_RESET (0x07)`

Empty payload. On receipt:

1. Call `mmWave.begin(Serial1)` to re-initialize the radar module
2. Success: reply `EVT_ACK(CMD_RESET, ACK_OK)`, reset `diag.mmwaveConsecFails` to 0
3. Failure: reply `EVT_ERR(CMD_RESET, ERR_RADAR_INIT_FAIL)` (new error code `6`)
4. All other state untouched: guard rails, focus cluster, intervals, presence tracking, vitals cache, light sensor

### Python Protocol

- `CMD_RESET = 0x07` constant and `pack_cmd_reset()` encoder in `mmwave_protocol.py`
- `ERR_RADAR_INIT_FAIL = 6` constant
- Decode support for the new error code

### Python mmWave Tool: Auto-Recovery

In `run_session()`, after version handshake:

1. Check `self._last_consec_fails >= RESET_CONSEC_FAIL_THRESHOLD` (default 20, env var `HEALTHY_MM_WAVE_RESET_THRESHOLD`)
2. If yes: send `CMD_RESET`, wait up to 2s for `EVT_ACK` or `EVT_ERR`
3. `ACK_OK`: log info, clear stored fail count, proceed normally
4. `ERR_RADAR_INIT_FAIL` or timeout: log warning, proceed anyway (normal error paths handle it)

The `_last_consec_fails` is populated from `EVT_DIAG` frames parsed during `_poll_events()` and persisted on the `MmWave` instance across calls.

### Data Flow

```
EVT_DIAG (during any session)
  → _poll_events() parses mmwave_consecutive_fails
  → stored on self._last_consec_fails

Next session start:
  → _handshake_version() succeeds
  → self._last_consec_fails >= threshold?
     Yes → send CMD_RESET, wait for ACK
     No  → proceed normally
  → scan/measure as usual
```

### Env Vars

| Variable | Default | Description |
|---|---|---|
| `HEALTHY_MM_WAVE_RESET_THRESHOLD` | `20` | Consecutive mmWave fails before auto CMD_RESET |

### What This Does NOT Do

- Does not reset host settings (guard rails, focus cluster, intervals)
- Does not reset presence tracking, vitals cache, or timers
- Does not re-init the BH1750 light sensor (separate recovery path exists)
- Does not touch IdlePolicy — auto-recovery lives in the mmWave tool session

## Approach Chosen Over Alternatives

- **Fire-and-forget + EVT_HELLO**: Rejected — muddies EVT_HELLO semantics (currently means "fresh boot")
- **DTR-based reset from Python**: Rejected — full MCU reboot, loses all state, slow, flaky
- **CMD_RESET with EVT_ACK**: Chosen — clean protocol extension, re-uses existing ACK/ERR patterns, radar-only scope
