# CMD_RESET Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a CMD_RESET command that re-initializes the mmWave radar module when it's stuck, with auto-recovery triggered by high consecutive failure counts from EVT_DIAG.

**Architecture:** New command `CMD_RESET (0x07)` with empty payload. Firmware calls `mmWave.begin()` and replies ACK/ERR. Python encoder + auto-trigger in `run_session()` based on `_last_consec_fails` from EVT_DIAG frames. New error code `ERR_RADAR_INIT_FAIL = 6`.

**Tech Stack:** Arduino C++ (firmware), Python 3.12 (protocol + tool), pytest + pytest-asyncio (tests)

---

### Task 1: Firmware — Add CMD_RESET handler

**Files:**
- Modify: `hardware/arduino/reachy-sensor/reachy-sensor.ino:143` (add constant), `:166` (add error code), `:810-853` (add handler before the unknown-cmd fallback)
- Modify: `hardware/README.md:49` (add protocol docs)

**Context:** The firmware dispatches commands in `applyBinaryCommand()` (line 738). Each command is an `if (msgType == CMD_...)` block that validates payload length, processes the command, and replies with `emitAck()` or `emitErr()`. The unknown-command fallback `emitErr(msgType, ERR_UNKNOWN_CMD)` is at line 855.

**Step 1: Add CMD_RESET constant and ERR_RADAR_INIT_FAIL error code**

In `reachy-sensor.ino`, after line 143 (`CMD_SET_GUARD_RAILS`), add:

```cpp
static const uint8_t CMD_RESET           = 0x07;  // re-initialize mmWave radar (no payload)
```

After line 166 (`ERR_UNSUPPORTED_VERSION`), add:

```cpp
static const uint8_t ERR_RADAR_INIT_FAIL = 6;  // radar re-initialization failed
```

**Step 2: Add CMD_RESET handler in applyBinaryCommand()**

In `reachy-sensor.ino`, insert after the CMD_SET_GUARD_RAILS block (after line 853, before `emitErr(msgType, ERR_UNKNOWN_CMD)`):

```cpp
  // CMD_RESET: re-initialize the mmWave radar module.
  // Used by the host to recover from a stuck radar (mmWave.update() failing
  // persistently). Only re-inits the radar; all other state (guard rails,
  // focus cluster, intervals, presence tracking, vitals cache) is preserved.
  if (msgType == CMD_RESET) {
    if (payloadLen != 0) {
      emitErr(msgType, ERR_BAD_LEN);
      return;
    }
    mmWave.begin(&mmWaveSerial);
    // Re-read to verify initialization succeeded
    if (mmWave.update(200)) {
      diag.mmwaveConsecFails = 0;
      emitAck(msgType, ACK_OK, 0);
    } else {
      emitErr(msgType, ERR_RADAR_INIT_FAIL);
    }
    return;
  }
```

**Step 3: Add CMD_RESET to hardware/README.md**

After the CMD_SET_GUARD_RAILS line, add:

```
- `0x07 CMD_RESET` payload: *(empty)* — re-initialize mmWave radar module; replies `ACK_OK` on success or `ERR_RADAR_INIT_FAIL` on failure
```

**Step 4: Commit**

```bash
git add hardware/arduino/reachy-sensor/reachy-sensor.ino hardware/README.md
git commit -m "feat(fw): add CMD_RESET for mmWave radar re-initialization"
```

---

### Task 2: Python protocol — Add CMD_RESET encoder and ERR code

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmwave_protocol.py:17` (add constant), `:41` (add error code), after `:213` (add encoder)
- Test: `tests/test_mmwave.py`

**Context:** The protocol file defines command constants (lines 12-17), error codes (lines 36-40), and pack functions (lines 183-213). Each `pack_cmd_*()` function returns a `bytes` payload. The `decode_event()` function (line 216) already decodes `EVT_ACK` and `EVT_ERR` — no changes needed there since the existing ACK/ERR decode handles any `cmd_id` value.

**Step 1: Write failing tests**

Add to `tests/test_mmwave.py`, in the protocol test section (near the guard rails tests around line 2469):

```python
class TestCmdResetProtocol:
    """CMD_RESET encode/decode round-trip tests."""

    def test_pack_cmd_reset_empty_payload(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            pack_cmd_reset,
        )
        payload = pack_cmd_reset()
        assert payload == b""

    def test_cmd_reset_constant_value(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
        )
        assert CMD_RESET == 0x07

    def test_err_radar_init_fail_constant(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            ERR_RADAR_INIT_FAIL,
        )
        assert ERR_RADAR_INIT_FAIL == 6

    def test_cmd_reset_frame_roundtrip(self) -> None:
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            decode_frame,
            encode_frame,
            pack_cmd_reset,
        )
        frame = encode_frame(CMD_RESET, pack_cmd_reset(), seq=42)
        version, msg_type, seq, payload = decode_frame(frame[:-1])  # strip 0x00 delimiter
        assert version == 1
        assert msg_type == CMD_RESET
        assert seq == 42
        assert payload == b""

    def test_ack_for_cmd_reset_decodes(self) -> None:
        """EVT_ACK with cmd_id=CMD_RESET decodes correctly."""
        import struct
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            ACK_OK,
            CMD_RESET,
            EVT_ACK,
            decode_event,
        )
        payload = struct.pack("<BBi", CMD_RESET, ACK_OK, 0)
        ev = decode_event(EVT_ACK, payload)
        assert ev["type"] == "ack"
        assert ev["cmd_id"] == CMD_RESET
        assert ev["status_code"] == ACK_OK

    def test_err_for_cmd_reset_decodes(self) -> None:
        """EVT_ERR with cmd_id=CMD_RESET and ERR_RADAR_INIT_FAIL decodes."""
        import struct
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            ERR_RADAR_INIT_FAIL,
            EVT_ERR,
            decode_event,
        )
        payload = struct.pack("<BB", CMD_RESET, ERR_RADAR_INIT_FAIL)
        ev = decode_event(EVT_ERR, payload)
        assert ev["type"] == "err"
        assert ev["cmd_id"] == CMD_RESET
        assert ev["err_code"] == ERR_RADAR_INIT_FAIL
```

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_mmwave.py::TestCmdResetProtocol -v`
Expected: FAIL with `ImportError` (CMD_RESET not defined)

**Step 3: Add constants and encoder to mmwave_protocol.py**

After line 17 (`CMD_SET_GUARD_RAILS = 0x06`), add:

```python
CMD_RESET = 0x07
```

After line 40 (`ERR_UNSUPPORTED_VERSION = 5`), add:

```python
ERR_RADAR_INIT_FAIL = 6
```

After `pack_cmd_set_guard_rails()` (after line 213), add:

```python
def pack_cmd_reset() -> bytes:
    """Build payload for CMD_RESET (empty)."""
    return b""
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_mmwave.py::TestCmdResetProtocol -v`
Expected: 6 PASS

**Step 5: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmwave_protocol.py tests/test_mmwave.py
git commit -m "feat: add CMD_RESET protocol encoder and ERR_RADAR_INIT_FAIL constant"
```

---

### Task 3: Firmware codec cross-validation — Add CMD_RESET tests

**Files:**
- Modify: `tests/test_firmware_codec.py` (add CMD_RESET cross-validation test)
- Modify: `hardware/tests/reachy_codec_shim.c` (only if CMD_RESET needs codec-level functions — it doesn't, since CMD_RESET uses existing emit/decode infrastructure)

**Context:** `tests/test_firmware_codec.py` uses a ctypes shim to test `reachy_codec.h` functions against the Python protocol. CMD_RESET has an empty payload so there's no new codec function, but we should add a frame round-trip test that encodes a CMD_RESET frame using Python and verifies COBS/CRC via the C codec.

**Step 1: Write failing test**

Add to `tests/test_firmware_codec.py`:

```python
class TestCmdResetCodec:
    """Cross-validate CMD_RESET frame encoding between Python and C codec."""

    def test_cmd_reset_frame_cobs_crc_matches_c(self, codec: Any) -> None:
        """Encode CMD_RESET in Python, decode via C COBS, verify CRC matches."""
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            encode_frame,
            pack_cmd_reset,
        )
        frame = encode_frame(CMD_RESET, pack_cmd_reset(), seq=0)
        encoded = frame[:-1]  # strip 0x00 delimiter

        # COBS decode via C
        in_buf = (ctypes.c_uint8 * len(encoded))(*encoded)
        out_buf = (ctypes.c_uint8 * 256)()
        out_len = ctypes.c_size_t(0)
        ok = codec.rc_cobs_decode(in_buf, len(encoded), out_buf, ctypes.byref(out_len), 256)
        assert ok == 1

        # Verify CRC via C
        decoded_bytes = bytes(out_buf[: out_len.value])
        # CRC covers everything except the last 2 bytes (the CRC itself)
        data_len = out_len.value - 2
        data_buf = (ctypes.c_uint8 * data_len)(*decoded_bytes[:data_len])
        crc_c = codec.rc_crc16_ccitt_false(data_buf, data_len)

        # Extract CRC from packet (last 2 bytes, little-endian)
        import struct
        crc_in_packet = struct.unpack_from("<H", decoded_bytes, data_len)[0]
        assert crc_c == crc_in_packet
```

**Step 2: Run test to verify it fails (or passes — it may pass immediately since encode_frame already works)**

Run: `uv run pytest tests/test_firmware_codec.py::TestCmdResetCodec -v`
Expected: PASS (the Python encoder already works; this is a cross-validation)

**Step 3: Commit**

```bash
git add tests/test_firmware_codec.py
git commit -m "test: add CMD_RESET cross-validation between Python and C codec"
```

---

### Task 4: mmWave tool — Store diag consecutive fails and auto-reset

**Files:**
- Modify: `src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py:16-32` (add imports), `:57-58` (add instance state), `:538-543` (store diag), `:690-698` (add reset logic in run_session)
- Test: `tests/test_mmwave.py`

**Context:** The `MmWave` tool is a long-lived singleton in the tool registry. `__init__` (line 57) runs once. `run_session()` (line 690) is a closure called inside `asyncio.to_thread()` for each tool invocation. During `_scan_sync()` and `_measure_sync()`, EVT_DIAG frames are parsed by `_poll_events()` and stored in `latest_diag` (lines 538-543). We need to:
1. Store `_last_consec_fails` on `self` so it persists across tool calls
2. Update it from diag data seen during scan/measure
3. In `run_session()`, after handshake, check threshold and send CMD_RESET if needed

**Step 1: Write failing tests**

Add to `tests/test_mmwave.py`:

```python
class TestAutoReset:
    """CMD_RESET auto-recovery based on EVT_DIAG consecutive fail count."""

    @pytest.fixture
    def mmwave_tool(self) -> MmWave:
        tool = MmWave()
        return tool

    def test_initial_consec_fails_is_zero(self, mmwave_tool: MmWave) -> None:
        assert mmwave_tool._last_consec_fails == 0

    def test_consec_fails_stored_from_diag(self, mmwave_tool: MmWave) -> None:
        """Diag data from scan should update _last_consec_fails."""
        mmwave_tool._last_consec_fails = 0
        mmwave_tool._update_diag_state({"mmwave_consecutive_fails": 15})
        assert mmwave_tool._last_consec_fails == 15

    def test_consec_fails_cleared_on_reset_success(self, mmwave_tool: MmWave) -> None:
        mmwave_tool._last_consec_fails = 25
        mmwave_tool._on_reset_success()
        assert mmwave_tool._last_consec_fails == 0

    def test_should_attempt_reset_below_threshold(self, mmwave_tool: MmWave) -> None:
        mmwave_tool._last_consec_fails = 5
        assert not mmwave_tool._should_attempt_reset()

    def test_should_attempt_reset_at_threshold(self, mmwave_tool: MmWave) -> None:
        mmwave_tool._last_consec_fails = 20
        assert mmwave_tool._should_attempt_reset()

    def test_should_attempt_reset_above_threshold(self, mmwave_tool: MmWave) -> None:
        mmwave_tool._last_consec_fails = 50
        assert mmwave_tool._should_attempt_reset()

    def test_reset_threshold_from_env(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("HEALTHY_MM_WAVE_RESET_THRESHOLD", "10")
        tool = MmWave()
        tool._last_consec_fails = 10
        assert tool._should_attempt_reset()
        tool._last_consec_fails = 9
        assert not tool._should_attempt_reset()

    @pytest.mark.asyncio
    async def test_reset_sent_when_threshold_exceeded(
        self, mmwave_tool: MmWave, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """When _last_consec_fails >= threshold, run_session sends CMD_RESET."""
        import struct
        from unittest.mock import MagicMock, patch
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            ACK_OK,
            CMD_RESET,
            EVT_ACK,
            encode_frame,
        )

        mmwave_tool._last_consec_fails = 25

        # Build an ACK response for CMD_RESET
        ack_payload = struct.pack("<BBi", CMD_RESET, ACK_OK, 0)
        ack_frame = encode_frame(EVT_ACK, ack_payload, seq=0)

        mock_ser = MagicMock()
        mock_ser.read = MagicMock(return_value=ack_frame)
        mock_ser.in_waiting = len(ack_frame)
        mock_ser.write = MagicMock()
        mock_ser.flush = MagicMock()

        tx_state = {"seq": 0}
        rx_buffer = bytearray()

        result = mmwave_tool._attempt_radar_reset(mock_ser, tx_state, rx_buffer)
        assert result is True
        assert mmwave_tool._last_consec_fails == 0

    @pytest.mark.asyncio
    async def test_reset_not_sent_below_threshold(self, mmwave_tool: MmWave) -> None:
        """When _last_consec_fails < threshold, no reset attempt."""
        mmwave_tool._last_consec_fails = 5
        assert not mmwave_tool._should_attempt_reset()

    @pytest.mark.asyncio
    async def test_reset_failure_does_not_clear_counter(
        self, mmwave_tool: MmWave
    ) -> None:
        """When CMD_RESET gets ERR, _last_consec_fails stays."""
        import struct
        from unittest.mock import MagicMock
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            CMD_RESET,
            ERR_RADAR_INIT_FAIL,
            EVT_ERR,
            encode_frame,
        )

        mmwave_tool._last_consec_fails = 25

        err_payload = struct.pack("<BB", CMD_RESET, ERR_RADAR_INIT_FAIL)
        err_frame = encode_frame(EVT_ERR, err_payload, seq=0)

        mock_ser = MagicMock()
        mock_ser.read = MagicMock(return_value=err_frame)
        mock_ser.in_waiting = len(err_frame)
        mock_ser.write = MagicMock()
        mock_ser.flush = MagicMock()

        tx_state = {"seq": 0}
        rx_buffer = bytearray()

        result = mmwave_tool._attempt_radar_reset(mock_ser, tx_state, rx_buffer)
        assert result is False
        assert mmwave_tool._last_consec_fails == 25

    @pytest.mark.asyncio
    async def test_reset_timeout_does_not_clear_counter(
        self, mmwave_tool: MmWave
    ) -> None:
        """When CMD_RESET times out, _last_consec_fails stays."""
        from unittest.mock import MagicMock

        mmwave_tool._last_consec_fails = 25

        mock_ser = MagicMock()
        mock_ser.read = MagicMock(return_value=b"")  # no response
        mock_ser.in_waiting = 0
        mock_ser.write = MagicMock()
        mock_ser.flush = MagicMock()

        tx_state = {"seq": 0}
        rx_buffer = bytearray()

        result = mmwave_tool._attempt_radar_reset(mock_ser, tx_state, rx_buffer)
        assert result is False
        assert mmwave_tool._last_consec_fails == 25
```

**Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_mmwave.py::TestAutoReset -v`
Expected: FAIL — `_last_consec_fails` attribute missing, methods missing

**Step 3: Implement the auto-reset logic in mmWave.py**

In `mmWave.py`, add imports (after line 17, in the import block from mmwave_protocol):

```python
    CMD_RESET,
    ACK_OK,
    ERR_RADAR_INIT_FAIL,
    pack_cmd_reset,
```

In `__init__` (line 57-58), add instance state:

```python
    def __init__(self) -> None:  # noqa: D107
        self._warned_version_mismatch = False
        self._last_consec_fails = 0
        self._reset_threshold = coerce_int(
            os.getenv("HEALTHY_MM_WAVE_RESET_THRESHOLD", "20"), 20, min_value=1
        )
```

Add helper methods after `__init__`:

```python
    def _update_diag_state(self, diag: dict[str, Any]) -> None:
        """Store latest consecutive fail count from EVT_DIAG."""
        consec = diag.get("mmwave_consecutive_fails")
        if isinstance(consec, int):
            self._last_consec_fails = consec

    def _on_reset_success(self) -> None:
        """Clear stored fail count after successful CMD_RESET."""
        self._last_consec_fails = 0

    def _should_attempt_reset(self) -> bool:
        """Check if auto-reset should be attempted."""
        return self._last_consec_fails >= self._reset_threshold

    def _attempt_radar_reset(
        self, ser: Any, tx_state: Dict[str, int], rx_buffer: bytearray
    ) -> bool:
        """Send CMD_RESET and wait for ACK. Returns True on success."""
        logger.info(
            "Attempting radar reset (consecutive_fails=%d, threshold=%d)",
            self._last_consec_fails,
            self._reset_threshold,
        )
        try:
            self._send_command(ser, tx_state, CMD_RESET, pack_cmd_reset())
        except OSError as exc:
            logger.warning("CMD_RESET write failed: %s", exc)
            return False

        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                events = self._poll_events(ser, rx_buffer)
            except OSError:
                return False
            for event in events:
                if event.get("type") == "ack" and event.get("cmd_id") == CMD_RESET:
                    if event.get("status_code") == ACK_OK:
                        logger.info("Radar reset succeeded")
                        self._on_reset_success()
                        return True
                if event.get("type") == "err" and event.get("cmd_id") == CMD_RESET:
                    logger.warning("Radar reset failed: err_code=%s", event.get("err_code"))
                    return False

        logger.warning("Radar reset timed out")
        return False
```

In `_scan_sync()`, update the diag storage block (around line 538-543) to also call `_update_diag_state`:

```python
        if latest_diag:
            state["diagnostics"] = {
                "mmwave_fail_count": latest_diag["mmwave_fail_count"],
                "mmwave_consecutive_fails": latest_diag["mmwave_consecutive_fails"],
                "tx_drop_count": latest_diag["tx_drop_count"],
            }
            self._update_diag_state(latest_diag)
```

Same change in `_measure_sync()` — there are two places where `latest_diag` is written to `result["diagnostics"]` (lines 618-623 and 635-640). Add `self._update_diag_state(latest_diag)` after each.

In `run_session()` (line 690), after the handshake check (line 696-698), add:

```python
                if self._should_attempt_reset():
                    self._attempt_radar_reset(ser, tx_state, rx_buffer)
```

**Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_mmwave.py::TestAutoReset -v`
Expected: 12 PASS

**Step 5: Run full test suite**

Run: `uv run pytest tests/ --ignore=tests/vision -q`
Expected: 490+ passed (plus 2 pre-existing failures)

**Step 6: Commit**

```bash
git add src/healthy_heartrate_breathing/profiles/_healthy_heartrate_breathing_locked_profile/mmWave.py tests/test_mmwave.py
git commit -m "feat: auto-send CMD_RESET when EVT_DIAG shows high consecutive fails"
```

---

### Task 5: Documentation — Update all four doc surfaces

**Files:**
- Modify: `CLAUDE.md` (env vars table, protocol section, tool description)
- Modify: `README.md` (mmWave tool modes table, configuration table)
- Modify: `docs/TODO.md` (mark Phase 4 item 3 done)
- Modify: `docs/20260223_roadmap.md` (strike through Phase 4 item 3)

**Step 1: Update CLAUDE.md**

In the Environment Variables section, under "### mmWave Idle Scanning Policy", add:

```markdown
| `HEALTHY_MM_WAVE_RESET_THRESHOLD` | `20` | Consecutive mmWave fails (from EVT_DIAG) before auto-sending CMD_RESET |
```

In the "### Hardware: mmWave Sensor Module" section, under "Host commands", add CMD_RESET to the list:
```
`CMD_RESET` (radar re-init)
```

Add `ERR_RADAR_INIT_FAIL` to the error codes list.

**Step 2: Update README.md**

In the Configuration table, add the new env var.

**Step 3: Update docs/TODO.md**

Add a new entry under "Done (recent)":
```markdown
- [x] **FW-PHASE4-3**: CMD_RESET soft-reset command — `CMD_RESET (0x07)` re-initializes mmWave radar without MCU reboot; auto-triggered when EVT_DIAG consecutive fails exceed threshold (default 20); new `ERR_RADAR_INIT_FAIL` error code; Python auto-recovery in `run_session()` (2026-03-05)
```

**Step 4: Update docs/20260223_roadmap.md**

Strike through Phase 4 item 3:
```markdown
3. ~~Implement soft-reset command (`CMD_RESET`) for recovery from sensor/I2C failures without power cycle~~ (done: `CMD_RESET (0x07)` re-inits mmWave radar, auto-triggered by EVT_DIAG consecutive fails >= threshold; `ERR_RADAR_INIT_FAIL` error code; 2026-03-05)
```

**Step 5: Commit**

```bash
git add CLAUDE.md README.md docs/TODO.md docs/20260223_roadmap.md
git commit -m "docs: add CMD_RESET to all documentation surfaces"
```

---

### Task 6: Run full verification

**Step 1: Run lint**

Run: `uv run ruff check .`
Expected: clean

**Step 2: Run full tests**

Run: `uv run pytest tests/ --ignore=tests/vision -q`
Expected: 500+ passed (plus 2 pre-existing failures)

**Step 3: Run type check**

Run: `uv run mypy src/ tests/`
Expected: clean (or pre-existing issues only)
