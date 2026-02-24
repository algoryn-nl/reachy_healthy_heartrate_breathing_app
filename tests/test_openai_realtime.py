"""Tests for OpenaiRealtimeHandler: emit, receive, idle, tool dispatch, event loop."""
# ruff: noqa: D101, D102, D103

import base64
import asyncio
import logging
from typing import Any
from decimal import Decimal
from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np
import pytest

import healthy_heartrate_breathing.openai_realtime as rt_mod
from healthy_heartrate_breathing.openai_realtime import OpenaiRealtimeHandler, _compute_response_cost
from healthy_heartrate_breathing.tools.core_tools import ToolDependencies


# ---------------------------------------------------------------------------
# Reusable fakes
# ---------------------------------------------------------------------------


class _FakeEvent:
    """Lightweight event stub with arbitrary attributes."""

    def __init__(self, **kwargs: Any) -> None:
        for k, v in kwargs.items():
            setattr(self, k, v)


class _EventConn:
    """Reusable fake connection that yields a configurable event sequence."""

    def __init__(self, events: list[Any] | None = None) -> None:
        self._events = list(events or [])
        self._idx = 0
        self.session_update_calls: list[dict[str, Any]] = []
        self.item_create_calls: list[dict[str, Any]] = []
        self.response_create_calls: list[dict[str, Any]] = []
        self.appended_audio: list[str] = []
        self._closed = False

        conn_ref = self

        class _Session:
            async def update(self, **kw: Any) -> None:
                conn_ref.session_update_calls.append(kw)

        self.session = _Session()

        class _InputAudioBuffer:
            async def append(self, **kw: Any) -> None:
                audio = kw.get("audio", "")
                conn_ref.appended_audio.append(audio)

        self.input_audio_buffer = _InputAudioBuffer()

        class _Item:
            async def create(self, **kw: Any) -> None:
                conn_ref.item_create_calls.append(kw)

        class _Conversation:
            item = _Item()

        self.conversation = _Conversation()

        class _Response:
            async def create(self, **kw: Any) -> None:
                conn_ref.response_create_calls.append(kw)

            async def cancel(self, **kw: Any) -> None:
                return None

        self.response = _Response()

    async def __aenter__(self) -> "_EventConn":
        return self

    async def __aexit__(self, *args: Any) -> bool:
        return False

    async def close(self) -> None:
        self._closed = True

    def __aiter__(self) -> "_EventConn":
        return self

    async def __anext__(self) -> Any:
        if self._idx >= len(self._events):
            raise StopAsyncIteration
        event = self._events[self._idx]
        self._idx += 1
        return event


def _make_event_client(events: list[Any] | None = None) -> tuple[Any, _EventConn]:
    """Create a (client, conn) pair where conn yields the given events."""
    conn = _EventConn(events)

    class _Realtime:
        def connect(self, **_kw: Any) -> _EventConn:
            return conn

    class _Client:
        realtime = _Realtime()

    return _Client(), conn


def _deps(**overrides: Any) -> ToolDependencies:
    """Build minimal ToolDependencies with optional overrides."""
    mm = MagicMock()
    mm.is_idle.return_value = True
    defaults: dict[str, Any] = {
        "reachy_mini": MagicMock(),
        "movement_manager": mm,
    }
    defaults.update(overrides)
    return ToolDependencies(**defaults)


def _handler(**overrides: Any) -> OpenaiRealtimeHandler:
    """Build a handler with sane defaults for testing."""
    deps = overrides.pop("deps", _deps())
    return OpenaiRealtimeHandler(deps, **overrides)


async def _run_session_with_events(
    handler: OpenaiRealtimeHandler,
    events: list[Any],
    monkeypatch: Any,
    *,
    tool_specs: list[dict[str, Any]] | None = None,
    dispatch_return: dict[str, Any] | None = None,
) -> _EventConn:
    """Run _run_realtime_session with the given events and standard mocks."""
    client, conn = _make_event_client(events)
    monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "test instructions")
    monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")
    monkeypatch.setattr(rt_mod, "get_tool_specs", lambda: tool_specs or [])
    monkeypatch.setattr(
        rt_mod,
        "dispatch_tool_call",
        AsyncMock(return_value=dispatch_return or {"status": "ok"}),
    )
    handler.client = client
    await handler._run_realtime_session()
    # Give background tasks (tool dispatch) a moment to settle
    await asyncio.sleep(0.05)
    return conn


# ---------------------------------------------------------------------------
# Legacy helpers (kept for existing tests)
# ---------------------------------------------------------------------------


def _build_handler(loop: asyncio.AbstractEventLoop) -> OpenaiRealtimeHandler:
    asyncio.set_event_loop(loop)
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    return OpenaiRealtimeHandler(deps)


def _make_usage(
    audio_in: int | None = 0,
    text_in: int | None = 0,
    image_in: int | None = 0,
    audio_out: int | None = 0,
    text_out: int | None = 0,
    has_input: bool = True,
    has_output: bool = True,
) -> MagicMock:
    """Build a fake usage object matching the OpenAI response.usage shape."""
    usage = MagicMock()
    if has_input:
        inp = MagicMock()
        inp.audio_tokens = audio_in
        inp.text_tokens = text_in
        inp.image_tokens = image_in
        usage.input_token_details = inp
    else:
        usage.input_token_details = None
    if has_output:
        out = MagicMock()
        out.audio_tokens = audio_out
        out.text_tokens = text_out
        usage.output_token_details = out
    else:
        usage.output_token_details = None
    return usage


# ====================================================================
# Existing tests (preserved)
# ====================================================================


def test_format_timestamp_uses_wall_clock() -> None:
    """Test that format_timestamp uses wall clock time."""
    loop = asyncio.new_event_loop()
    try:
        handler = _build_handler(loop)
        formatted = handler.format_timestamp()
    finally:
        asyncio.set_event_loop(None)
        loop.close()

    year = int(formatted[1:5])
    assert year == datetime.now(timezone.utc).year


@pytest.mark.asyncio
async def test_start_up_retries_on_abrupt_close(monkeypatch: Any, caplog: Any) -> None:
    """First connection dies with ConnectionClosedError during iteration -> retried.

    Second connection iterates cleanly (no events) -> start_up returns without raising.
    Ensures handler clears self.connection at the end.
    """
    caplog.set_level(logging.WARNING)

    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    async def _fast_sleep(*_a: Any, **_kw: Any) -> None:
        return None

    monkeypatch.setattr(asyncio, "sleep", _fast_sleep, raising=False)

    attempt_counter = {"n": 0}

    class FakeConn:
        def __init__(self, mode: str):
            self._mode = mode

            class _Session:
                async def update(self, **_kw: Any) -> None:
                    return None

            self.session = _Session()

            class _InputAudioBuffer:
                async def append(self, **_kw: Any) -> None:
                    return None

            self.input_audio_buffer = _InputAudioBuffer()

            class _Item:
                async def create(self, **_kw: Any) -> None:
                    return None

            class _Conversation:
                item = _Item()

            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None:
                    return None

                async def cancel(self, **_kw: Any) -> None:
                    return None

            self.response = _Response()

        async def __aenter__(self) -> "FakeConn":
            return self

        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
            return False

        async def close(self) -> None:
            return None

        def __aiter__(self) -> "FakeConn":
            return self

        async def __anext__(self) -> None:
            if self._mode == "raise_on_iter":
                raise FakeCCE("abrupt close (simulated)")
            raise StopAsyncIteration

    class FakeRealtime:
        def connect(self, **_kw: Any) -> FakeConn:
            attempt_counter["n"] += 1
            mode = "raise_on_iter" if attempt_counter["n"] == 1 else "clean"
            return FakeConn(mode)

    class FakeClient:
        def __init__(self, **_kw: Any) -> None:
            self.realtime = FakeRealtime()

    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)

    await handler.start_up()

    assert attempt_counter["n"] == 2
    assert handler.connection is None

    warnings = [r for r in caplog.records if r.levelname == "WARNING" and "closed unexpectedly" in r.msg]
    assert len(warnings) == 1


# ---- Cost calculation tests ----


@pytest.mark.parametrize(
    "usage_kwargs, expect_positive",
    [
        ({"audio_in": 1000, "text_in": 2000, "image_in": 500, "audio_out": 800, "text_out": 300}, True),
        ({"audio_in": None, "text_in": None, "image_in": None, "audio_out": None, "text_out": None}, False),
        ({"audio_in": None, "text_in": 500, "image_in": None, "audio_out": 1000, "text_out": None}, True),
        ({"has_input": False, "has_output": False}, False),
    ],
    ids=["normal", "all_none", "mixed", "missing_details"],
)
def test_compute_response_cost(usage_kwargs: dict[str, Any], expect_positive: bool) -> None:
    """Verify _compute_response_cost handles various token combinations without crashing."""
    usage = _make_usage(**usage_kwargs)
    cost = _compute_response_cost(usage)
    assert isinstance(cost, Decimal), "cost must be Decimal for precision"
    if expect_positive:
        assert cost > 0
    else:
        assert cost == Decimal(0)


# ---- Idle flag reset tests ----


@pytest.mark.asyncio
async def test_idle_flag_reset_on_send_idle_signal_exception(monkeypatch: Any) -> None:
    """is_idle_tool_call must be reset when send_idle_signal raises."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)

    handler.last_activity_time = 0.0
    handler.start_time = 0.0
    deps.movement_manager.is_idle.return_value = True

    async def _raise(*_a: Any, **_kw: Any) -> None:
        handler.is_idle_tool_call = True
        raise ConnectionError("simulated disconnect")

    monkeypatch.setattr(handler, "send_idle_signal", _raise)
    monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
    monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

    result = await handler.emit()

    assert result is None
    assert handler.is_idle_tool_call is False, "Flag must be reset after exception"


@pytest.mark.asyncio
async def test_emit_returns_none_when_idle_signal_fails(monkeypatch: Any) -> None:
    """emit() returns None (not crash) when send_idle_signal fails."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)

    handler.last_activity_time = 0.0
    deps.movement_manager.is_idle.return_value = True
    monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
    monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

    async def _raise(*_a: Any, **_kw: Any) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(handler, "send_idle_signal", _raise)

    result = await handler.emit()
    assert result is None


# ---- Connection cleanup tests ----


@pytest.mark.asyncio
async def test_connection_cleared_on_event_loop_exception(monkeypatch: Any) -> None:
    """self.connection must be None after event loop raises."""
    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    class CrashConn:
        def __init__(self) -> None:
            class _Session:
                async def update(self, **_kw: Any) -> None:
                    return None

            self.session = _Session()

            class _Item:
                async def create(self, **_kw: Any) -> None:
                    return None

            class _Conversation:
                item = _Item()

            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None:
                    return None

            self.response = _Response()

        async def __aenter__(self) -> "CrashConn":
            return self

        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
            return False

        async def close(self) -> None:
            return None

        def __aiter__(self) -> "CrashConn":
            return self

        async def __anext__(self) -> None:
            raise RuntimeError("event loop crash")

    class FakeRealtime:
        def connect(self, **_kw: Any) -> CrashConn:
            return CrashConn()

    class FakeClient:
        def __init__(self, **_kw: Any) -> None:
            self.realtime = FakeRealtime()

    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)
    handler.client = FakeClient()
    with pytest.raises(RuntimeError, match="event loop crash"):
        await handler._run_realtime_session()

    assert handler.connection is None, "connection must be cleared after event loop crash"
    assert not handler._connected_event.is_set(), "connected event must be cleared"


@pytest.mark.asyncio
async def test_connection_cleared_on_restart_failure(monkeypatch: Any) -> None:
    """_restart_session cleans up even when new session fails during event loop."""
    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    class FailConn:
        def __init__(self) -> None:
            class _Session:
                async def update(self, **_kw: Any) -> None:
                    return None

            self.session = _Session()

            class _Item:
                async def create(self, **_kw: Any) -> None:
                    return None

            class _Conversation:
                item = _Item()

            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None:
                    return None

            self.response = _Response()

        async def __aenter__(self) -> "FailConn":
            return self

        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
            return False

        async def close(self) -> None:
            return None

        def __aiter__(self) -> "FailConn":
            return self

        async def __anext__(self) -> None:
            raise RuntimeError("restart crash")

    class FakeRealtime:
        def connect(self, **_kw: Any) -> FailConn:
            return FailConn()

    class FakeClient:
        def __init__(self, **_kw: Any) -> None:
            self.realtime = FakeRealtime()

    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)
    handler.client = FakeClient()
    await handler._restart_session()
    await asyncio.sleep(0.1)

    assert handler.connection is None, "connection must be cleaned up after restart failure"


# ---- Receive guard tests ----


@pytest.mark.asyncio
async def test_receive_guards_closed_connection() -> None:
    """receive() silently drops frames when connection is None."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)
    handler.connection = None

    frame = (24000, np.zeros(480, dtype=np.int16))
    await handler.receive(frame)


# ====================================================================
# NEW: receive() audio processing tests
# ====================================================================


class TestReceive:
    @pytest.mark.asyncio
    async def test_sends_mono_audio_to_connection(self) -> None:
        """receive() encodes mono audio as base64 and sends to connection."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        samples = np.array([100, -200, 300, 400], dtype=np.int16)
        await handler.receive((24000, samples))

        assert len(conn.appended_audio) == 1
        decoded = np.frombuffer(base64.b64decode(conn.appended_audio[0]), dtype=np.int16)
        np.testing.assert_array_equal(decoded, samples)

    @pytest.mark.asyncio
    async def test_converts_stereo_to_mono(self) -> None:
        """receive() extracts first channel from stereo audio."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        # Stereo: shape (N, 2), channels-last
        left = np.array([10, 20, 30, 40], dtype=np.int16)
        right = np.array([50, 60, 70, 80], dtype=np.int16)
        stereo = np.column_stack([left, right])
        await handler.receive((24000, stereo))

        assert len(conn.appended_audio) == 1
        decoded = np.frombuffer(base64.b64decode(conn.appended_audio[0]), dtype=np.int16)
        # Should have used first channel
        assert len(decoded) == 4

    @pytest.mark.asyncio
    async def test_resamples_when_rate_differs(self) -> None:
        """receive() resamples audio when input rate != expected rate."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        # Send at 48000 Hz (double the expected 24000)
        samples = np.zeros(960, dtype=np.int16)
        await handler.receive((48000, samples))

        assert len(conn.appended_audio) == 1
        decoded = np.frombuffer(base64.b64decode(conn.appended_audio[0]), dtype=np.int16)
        # Resampled from 960 samples at 48kHz → ~480 samples at 24kHz
        assert abs(len(decoded) - 480) <= 1

    @pytest.mark.asyncio
    async def test_drops_frame_on_send_exception(self) -> None:
        """receive() silently drops frame when connection.append raises."""
        handler = _handler()

        class _BrokenConn:
            class _IAB:
                async def append(self, **_kw: Any) -> None:
                    raise RuntimeError("connection gone")

            input_audio_buffer = _IAB()

        handler.connection = _BrokenConn()
        # Should not raise
        samples = np.zeros(480, dtype=np.int16)
        await handler.receive((24000, samples))

    @pytest.mark.asyncio
    async def test_handles_channels_first_stereo(self) -> None:
        """receive() transposes channels-first stereo (shape[1] > shape[0])."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        # Channels-first: shape (2, N) where shape[1] > shape[0]
        left = np.array([10, 20, 30, 40, 50], dtype=np.int16)
        right = np.array([60, 70, 80, 90, 100], dtype=np.int16)
        channels_first = np.vstack([left, right])  # (2, 5)
        assert channels_first.shape == (2, 5)

        await handler.receive((24000, channels_first))
        assert len(conn.appended_audio) == 1


# ====================================================================
# NEW: emit() idle detection logic tests
# ====================================================================


class TestEmitIdleLogic:
    @pytest.mark.asyncio
    async def test_returns_queue_item_when_not_idle(self, monkeypatch: Any) -> None:
        """When not idle, emit() returns next item from output_queue."""
        handler = _handler()
        # Recent activity → not idle
        handler.last_activity_time = asyncio.get_event_loop().time()

        sentinel = (24000, np.zeros(480, dtype=np.int16))
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=sentinel))

        result = await handler.emit()
        assert result is sentinel

    @pytest.mark.asyncio
    async def test_skips_idle_when_robot_moving(self, monkeypatch: Any) -> None:
        """No idle signal when movement_manager says robot is not idle."""
        deps = _deps()
        deps.movement_manager.is_idle.return_value = False  # robot moving
        handler = _handler(deps=deps)
        handler.last_activity_time = 0.0  # stale → would trigger idle

        send_idle = AsyncMock()
        monkeypatch.setattr(handler, "send_idle_signal", send_idle)
        monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

        await handler.emit()
        send_idle.assert_not_called()

    @pytest.mark.asyncio
    async def test_triggers_idle_without_mmwave(self, monkeypatch: Any) -> None:
        """Without mmWave, idle triggers send_idle_signal directly."""
        handler = _handler()
        handler.last_activity_time = 0.0  # stale

        send_idle = AsyncMock()
        monkeypatch.setattr(handler, "send_idle_signal", send_idle)
        monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

        await handler.emit()
        send_idle.assert_called_once()

    @pytest.mark.asyncio
    async def test_mmwave_idle_suppressed_by_policy(self, monkeypatch: Any) -> None:
        """With mmWave, idle is suppressed when IdlePolicy.should_trigger returns False."""
        handler = _handler()
        handler.last_activity_time = 0.0

        # should_trigger → False → suppress
        monkeypatch.setattr(handler.idle_policy, "should_trigger", lambda *a, **kw: False)
        monkeypatch.setattr(handler, "_has_tool", lambda name: name == "mmWave")

        send_idle = AsyncMock()
        monkeypatch.setattr(handler, "send_idle_signal", send_idle)
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

        result = await handler.emit()
        send_idle.assert_not_called()
        assert result is None

    @pytest.mark.asyncio
    async def test_mmwave_idle_triggered_by_policy(self, monkeypatch: Any) -> None:
        """With mmWave, idle triggers send_idle_signal when policy approves."""
        handler = _handler()
        handler.last_activity_time = 0.0

        monkeypatch.setattr(handler.idle_policy, "should_trigger", lambda *a, **kw: True)
        monkeypatch.setattr(handler, "_has_tool", lambda name: name == "mmWave")

        send_idle = AsyncMock()
        monkeypatch.setattr(handler, "send_idle_signal", send_idle)
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

        await handler.emit()
        send_idle.assert_called_once()

    @pytest.mark.asyncio
    async def test_activity_time_updated_after_idle_signal(self, monkeypatch: Any) -> None:
        """last_activity_time is refreshed after successful idle signal."""
        handler = _handler()
        handler.last_activity_time = 0.0

        monkeypatch.setattr(handler, "send_idle_signal", AsyncMock())
        monkeypatch.setattr(handler, "_has_tool", lambda _name: False)
        monkeypatch.setattr(rt_mod, "wait_for_item", AsyncMock(return_value=None))

        before = asyncio.get_event_loop().time()
        await handler.emit()

        assert handler.last_activity_time >= before


# ====================================================================
# NEW: send_idle_signal() tests
# ====================================================================


class TestSendIdleSignal:
    @pytest.mark.asyncio
    async def test_sends_mmwave_idle_when_tool_available(self, monkeypatch: Any) -> None:
        """With mmWave tool, idle signal includes locate_and_measure instructions."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        monkeypatch.setattr(handler, "_has_tool", lambda name: name == "mmWave")

        await handler.send_idle_signal(60.0)

        assert handler.is_idle_tool_call is True
        # Should have created a message and a response
        assert len(conn.item_create_calls) == 1
        assert len(conn.response_create_calls) == 1
        # Response should require tool use
        resp_kw = conn.response_create_calls[0]
        assert resp_kw["response"]["tool_choice"] == "required"

    @pytest.mark.asyncio
    async def test_sends_generic_idle_without_mmwave(self, monkeypatch: Any) -> None:
        """Without mmWave, idle signal sends generic creative prompt."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        monkeypatch.setattr(handler, "_has_tool", lambda _name: False)

        await handler.send_idle_signal(30.0)

        assert handler.is_idle_tool_call is True
        assert len(conn.item_create_calls) == 1
        # Check message content references being creative
        item = conn.item_create_calls[0]["item"]
        text = item["content"][0]["text"]
        assert "creative" in text.lower() or "idle" in text.lower()

    @pytest.mark.asyncio
    async def test_no_op_without_connection(self, monkeypatch: Any) -> None:
        """send_idle_signal returns early when there is no connection."""
        handler = _handler()
        handler.connection = None
        monkeypatch.setattr(handler, "_has_tool", lambda _name: False)

        # Should not raise
        await handler.send_idle_signal(15.0)
        # is_idle_tool_call was set before the guard
        assert handler.is_idle_tool_call is True

    @pytest.mark.asyncio
    async def test_mmwave_idle_sweep_flag_matches_policy(self, monkeypatch: Any) -> None:
        """sweep_if_unseen in idle instructions matches idle_policy.sweep_allowed."""
        handler = _handler()
        conn = _EventConn()
        handler.connection = conn
        monkeypatch.setattr(handler, "_has_tool", lambda name: name == "mmWave")

        # Force sweep_allowed to return True
        monkeypatch.setattr(handler.idle_policy, "sweep_allowed", lambda _now: True)

        await handler.send_idle_signal(60.0)

        resp_instructions = conn.response_create_calls[0]["response"]["instructions"]
        assert "sweep_if_unseen=true" in resp_instructions.lower()


# ====================================================================
# NEW: Event loop (_run_realtime_session) routing tests
# ====================================================================


class TestEventLoop:
    @pytest.mark.asyncio
    async def test_speech_started_sets_listening_and_resets_wobbler(self, monkeypatch: Any) -> None:
        """speech_started event sets listening=True and resets head wobbler."""
        wobbler = MagicMock()
        deps = _deps(head_wobbler=wobbler)
        handler = _handler(deps=deps)

        events = [_FakeEvent(type="input_audio_buffer.speech_started")]
        await _run_session_with_events(handler, events, monkeypatch)

        deps.movement_manager.set_listening.assert_called_with(True)
        wobbler.reset.assert_called()

    @pytest.mark.asyncio
    async def test_speech_stopped_clears_listening(self, monkeypatch: Any) -> None:
        """speech_stopped event sets listening=False."""
        handler = _handler()

        events = [_FakeEvent(type="input_audio_buffer.speech_stopped")]
        await _run_session_with_events(handler, events, monkeypatch)

        handler.deps.movement_manager.set_listening.assert_called_with(False)

    @pytest.mark.asyncio
    async def test_response_done_accumulates_cost(self, monkeypatch: Any) -> None:
        """response.done with usage data accumulates cost."""
        handler = _handler()
        usage = _make_usage(audio_in=100_000, audio_out=50_000)
        response = _FakeEvent(usage=usage)

        events = [_FakeEvent(type="response.done", response=response)]
        await _run_session_with_events(handler, events, monkeypatch)

        assert handler.cumulative_cost > 0
        assert isinstance(handler.cumulative_cost, Decimal)

    @pytest.mark.asyncio
    async def test_response_done_no_usage_warns(self, monkeypatch: Any, caplog: Any) -> None:
        """response.done without usage logs a warning."""
        caplog.set_level(logging.WARNING)
        handler = _handler()

        events = [_FakeEvent(type="response.done", response=_FakeEvent(usage=None))]
        await _run_session_with_events(handler, events, monkeypatch)

        assert any("No usage data" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_response_done_no_response_object(self, monkeypatch: Any, caplog: Any) -> None:
        """response.done with no response attribute still works."""
        caplog.set_level(logging.WARNING)
        handler = _handler()

        events = [_FakeEvent(type="response.done")]
        await _run_session_with_events(handler, events, monkeypatch)

        assert any("No usage data" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_audio_delta_enqueues_audio(self, monkeypatch: Any) -> None:
        """Audio delta events decode and enqueue audio data."""
        handler = _handler()

        samples = np.array([100, -200, 300], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        events = [_FakeEvent(type="response.audio.delta", delta=delta_b64)]
        await _run_session_with_events(handler, events, monkeypatch)

        # Check output_queue got audio data
        assert not handler.output_queue.empty()

    @pytest.mark.asyncio
    async def test_output_audio_delta_variant(self, monkeypatch: Any) -> None:
        """response.output_audio.delta is also handled."""
        handler = _handler()

        samples = np.array([42], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        events = [_FakeEvent(type="response.output_audio.delta", delta=delta_b64)]
        await _run_session_with_events(handler, events, monkeypatch)

        assert not handler.output_queue.empty()

    @pytest.mark.asyncio
    async def test_tool_dispatch_event_fires_dispatcher(self, monkeypatch: Any) -> None:
        """function_call_arguments.done routes to tool dispatcher."""
        handler = _handler()

        events = [
            _FakeEvent(
                type="response.function_call_arguments.done",
                name="do_nothing",
                arguments="{}",
                call_id="call-42",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch, dispatch_return={"status": "ok"})

    @pytest.mark.asyncio
    async def test_invalid_tool_call_skipped(self, monkeypatch: Any, caplog: Any) -> None:
        """function_call_arguments.done with non-string name is skipped."""
        caplog.set_level(logging.ERROR)
        handler = _handler()

        events = [
            _FakeEvent(
                type="response.function_call_arguments.done",
                name=None,
                arguments="{}",
                call_id="call-bad",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        assert any("Invalid tool call" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_tool_dispatch_resets_idle_flag(self, monkeypatch: Any) -> None:
        """is_idle_tool_call is cleared after dispatching an idle tool call."""
        handler = _handler()
        handler.is_idle_tool_call = True

        events = [
            _FakeEvent(
                type="response.function_call_arguments.done",
                name="mmWave",
                arguments='{"mode":"scan"}',
                call_id="call-idle",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        assert handler.is_idle_tool_call is False

    @pytest.mark.asyncio
    async def test_error_event_enqueued(self, monkeypatch: Any) -> None:
        """Error events are enqueued as assistant messages."""
        handler = _handler()

        err_obj = _FakeEvent(message="rate_limit_exceeded", code="rate_limit")
        events = [_FakeEvent(type="error", error=err_obj)]
        await _run_session_with_events(handler, events, monkeypatch)

        # Drain queue and find error output
        items = []
        while not handler.output_queue.empty():
            items.append(handler.output_queue.get_nowait())
        # Should have at least one AdditionalOutputs with error content
        assert len(items) >= 1

    @pytest.mark.asyncio
    async def test_ignorable_error_not_enqueued(self, monkeypatch: Any) -> None:
        """Error events with ignorable codes don't enqueue output."""
        handler = _handler()

        err_obj = _FakeEvent(message="empty buffer", code="input_audio_buffer_commit_empty")
        events = [_FakeEvent(type="error", error=err_obj)]
        await _run_session_with_events(handler, events, monkeypatch)

        # Queue should be empty (no output for ignorable errors)
        assert handler.output_queue.empty()

    @pytest.mark.asyncio
    async def test_conversation_already_active_not_enqueued(self, monkeypatch: Any) -> None:
        """conversation_already_has_active_response error is silently ignored."""
        handler = _handler()

        err_obj = _FakeEvent(message="already active", code="conversation_already_has_active_response")
        events = [_FakeEvent(type="error", error=err_obj)]
        await _run_session_with_events(handler, events, monkeypatch)

        assert handler.output_queue.empty()

    @pytest.mark.asyncio
    async def test_connected_event_set_during_loop(self, monkeypatch: Any) -> None:
        """_connected_event is set during the event loop and cleared after."""
        handler = _handler()
        assert not handler._connected_event.is_set()

        # Track whether _connected_event was set when movement_manager.set_listening was called
        # (which happens during speech_stopped event processing, inside the loop)
        was_connected: list[bool] = []
        original_set_listening = handler.deps.movement_manager.set_listening

        def _spy_set_listening(val: Any) -> None:
            was_connected.append(handler._connected_event.is_set())
            original_set_listening(val)

        handler.deps.movement_manager.set_listening = _spy_set_listening

        events = [_FakeEvent(type="input_audio_buffer.speech_stopped")]
        await _run_session_with_events(handler, events, monkeypatch)

        # During the loop the event should have been set
        assert was_connected == [True]
        # After the loop it should be cleared
        assert not handler._connected_event.is_set()

    @pytest.mark.asyncio
    async def test_transcript_completed_enqueues(self, monkeypatch: Any) -> None:
        """Completed transcript events enqueue user output."""
        handler = _handler()

        events = [
            _FakeEvent(
                type="conversation.item.input_audio_transcription.completed",
                transcript="hello world",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        # Drain queue
        items = []
        while not handler.output_queue.empty():
            items.append(handler.output_queue.get_nowait())
        assert len(items) >= 1

    @pytest.mark.asyncio
    async def test_assistant_transcript_done_enqueues(self, monkeypatch: Any) -> None:
        """Assistant transcript done events enqueue assistant output."""
        handler = _handler()

        events = [
            _FakeEvent(
                type="response.audio_transcript.done",
                transcript="I'm doing well",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        items = []
        while not handler.output_queue.empty():
            items.append(handler.output_queue.get_nowait())
        assert len(items) >= 1

    @pytest.mark.asyncio
    async def test_output_audio_transcript_done_variant(self, monkeypatch: Any) -> None:
        """response.output_audio_transcript.done variant is also handled."""
        handler = _handler()

        events = [
            _FakeEvent(
                type="response.output_audio_transcript.done",
                transcript="response variant",
            ),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        items = []
        while not handler.output_queue.empty():
            items.append(handler.output_queue.get_nowait())
        assert len(items) >= 1

    @pytest.mark.asyncio
    async def test_session_update_called_with_instructions(self, monkeypatch: Any) -> None:
        """_run_realtime_session sends session.update with instructions and tools."""
        handler = _handler()

        tool_specs = [{"type": "function", "name": "dance", "description": "Dance"}]
        conn = await _run_session_with_events(handler, [], monkeypatch, tool_specs=tool_specs)

        assert len(conn.session_update_calls) == 1
        session = conn.session_update_calls[0]["session"]
        assert session["instructions"] == "test instructions"
        assert session["tools"] == tool_specs

    @pytest.mark.asyncio
    async def test_session_update_failure_aborts(self, monkeypatch: Any, caplog: Any) -> None:
        """If session.update fails, the session aborts without entering the event loop."""
        caplog.set_level(logging.ERROR)
        handler = _handler()

        class _FailSession:
            async def update(self, **_kw: Any) -> None:
                raise RuntimeError("session config error")

        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "test")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")
        monkeypatch.setattr(rt_mod, "get_tool_specs", lambda: [])

        client, conn = _make_event_client([])
        conn.session = _FailSession()  # type: ignore[assignment]
        handler.client = client
        # Should return without raising (early abort)
        await handler._run_realtime_session()

        assert any("session.update failed" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_multiple_cost_accumulation(self, monkeypatch: Any) -> None:
        """Multiple response.done events accumulate cost correctly."""
        handler = _handler()

        usage1 = _make_usage(audio_in=500_000, audio_out=500_000)
        usage2 = _make_usage(text_in=1_000_000, text_out=1_000_000)

        events = [
            _FakeEvent(type="response.done", response=_FakeEvent(usage=usage1)),
            _FakeEvent(type="response.done", response=_FakeEvent(usage=usage2)),
        ]
        await _run_session_with_events(handler, events, monkeypatch)

        # Both should contribute to cost
        assert handler.cumulative_cost > 0
        # Verify it's the sum of both (not just the last)
        cost1 = _compute_response_cost(usage1)
        cost2 = _compute_response_cost(usage2)
        assert handler.cumulative_cost == cost1 + cost2


# ====================================================================
# NEW: shutdown() tests
# ====================================================================


class TestShutdown:
    @pytest.mark.asyncio
    async def test_cancels_handlers_and_flushes(self) -> None:
        """shutdown() cancels transcript handler, dispatcher, and flushes light baseline."""
        handler = _handler()

        # Simulate that _run_realtime_session has been called and set up internal handlers
        transcript = MagicMock()
        transcript.cancel_pending = AsyncMock()
        handler._transcript_handler = transcript

        dispatcher = MagicMock()
        dispatcher.cancel = AsyncMock()
        handler._dispatcher = dispatcher

        handler.connection = None

        await handler.shutdown()

        assert handler._shutdown_requested is True
        transcript.cancel_pending.assert_called_once()
        dispatcher.cancel.assert_called_once()

    @pytest.mark.asyncio
    async def test_drains_output_queue(self) -> None:
        """shutdown() drains remaining items from output_queue."""
        handler = _handler()
        handler.connection = None

        # Pre-fill queue
        handler.output_queue.put_nowait((24000, np.zeros(480, dtype=np.int16)))
        handler.output_queue.put_nowait((24000, np.zeros(480, dtype=np.int16)))

        await handler.shutdown()

        assert handler.output_queue.empty()

    @pytest.mark.asyncio
    async def test_closes_connection(self) -> None:
        """shutdown() closes and clears the connection."""
        handler = _handler()
        conn = MagicMock()
        conn.close = AsyncMock()
        handler.connection = conn

        await handler.shutdown()

        conn.close.assert_called_once()
        assert handler.connection is None

    @pytest.mark.asyncio
    async def test_handles_connection_already_closed(self) -> None:
        """shutdown() handles ConnectionClosedError during close."""
        handler = _handler()

        FakeCCE = type("FakeCCE", (Exception,), {})

        conn = MagicMock()
        conn.close = AsyncMock(side_effect=FakeCCE("already closed"))
        handler.connection = conn

        with patch.object(rt_mod, "ConnectionClosedError", FakeCCE):
            await handler.shutdown()

        assert handler.connection is None

    @pytest.mark.asyncio
    async def test_flushes_light_orchestrator(self) -> None:
        """shutdown() calls light_orchestrator.flush()."""
        handler = _handler()
        handler.connection = None

        flush_mock = MagicMock()
        handler.light_orchestrator.flush = flush_mock

        await handler.shutdown()

        flush_mock.assert_called_once()


# ====================================================================
# NEW: apply_personality() tests
# ====================================================================


class TestApplyPersonality:
    @pytest.mark.asyncio
    async def test_no_connection_records_for_later(self, monkeypatch: Any) -> None:
        """Without a connection, personality is recorded for next session."""
        handler = _handler()
        handler.connection = None

        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "new instructions")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "sage")

        result = await handler.apply_personality("my_profile")
        assert "next connection" in result.lower() or "will take effect" in result.lower()

    @pytest.mark.asyncio
    async def test_with_connection_attempts_restart(self, monkeypatch: Any) -> None:
        """With an active connection, apply_personality tries live update + restart."""
        handler = _handler()

        conn = MagicMock()
        conn.session = MagicMock()
        conn.session.update = AsyncMock()
        handler.connection = conn

        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "new")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")

        restart_mock = AsyncMock()
        monkeypatch.setattr(handler, "_restart_session", restart_mock)

        result = await handler.apply_personality("new_profile")

        conn.session.update.assert_called_once()
        restart_mock.assert_called_once()
        assert "restarted" in result.lower()

    @pytest.mark.asyncio
    async def test_error_returns_message(self, monkeypatch: Any) -> None:
        """apply_personality returns error message on failure."""
        handler = _handler()
        handler.connection = None

        # Force get_session_instructions to fail
        def _fail() -> str:
            raise RuntimeError("broken prompt")

        monkeypatch.setattr(rt_mod, "get_session_instructions", _fail)

        result = await handler.apply_personality("broken")
        assert "failed" in result.lower()

    @pytest.mark.asyncio
    async def test_live_update_failure_still_restarts(self, monkeypatch: Any) -> None:
        """If live update fails, restart is still attempted."""
        handler = _handler()

        conn = MagicMock()
        conn.session = MagicMock()
        conn.session.update = AsyncMock(side_effect=RuntimeError("live update error"))
        handler.connection = conn

        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "new")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")

        restart_mock = AsyncMock()
        monkeypatch.setattr(handler, "_restart_session", restart_mock)

        result = await handler.apply_personality("profile")
        restart_mock.assert_called_once()
        assert "restarted" in result.lower()

    @pytest.mark.asyncio
    async def test_restart_failure_still_returns(self, monkeypatch: Any) -> None:
        """If restart fails, a fallback message is returned."""
        handler = _handler()

        conn = MagicMock()
        conn.session = MagicMock()
        conn.session.update = AsyncMock()
        handler.connection = conn

        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "new")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")
        monkeypatch.setattr(handler, "_restart_session", AsyncMock(side_effect=RuntimeError("restart failed")))

        result = await handler.apply_personality("profile")
        assert "next connection" in result.lower() or "will take effect" in result.lower()


# ====================================================================
# NEW: _restart_session() tests
# ====================================================================


class TestRestartSession:
    @pytest.mark.asyncio
    async def test_no_client_returns_early(self, caplog: Any) -> None:
        """_restart_session logs warning and returns if client is not set."""
        caplog.set_level(logging.WARNING)
        handler = _handler()
        handler.connection = None
        # Ensure no client attribute
        if hasattr(handler, "client"):
            delattr(handler, "client")

        await handler._restart_session()

        assert any("Cannot restart" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_closes_existing_connection_first(self, monkeypatch: Any) -> None:
        """_restart_session closes existing connection before creating new one."""
        handler = _handler()

        old_conn = MagicMock()
        old_conn.close = AsyncMock()
        handler.connection = old_conn

        client, _ = _make_event_client([])
        handler.client = client
        monkeypatch.setattr(rt_mod, "get_session_instructions", lambda: "test")
        monkeypatch.setattr(rt_mod, "get_session_voice", lambda: "cedar")
        monkeypatch.setattr(rt_mod, "get_tool_specs", lambda: [])
        monkeypatch.setattr(rt_mod, "dispatch_tool_call", AsyncMock(return_value={}))

        await handler._restart_session()
        await asyncio.sleep(0.1)

        old_conn.close.assert_called_once()


# ====================================================================
# NEW: Miscellaneous helper tests
# ====================================================================


class TestMiscHelpers:
    @pytest.mark.asyncio
    async def test_copy_returns_new_instance(self) -> None:
        """copy() creates a distinct handler with same deps."""
        handler = _handler()
        copied = handler.copy()

        assert copied is not handler
        assert copied.deps is handler.deps
        assert copied.gradio_mode == handler.gradio_mode

    @pytest.mark.asyncio
    async def test_has_tool_checks_specs(self, monkeypatch: Any) -> None:
        """_has_tool returns True only when tool is in specs."""
        handler = _handler()
        specs = [{"name": "dance"}, {"name": "mmWave"}]
        monkeypatch.setattr(rt_mod, "get_tool_specs", lambda: specs)

        assert handler._has_tool("dance") is True
        assert handler._has_tool("mmWave") is True
        assert handler._has_tool("nonexistent") is False

    @pytest.mark.asyncio
    async def test_replace_sensor_state_clears_and_updates(self) -> None:
        """_replace_sensor_state replaces (not merges) sensor state."""
        handler = _handler()
        handler.sensor_state = {"old_key": "old_value", "stale": True}

        handler._replace_sensor_state({"new_key": "new_value"})

        assert handler.sensor_state == {"new_key": "new_value"}
        assert "old_key" not in handler.sensor_state

    @pytest.mark.asyncio
    async def test_touch_activity_updates_timestamp(self) -> None:
        """_touch_activity updates last_activity_time to current loop time."""
        handler = _handler()
        old_time = handler.last_activity_time
        handler._touch_activity()

        assert handler.last_activity_time >= old_time

    @pytest.mark.asyncio
    async def test_resolve_runtime_data_path_with_instance(self) -> None:
        """_resolve_runtime_data_path uses instance_path when set."""
        handler = _handler(instance_path="/tmp/test_instance")
        path = handler._resolve_runtime_data_path("foo.json")

        assert str(path) == "/tmp/test_instance/foo.json"

    @pytest.mark.asyncio
    async def test_resolve_runtime_data_path_without_instance(self) -> None:
        """_resolve_runtime_data_path uses cwd when instance_path is None."""
        handler = _handler()
        assert handler.instance_path is None
        path = handler._resolve_runtime_data_path("bar.json")

        assert path.name == "bar.json"

    @pytest.mark.asyncio
    async def test_initial_state(self) -> None:
        """Handler initializes with expected default state."""
        handler = _handler()

        assert handler.connection is None
        assert handler.cumulative_cost == Decimal(0)
        assert handler.sensor_state == {}
        assert handler.is_idle_tool_call is False
        assert handler._shutdown_requested is False
        assert not handler._connected_event.is_set()
        assert handler.gradio_mode is False


# ====================================================================
# NEW: _persist_api_key_if_needed tests
# ====================================================================


class TestPersistApiKey:
    @pytest.mark.asyncio
    async def test_skips_when_not_gradio_mode(self, caplog: Any) -> None:
        """Persistence is skipped in non-gradio mode."""
        caplog.set_level(logging.WARNING)
        handler = _handler(gradio_mode=False)
        handler._key_source = "textbox"
        handler._provided_api_key = "sk-test"

        handler._persist_api_key_if_needed()

        assert any("Not in Gradio mode" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_skips_when_key_from_env(self) -> None:
        """Persistence is skipped when key came from env, not textbox."""
        handler = _handler(gradio_mode=True)
        handler._key_source = "env"

        handler._persist_api_key_if_needed()
        # No crash, just returns

    @pytest.mark.asyncio
    async def test_skips_when_no_instance_path(self, caplog: Any) -> None:
        """Persistence is skipped when instance_path is None."""
        caplog.set_level(logging.WARNING)
        handler = _handler(gradio_mode=True)
        handler._key_source = "textbox"
        handler._provided_api_key = "sk-test"
        handler.instance_path = None

        handler._persist_api_key_if_needed()

        assert any("Instance path is None" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_writes_env_file(self, tmp_path: Any) -> None:
        """Persists API key to .env file in instance_path."""
        handler = _handler(gradio_mode=True, instance_path=str(tmp_path))
        handler._key_source = "textbox"
        handler._provided_api_key = "sk-test-key-123"

        handler._persist_api_key_if_needed()

        env_file = tmp_path / ".env"
        assert env_file.exists()
        content = env_file.read_text()
        assert "OPENAI_API_KEY=sk-test-key-123" in content

    @pytest.mark.asyncio
    async def test_respects_existing_env(self, tmp_path: Any, caplog: Any) -> None:
        """Does not overwrite an existing .env file."""
        caplog.set_level(logging.INFO)
        env_file = tmp_path / ".env"
        env_file.write_text("OPENAI_API_KEY=existing-key\n")

        handler = _handler(gradio_mode=True, instance_path=str(tmp_path))
        handler._key_source = "textbox"
        handler._provided_api_key = "sk-new-key"

        handler._persist_api_key_if_needed()

        content = env_file.read_text()
        assert "existing-key" in content
        assert "sk-new-key" not in content

    @pytest.mark.asyncio
    async def test_uses_env_example_as_template(self, tmp_path: Any) -> None:
        """Uses .env.example as template when available."""
        example = tmp_path / ".env.example"
        example.write_text("OPENAI_API_KEY=placeholder\nOTHER_VAR=value\n")

        handler = _handler(gradio_mode=True, instance_path=str(tmp_path))
        handler._key_source = "textbox"
        handler._provided_api_key = "sk-real-key"

        handler._persist_api_key_if_needed()

        env_file = tmp_path / ".env"
        content = env_file.read_text()
        assert "OPENAI_API_KEY=sk-real-key" in content
        assert "OTHER_VAR=value" in content
