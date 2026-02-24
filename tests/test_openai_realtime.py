import asyncio
import logging
from typing import Any
from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest

import healthy_heartrate_breathing.openai_realtime as rt_mod
from healthy_heartrate_breathing.openai_realtime import OpenaiRealtimeHandler, _compute_response_cost
from healthy_heartrate_breathing.tools.core_tools import ToolDependencies


def _build_handler(loop: asyncio.AbstractEventLoop) -> OpenaiRealtimeHandler:
    asyncio.set_event_loop(loop)
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    return OpenaiRealtimeHandler(deps)


def test_format_timestamp_uses_wall_clock() -> None:
    """Test that format_timestamp uses wall clock time."""
    loop = asyncio.new_event_loop()
    try:
        print("Testing format_timestamp...")
        handler = _build_handler(loop)
        formatted = handler.format_timestamp()
        print(f"Formatted timestamp: {formatted}")
    finally:
        asyncio.set_event_loop(None)
        loop.close()

    # Extract year from "[YYYY-MM-DD ...]"
    year = int(formatted[1:5])
    assert year == datetime.now(timezone.utc).year

@pytest.mark.asyncio
async def test_start_up_retries_on_abrupt_close(monkeypatch: Any, caplog: Any) -> None:
    """First connection dies with ConnectionClosedError during iteration -> retried.

    Second connection iterates cleanly (no events) -> start_up returns without raising.
    Ensures handler clears self.connection at the end.
    """
    caplog.set_level(logging.WARNING)

    # Use a local Exception as the module's ConnectionClosedError to avoid ws dependency
    FakeCCE = type("FakeCCE", (Exception,), {})
    monkeypatch.setattr(rt_mod, "ConnectionClosedError", FakeCCE)

    # Make asyncio.sleep return immediately (for backoff)
    async def _fast_sleep(*_a: Any, **_kw: Any) -> None: return None
    monkeypatch.setattr(asyncio, "sleep", _fast_sleep, raising=False)

    attempt_counter = {"n": 0}

    class FakeConn:
        """Minimal realtime connection stub."""

        def __init__(self, mode: str):
            self._mode = mode

            class _Session:
                async def update(self, **_kw: Any) -> None: return None
            self.session = _Session()

            class _InputAudioBuffer:
                async def append(self, **_kw: Any) -> None: return None
            self.input_audio_buffer = _InputAudioBuffer()

            class _Item:
                async def create(self, **_kw: Any) -> None: return None

            class _Conversation:
                item = _Item()
            self.conversation = _Conversation()

            class _Response:
                async def create(self, **_kw: Any) -> None: return None
                async def cancel(self, **_kw: Any) -> None: return None
            self.response = _Response()

        async def __aenter__(self) -> "FakeConn": return self
        async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool: return False
        async def close(self) -> None: return None

        # Async iterator protocol
        def __aiter__(self) -> "FakeConn": return self
        async def __anext__(self) -> None:
            if self._mode == "raise_on_iter":
                raise FakeCCE("abrupt close (simulated)")
            raise StopAsyncIteration  # clean exit (no events)

    class FakeRealtime:
        def connect(self, **_kw: Any) -> FakeConn:
            attempt_counter["n"] += 1
            mode = "raise_on_iter" if attempt_counter["n"] == 1 else "clean"
            return FakeConn(mode)

    class FakeClient:
        def __init__(self, **_kw: Any) -> None: self.realtime = FakeRealtime()

    # Patch the OpenAI client used by the handler
    monkeypatch.setattr(rt_mod, "AsyncOpenAI", FakeClient)

    # Build handler with minimal deps
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = rt_mod.OpenaiRealtimeHandler(deps)

    # Run: should retry once and exit cleanly
    await handler.start_up()

    # Validate: two attempts total (fail -> retry -> succeed), and connection cleared
    assert attempt_counter["n"] == 2
    assert handler.connection is None

    # Optional: confirm we logged the unexpected close once
    warnings = [r for r in caplog.records if r.levelname == "WARNING" and "closed unexpectedly" in r.msg]
    assert len(warnings) == 1


# ---- Cost calculation tests ----


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


@pytest.mark.parametrize(
    "usage_kwargs, expect_positive",
    [
        # All token types present → positive cost
        ({"audio_in": 1000, "text_in": 2000, "image_in": 500, "audio_out": 800, "text_out": 300}, True),
        # All None tokens → must not crash
        ({"audio_in": None, "text_in": None, "image_in": None, "audio_out": None, "text_out": None}, False),
        # Mix of None and valid ints
        ({"audio_in": None, "text_in": 500, "image_in": None, "audio_out": 1000, "text_out": None}, True),
        # Missing input/output details entirely
        ({"has_input": False, "has_output": False}, False),
    ],
    ids=["normal", "all_none", "mixed", "missing_details"],
)
def test_compute_response_cost(usage_kwargs: dict[str, Any], expect_positive: bool) -> None:
    """Verify _compute_response_cost handles various token combinations without crashing."""
    usage = _make_usage(**usage_kwargs)
    cost = _compute_response_cost(usage)
    if expect_positive:
        assert cost > 0
    else:
        assert cost == 0.0


# ---- Idle flag reset tests ----


@pytest.mark.asyncio
async def test_idle_flag_reset_on_send_idle_signal_exception(monkeypatch: Any) -> None:
    """is_idle_tool_call must be reset when send_idle_signal raises."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)

    # Force idle conditions: long idle duration, not moving
    handler.last_activity_time = 0.0
    handler.start_time = 0.0
    deps.movement_manager.is_idle.return_value = True

    # Make send_idle_signal set flag (as the real method does) then raise
    async def _raise(*_a: Any, **_kw: Any) -> None:
        handler.is_idle_tool_call = True
        raise ConnectionError("simulated disconnect")

    monkeypatch.setattr(handler, "send_idle_signal", _raise)

    # Stub _has_tool to return False (no mmWave -> uses simple idle_interval)
    monkeypatch.setattr(handler, "_has_tool", lambda _name: False)

    # Stub wait_for_item so it doesn't block
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
        """Connection that crashes during event iteration."""

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
    handler.client = FakeClient()  # type: ignore[assignment]

    # Call _run_realtime_session directly (as _restart_session does via create_task)
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
    handler.client = FakeClient()  # type: ignore[assignment]

    await handler._restart_session()
    # Give the background task time to run and fail
    await asyncio.sleep(0.1)

    assert handler.connection is None, "connection must be cleaned up after restart failure"


# ---- Receive guard tests ----


@pytest.mark.asyncio
async def test_receive_guards_closed_connection() -> None:
    """receive() silently drops frames when connection is None."""
    deps = ToolDependencies(reachy_mini=MagicMock(), movement_manager=MagicMock())
    handler = OpenaiRealtimeHandler(deps)
    handler.connection = None

    # Should not raise
    frame = (24000, np.zeros(480, dtype=np.int16))
    await handler.receive(frame)
