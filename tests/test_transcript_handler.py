"""Tests for TranscriptHandler debouncing and output.

Note: asyncio.sleep calls in this module are intentional.  The production
TranscriptHandler uses real asyncio.sleep for debouncing, so tests must wait
for the debounce window to fire (or expire without firing).  Delays are set to
the minimum viable values.
"""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import asyncio
from typing import Any
from unittest.mock import AsyncMock
from collections.abc import Callable

import pytest

from healthy_heartrate_breathing.transcript_handler import TranscriptHandler


@pytest.fixture
def handler_factory() -> Callable[..., TranscriptHandler]:
    """Return a factory that builds a TranscriptHandler with sane defaults."""

    def _make(**overrides: Any) -> TranscriptHandler:
        defaults: dict[str, Any] = {
            "debounce_delay": 0.05,
            "enqueue_output": AsyncMock(),
        }
        defaults.update(overrides)
        return TranscriptHandler(**defaults)

    return _make


class TestOnPartial:
    @pytest.mark.asyncio
    async def test_emits_after_debounce(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.02)
        await h.on_partial("hello")
        await asyncio.sleep(0.05)
        enqueue.assert_called_once()
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "user_partial"
        assert payload["content"] == "hello"

    @pytest.mark.asyncio
    async def test_later_partial_cancels_earlier(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.05)
        await h.on_partial("hel")
        await asyncio.sleep(0.01)
        await h.on_partial("hello world")
        await asyncio.sleep(0.08)
        # Only the second partial should have been emitted
        assert enqueue.call_count == 1
        payload = enqueue.call_args[0][0]
        assert payload["content"] == "hello world"


class TestOnUserCompleted:
    @pytest.mark.asyncio
    async def test_cancels_pending_partial_and_emits(
        self, handler_factory: Callable[..., TranscriptHandler]
    ) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.5)
        await h.on_partial("hel")
        await h.on_user_completed("hello there")
        await asyncio.sleep(0.6)
        # Should see exactly 1 call (the completed), not 2 (partial + completed)
        assert enqueue.call_count == 1
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "user"
        assert payload["content"] == "hello there"

    @pytest.mark.asyncio
    async def test_emits_user_role(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue)
        await h.on_user_completed("test transcript")
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["role"] == "user"


class TestOnAssistantDone:
    @pytest.mark.asyncio
    async def test_emits_assistant_role(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue)
        await h.on_assistant_done("I'm fine, thanks")
        assert enqueue.call_count == 1
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "assistant"
        assert payload["content"] == "I'm fine, thanks"


class TestConcurrentOnPartial:
    @pytest.mark.asyncio
    async def test_rapid_burst_only_emits_last(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        """Rapid sequential on_partial() calls: only the final partial survives debounce."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.05)
        for i in range(10):
            await h.on_partial(f"partial-{i}")
        await asyncio.sleep(0.1)
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["content"] == "partial-9"

    @pytest.mark.asyncio
    async def test_overlapping_debounce_windows(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        """Partials arriving mid-debounce cancel the previous window."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.06)
        await h.on_partial("first")
        await asyncio.sleep(0.03)  # halfway through first debounce
        await h.on_partial("second")
        await asyncio.sleep(0.03)  # halfway through second debounce
        await h.on_partial("third")
        await asyncio.sleep(0.1)  # wait for third to emit
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["content"] == "third"

    @pytest.mark.asyncio
    async def test_concurrent_gather_only_emits_last(
        self, handler_factory: Callable[..., TranscriptHandler]
    ) -> None:
        """Concurrent on_partial() via gather: only the last-sequenced partial emits."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.05)
        await asyncio.gather(
            h.on_partial("a"),
            h.on_partial("b"),
            h.on_partial("c"),
        )
        await asyncio.sleep(0.1)
        # gather schedules all three coroutines; they interleave at await points in the
        # single-threaded event loop.  Exactly one survives the debounce — the one whose
        # _partial_sequence snapshot matches the final counter value.
        assert enqueue.call_count == 1
        emitted = enqueue.call_args[0][0]["content"]
        assert emitted in {"a", "b", "c"}
        assert enqueue.call_args[0][0]["role"] == "user_partial"

    @pytest.mark.asyncio
    async def test_burst_then_completed_suppresses_all_partials(
        self, handler_factory: Callable[..., TranscriptHandler]
    ) -> None:
        """A completed transcript after a rapid burst cancels all pending partials."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.05)
        for i in range(5):
            await h.on_partial(f"p-{i}")
        await h.on_user_completed("final answer")
        await asyncio.sleep(0.1)
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["role"] == "user"
        assert enqueue.call_args[0][0]["content"] == "final answer"

    @pytest.mark.asyncio
    async def test_sequence_counter_advances_monotonically(
        self, handler_factory: Callable[..., TranscriptHandler]
    ) -> None:
        """Each on_partial() call increments the sequence, ensuring stale emits are dropped."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.05)
        await h.on_partial("a")
        seq_after_a = h._partial_sequence
        await h.on_partial("b")
        seq_after_b = h._partial_sequence
        await h.on_partial("c")
        seq_after_c = h._partial_sequence
        assert seq_after_a < seq_after_b < seq_after_c
        await asyncio.sleep(0.1)
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["content"] == "c"

    @pytest.mark.asyncio
    async def test_spaced_partials_each_emit(self, handler_factory: Callable[..., TranscriptHandler]) -> None:
        """Partials spaced beyond the debounce window each emit independently."""
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.02)
        await h.on_partial("first")
        await asyncio.sleep(0.05)  # first emits
        await h.on_partial("second")
        await asyncio.sleep(0.05)  # second emits
        assert enqueue.call_count == 2
        contents = [call[0][0]["content"] for call in enqueue.call_args_list]
        assert contents == ["first", "second"]


class TestCancelPending:
    @pytest.mark.asyncio
    async def test_cancel_pending_suppresses_emission(
        self, handler_factory: Callable[..., TranscriptHandler]
    ) -> None:
        enqueue = AsyncMock()
        h = handler_factory(enqueue_output=enqueue, debounce_delay=0.5)
        await h.on_partial("hello")
        await h.cancel_pending()
        await asyncio.sleep(0.6)
        enqueue.assert_not_called()
