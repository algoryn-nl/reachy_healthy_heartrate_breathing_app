"""Tests for TranscriptHandler debouncing and output."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import asyncio
from unittest.mock import AsyncMock

import pytest

from healthy_heartrate_breathing.transcript_handler import TranscriptHandler


def _handler(**overrides: object) -> TranscriptHandler:
    defaults = {
        "debounce_delay": 0.05,
        "enqueue_output": AsyncMock(),
    }
    defaults.update(overrides)
    return TranscriptHandler(**defaults)


class TestOnPartial:
    @pytest.mark.asyncio
    async def test_emits_after_debounce(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue, debounce_delay=0.02)
        await h.on_partial("hello")
        await asyncio.sleep(0.05)
        enqueue.assert_called_once()
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "user_partial"
        assert payload["content"] == "hello"

    @pytest.mark.asyncio
    async def test_later_partial_cancels_earlier(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue, debounce_delay=0.05)
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
    async def test_cancels_pending_partial_and_emits(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue, debounce_delay=0.5)
        await h.on_partial("hel")
        await h.on_user_completed("hello there")
        await asyncio.sleep(0.6)
        # Should see exactly 1 call (the completed), not 2 (partial + completed)
        assert enqueue.call_count == 1
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "user"
        assert payload["content"] == "hello there"

    @pytest.mark.asyncio
    async def test_emits_user_role(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue)
        await h.on_user_completed("test transcript")
        assert enqueue.call_count == 1
        assert enqueue.call_args[0][0]["role"] == "user"


class TestOnAssistantDone:
    @pytest.mark.asyncio
    async def test_emits_assistant_role(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue)
        await h.on_assistant_done("I'm fine, thanks")
        assert enqueue.call_count == 1
        payload = enqueue.call_args[0][0]
        assert payload["role"] == "assistant"
        assert payload["content"] == "I'm fine, thanks"


class TestCancelPending:
    @pytest.mark.asyncio
    async def test_cancel_pending_suppresses_emission(self) -> None:
        enqueue = AsyncMock()
        h = _handler(enqueue_output=enqueue, debounce_delay=0.5)
        await h.on_partial("hello")
        await h.cancel_pending()
        await asyncio.sleep(0.6)
        enqueue.assert_not_called()
