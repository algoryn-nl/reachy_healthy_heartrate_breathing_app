"""Tests for AudioRouter audio delta handling."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import base64
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest

from healthy_heartrate_breathing.audio_router import AudioRouter


def _router(**overrides: object) -> AudioRouter:
    defaults = {
        "output_sample_rate": 24000,
        "enqueue_audio": AsyncMock(),
        "feed_head_wobbler": None,
        "on_activity": MagicMock(),
    }
    defaults.update(overrides)
    return AudioRouter(**defaults)


class TestOnAudioDelta:
    @pytest.mark.asyncio
    async def test_decodes_and_enqueues(self) -> None:
        enqueue = AsyncMock()
        samples = np.array([100, -200, 300], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        r = _router(enqueue_audio=enqueue)
        await r.on_audio_delta(delta_b64)

        enqueue.assert_called_once()
        rate, arr = enqueue.call_args[0]
        assert rate == 24000
        np.testing.assert_array_equal(arr.flatten(), samples)

    @pytest.mark.asyncio
    async def test_feeds_head_wobbler(self) -> None:
        wobbler_feed = MagicMock()
        samples = np.array([1, 2], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        r = _router(feed_head_wobbler=wobbler_feed)
        await r.on_audio_delta(delta_b64)

        wobbler_feed.assert_called_once_with(delta_b64)

    @pytest.mark.asyncio
    async def test_calls_on_activity(self) -> None:
        activity_cb = MagicMock()
        samples = np.array([1], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        r = _router(on_activity=activity_cb)
        await r.on_audio_delta(delta_b64)

        activity_cb.assert_called_once()

    @pytest.mark.asyncio
    async def test_no_wobbler_still_works(self) -> None:
        enqueue = AsyncMock()
        samples = np.array([42], dtype=np.int16)
        delta_b64 = base64.b64encode(samples.tobytes()).decode("utf-8")

        r = _router(enqueue_audio=enqueue, feed_head_wobbler=None)
        await r.on_audio_delta(delta_b64)

        enqueue.assert_called_once()
