"""Audio delta routing: decode, feed wobbler, enqueue output."""

from __future__ import annotations
import base64
import logging
from typing import Callable, Awaitable

import numpy as np
from numpy.typing import NDArray


logger = logging.getLogger(__name__)


class AudioRouter:
    """Routes audio deltas to the output queue and head wobbler.

    Testable without OpenAI connections -- output is routed via injected callbacks.
    """

    def __init__(  # noqa: D107
        self,
        *,
        output_sample_rate: int,
        enqueue_audio: Callable[[int, NDArray[np.int16]], Awaitable[None]],
        feed_head_wobbler: Callable[[str], None] | None,
        on_activity: Callable[[], None],
    ) -> None:
        self.output_sample_rate = output_sample_rate
        self._enqueue_audio = enqueue_audio
        self._feed_head_wobbler = feed_head_wobbler
        self._on_activity = on_activity

    async def on_audio_delta(self, delta: str) -> None:
        """Handle an audio delta event: decode, feed wobbler, enqueue."""
        if self._feed_head_wobbler is not None:
            self._feed_head_wobbler(delta)

        self._on_activity()

        audio_array = np.frombuffer(base64.b64decode(delta), dtype=np.int16).reshape(1, -1)

        await self._enqueue_audio(self.output_sample_rate, audio_array)
