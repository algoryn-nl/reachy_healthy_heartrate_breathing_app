"""Transcript debouncing and output routing."""

from __future__ import annotations
import asyncio
import logging
from typing import Any, Callable, Awaitable


logger = logging.getLogger(__name__)


class TranscriptHandler:
    """Manages partial transcript debouncing and completed transcript output.

    Testable without OpenAI connections -- output is routed via an injected
    enqueue callback.
    """

    def __init__(  # noqa: D107
        self,
        *,
        debounce_delay: float = 0.5,
        enqueue_output: Callable[[dict[str, Any]], Awaitable[None]],
    ) -> None:
        self.debounce_delay = debounce_delay
        self._enqueue_output = enqueue_output
        self._partial_task: asyncio.Task[None] | None = None
        self._partial_sequence: int = 0

    async def on_partial(self, transcript: str) -> None:
        """Handle a partial (in-progress) user transcript with debouncing."""
        self._partial_sequence += 1
        current_seq = self._partial_sequence

        await self._cancel_debounce_task()

        self._partial_task = asyncio.create_task(self._emit_debounced(transcript, current_seq))

    async def on_user_completed(self, transcript: str) -> None:
        """Handle a completed user transcript, cancelling any pending partial."""
        await self._cancel_debounce_task()
        await self._enqueue_output({"role": "user", "content": transcript})

    async def on_assistant_done(self, transcript: str) -> None:
        """Handle a completed assistant transcript."""
        await self._enqueue_output({"role": "assistant", "content": transcript})

    async def cancel_pending(self) -> None:
        """Cancel any in-flight debounce task (for shutdown)."""
        await self._cancel_debounce_task()

    async def _emit_debounced(self, transcript: str, sequence: int) -> None:
        """Emit partial transcript after debounce delay if still current."""
        try:
            await asyncio.sleep(self.debounce_delay)
            if self._partial_sequence == sequence:
                await self._enqueue_output({"role": "user_partial", "content": transcript})
                logger.debug("Debounced partial emitted: %s", transcript)
        except asyncio.CancelledError:
            logger.debug("Debounced partial cancelled")
            raise

    async def _cancel_debounce_task(self) -> None:
        """Cancel and await the current debounce task if active."""
        if self._partial_task is not None and not self._partial_task.done():
            self._partial_task.cancel()
            try:
                await self._partial_task
            except asyncio.CancelledError:
                pass
