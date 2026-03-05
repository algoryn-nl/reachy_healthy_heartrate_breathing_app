"""WebSocket broadcast for live sensor data."""

from __future__ import annotations
import json
import asyncio
import logging
from typing import Any


logger = logging.getLogger(__name__)


class SensorBroadcaster:
    """Manages WebSocket clients and broadcasts sensor state updates."""

    def __init__(self) -> None:
        """Initialize with an empty client set."""
        self._clients: set[Any] = set()

    def connect(self, ws: Any) -> None:
        """Register a WebSocket client."""
        self._clients.add(ws)
        logger.debug("WS client connected (%d total)", len(self._clients))

    def disconnect(self, ws: Any) -> None:
        """Remove a WebSocket client."""
        self._clients.discard(ws)
        logger.debug("WS client disconnected (%d total)", len(self._clients))

    def broadcast(self, data: dict[str, Any]) -> None:
        """Send data to all connected clients (fire-and-forget)."""
        if not self._clients:
            return
        text = json.dumps(data)
        for ws in list(self._clients):
            asyncio.ensure_future(self._safe_send(ws, text))

    async def _safe_send(self, ws: Any, text: str) -> None:
        """Send text to a single client, removing it on failure."""
        try:
            await ws.send_text(text)
        except Exception:
            self._clients.discard(ws)
            logger.debug("Removed dead WS client")
