"""Tests for WebSocket sensor broadcast."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import json
import asyncio

import pytest

from healthy_heartrate_breathing.sensor_ws import SensorBroadcaster


@pytest.fixture
def broadcaster() -> SensorBroadcaster:
    return SensorBroadcaster()


class TestBroadcaster:
    @pytest.mark.asyncio
    async def test_no_clients_no_error(self, broadcaster: SensorBroadcaster) -> None:
        broadcaster.broadcast({"device_state": "NO_TARGET"})

    @pytest.mark.asyncio
    async def test_broadcast_delivers_to_client(self, broadcaster: SensorBroadcaster) -> None:
        queue: asyncio.Queue[str] = asyncio.Queue()

        class FakeWS:
            async def send_text(self, text: str) -> None:
                await queue.put(text)

        ws = FakeWS()
        broadcaster.connect(ws)
        broadcaster.broadcast({"device_state": "MOVING"})
        # Give the background tasks a chance to run
        for _ in range(20):
            await asyncio.sleep(0)
        msg = await asyncio.wait_for(queue.get(), timeout=1.0)
        data = json.loads(msg)
        assert data["device_state"] == "MOVING"

    @pytest.mark.asyncio
    async def test_disconnect_removes_client(self, broadcaster: SensorBroadcaster) -> None:
        class FakeWS:
            async def send_text(self, text: str) -> None:
                pass

        ws = FakeWS()
        broadcaster.connect(ws)
        assert len(broadcaster._clients) == 1
        broadcaster.disconnect(ws)
        assert len(broadcaster._clients) == 0

    @pytest.mark.asyncio
    async def test_dead_client_auto_removed(self, broadcaster: SensorBroadcaster) -> None:
        class DeadWS:
            async def send_text(self, text: str) -> None:
                raise ConnectionError("gone")

        ws = DeadWS()
        broadcaster.connect(ws)
        broadcaster.broadcast({"test": True})
        for _ in range(20):
            await asyncio.sleep(0)
        assert len(broadcaster._clients) == 0
