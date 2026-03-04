"""Tests for tool_ok / tool_error helper functions."""

from __future__ import annotations

from healthy_heartrate_breathing.tools.core_tools import tool_error, tool_ok


class TestToolOk:
    def test_default_status(self) -> None:
        assert tool_ok() == {"status": "ok"}

    def test_custom_status(self) -> None:
        assert tool_ok("queued") == {"status": "queued"}

    def test_extra_keys(self) -> None:
        result = tool_ok("queued", move="nod", repeat=2)
        assert result == {"status": "queued", "move": "nod", "repeat": 2}

    def test_no_error_key(self) -> None:
        result = tool_ok("ok", data=123)
        assert "error" not in result


class TestToolError:
    def test_basic_error(self) -> None:
        assert tool_error("boom") == {"error": "boom"}

    def test_extra_keys(self) -> None:
        result = tool_error("serial fail", status="disconnected")
        assert result == {"error": "serial fail", "status": "disconnected"}

    def test_always_has_error_key(self) -> None:
        result = tool_error("fail", tool="mmWave")
        assert "error" in result
        assert result["error"] == "fail"
