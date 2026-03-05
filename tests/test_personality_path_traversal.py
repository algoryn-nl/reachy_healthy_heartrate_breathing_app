"""Security tests for path traversal prevention in personality management."""
# ruff: noqa: D101, D102, D103

from pathlib import Path

import pytest

from healthy_heartrate_breathing.headless_personality import (
    _write_profile,
    available_tools_for,
    resolve_profile_dir,
    read_instructions_for,
)


# Payloads that escape the profiles root directory via ../
TRAVERSAL_PAYLOADS = [
    "../../../etc/passwd",
    "../../secret",
    "../sibling_dir",
    "legit/../../escape",
    "user_personalities/../../escape",
]

# Payloads that escape user_personalities/ via ../
# (note: ../X stays in profiles root but escapes user_personalities/)
WRITE_TRAVERSAL_PAYLOADS = [
    "../sibling_dir",
    "../../escape",
    "../../../etc/passwd",
    "legit/../../escape",
]


class TestHeadlessPathTraversal:
    @pytest.mark.parametrize("payload", TRAVERSAL_PAYLOADS)
    def test_resolve_profile_dir_rejects_traversal(self, payload: str) -> None:
        with pytest.raises(ValueError, match="Invalid profile path"):
            resolve_profile_dir(payload)

    def test_resolve_profile_dir_allows_valid_profile(self) -> None:
        result = resolve_profile_dir("_healthy_heartrate_breathing_locked_profile")
        assert result.name == "_healthy_heartrate_breathing_locked_profile"

    def test_resolve_profile_dir_allows_user_personality(self) -> None:
        result = resolve_profile_dir("user_personalities/my_profile")
        assert "user_personalities" in str(result)
        assert result.name == "my_profile"

    @pytest.mark.parametrize("payload", TRAVERSAL_PAYLOADS)
    def test_read_instructions_rejects_traversal(self, payload: str) -> None:
        result = read_instructions_for(payload)
        assert "Could not load instructions" in result or "Invalid profile path" in result

    @pytest.mark.parametrize("payload", TRAVERSAL_PAYLOADS)
    def test_available_tools_rejects_traversal(self, payload: str) -> None:
        tools = available_tools_for(payload)
        assert isinstance(tools, list)

    @pytest.mark.parametrize("payload", WRITE_TRAVERSAL_PAYLOADS)
    def test_write_profile_rejects_traversal(self, payload: str) -> None:
        with pytest.raises(ValueError, match="Invalid profile name"):
            _write_profile(payload, "instructions", "tools", "cedar")

    def test_write_profile_allows_simple_name(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        import healthy_heartrate_breathing.headless_personality as hp

        monkeypatch.setattr(hp, "_profiles_root", lambda: tmp_path)
        _write_profile("safe_name", "instr", "tools", "cedar")
        assert (tmp_path / "user_personalities" / "safe_name" / "instructions.txt").exists()
