"""Tests for tool registry thread safety and module loading cleanup."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import sys
import threading
from pathlib import Path
from unittest.mock import patch

import pytest

from healthy_heartrate_breathing.tools import core_tools


class TestRegistryThreadSafety:
    """PY-HIGH-2: Two threads calling get_tool_specs() — registry initialized exactly once."""

    def test_concurrent_init_runs_once(self, tmp_path: Path) -> None:
        """Two threads calling get_tool_specs() concurrently — _load_profile_tools called once."""
        call_count = 0
        original_load = core_tools._load_profile_tools

        def counting_load() -> list[str]:
            nonlocal call_count
            call_count += 1
            return original_load()

        # Reset registry state
        old_tools = core_tools.ALL_TOOLS.copy()
        old_specs = core_tools.ALL_TOOL_SPECS.copy()
        old_init = core_tools._TOOLS_INITIALIZED
        try:
            core_tools.ALL_TOOLS.clear()
            core_tools.ALL_TOOL_SPECS.clear()
            core_tools._TOOLS_INITIALIZED = False

            barrier = threading.Barrier(2)
            results: list[list] = [[], []]

            def worker(idx: int) -> None:
                barrier.wait()
                with patch.object(core_tools, "_load_profile_tools", counting_load):
                    specs = core_tools.get_tool_specs()
                results[idx] = specs

            t1 = threading.Thread(target=worker, args=(0,))
            t2 = threading.Thread(target=worker, args=(1,))
            t1.start()
            t2.start()
            t1.join(timeout=5)
            t2.join(timeout=5)

            assert call_count == 1, f"Expected 1 init call, got {call_count}"
            # Both threads should get the same specs
            assert results[0] == results[1]
        finally:
            core_tools.ALL_TOOLS.clear()
            core_tools.ALL_TOOLS.update(old_tools)
            core_tools.ALL_TOOL_SPECS.clear()
            core_tools.ALL_TOOL_SPECS.extend(old_specs)
            core_tools._TOOLS_INITIALIZED = old_init

    def test_initialized_guard_prevents_double_init(self) -> None:
        """_TOOLS_INITIALIZED=True prevents re-initialization even under contention."""
        old_init = core_tools._TOOLS_INITIALIZED
        try:
            core_tools._TOOLS_INITIALIZED = True
            with patch.object(core_tools, "_load_profile_tools") as mock_load:
                core_tools._initialize_tools()
                mock_load.assert_not_called()
        finally:
            core_tools._TOOLS_INITIALIZED = old_init


class TestModuleLoadCleanup:
    """PY-LOW-3: sys.modules cleanup on exec_module failure."""

    def test_failed_exec_cleans_sys_modules(self, tmp_path: Path) -> None:
        """Module with syntax error is NOT left in sys.modules after failure."""
        bad_file = tmp_path / "bad_module.py"
        bad_file.write_text("def f(:\n    pass\n")  # syntax error

        module_name = "_test_bad_module_cleanup"
        assert module_name not in sys.modules

        with pytest.raises(SyntaxError):
            core_tools._load_module_from_file(module_name, bad_file)

        assert module_name not in sys.modules, "Broken module should not persist in sys.modules"

    def test_successful_exec_stays_in_sys_modules(self, tmp_path: Path) -> None:
        """Module that succeeds remains in sys.modules."""
        good_file = tmp_path / "good_module.py"
        good_file.write_text("X = 42\n")

        module_name = "_test_good_module_cleanup"
        try:
            core_tools._load_module_from_file(module_name, good_file)
            assert module_name in sys.modules
            assert sys.modules[module_name].X == 42  # type: ignore[attr-defined]
        finally:
            sys.modules.pop(module_name, None)
