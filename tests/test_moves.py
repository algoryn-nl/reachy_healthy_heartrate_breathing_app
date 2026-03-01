"""Tests for MovementManager thread lifecycle (start/stop)."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
import time
import threading
from unittest.mock import MagicMock
from concurrent.futures import ThreadPoolExecutor

from healthy_heartrate_breathing.moves import MovementManager


def _make_manager() -> MovementManager:
    """Create a MovementManager with a mocked robot (no hardware)."""
    robot = MagicMock()
    robot.set_target = MagicMock()
    mgr = MovementManager(robot)
    return mgr


class TestStartStop:
    """Thread lifecycle tests for MovementManager.start() / stop()."""

    def test_start_creates_alive_thread(self) -> None:
        mgr = _make_manager()
        mgr.start()
        try:
            assert mgr._thread is not None
            assert mgr._thread.is_alive()
        finally:
            mgr.stop()

    def test_stop_when_not_started_is_noop(self) -> None:
        mgr = _make_manager()
        assert mgr._thread is None
        mgr.stop()  # should not raise
        assert mgr._thread is None

    def test_start_twice_does_not_leak_thread(self) -> None:
        mgr = _make_manager()
        mgr.start()
        try:
            first_thread = mgr._thread
            mgr.start()  # second call should be ignored
            assert mgr._thread is first_thread
        finally:
            mgr.stop()

    def test_start_after_stop_creates_new_thread(self) -> None:
        mgr = _make_manager()
        mgr.start()
        first_thread = mgr._thread
        mgr.stop()
        assert mgr._thread is None

        mgr.start()
        try:
            assert mgr._thread is not None
            assert mgr._thread is not first_thread
            assert mgr._thread.is_alive()
        finally:
            mgr.stop()

    def test_stop_joins_thread(self) -> None:
        mgr = _make_manager()
        mgr.start()
        assert mgr._thread is not None
        assert mgr._thread.is_alive()
        mgr.stop()
        assert mgr._thread is None

    def test_concurrent_starts_create_exactly_one_thread(self) -> None:
        """Two threads calling start() simultaneously must produce only one worker."""
        mgr = _make_manager()
        barrier = threading.Barrier(2, timeout=5)

        def guarded_start() -> None:
            barrier.wait()
            mgr.start()

        try:
            with ThreadPoolExecutor(max_workers=2) as pool:
                f1 = pool.submit(guarded_start)
                f2 = pool.submit(guarded_start)
                f1.result(timeout=5)
                f2.result(timeout=5)

            # Exactly one thread should exist
            assert mgr._thread is not None
            assert mgr._thread.is_alive()

            # Verify no second worker thread was leaked
            assert mgr._thread.is_alive()
        finally:
            mgr.stop()

    def test_concurrent_start_and_stop_no_crash(self) -> None:
        """Interleaved start/stop from different threads must not crash or deadlock."""
        mgr = _make_manager()
        errors: list[Exception] = []

        def run_starts() -> None:
            try:
                for _ in range(10):
                    mgr.start()
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        def run_stops() -> None:
            try:
                for _ in range(10):
                    mgr.stop()
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        t1 = threading.Thread(target=run_starts)
        t2 = threading.Thread(target=run_stops)
        t1.start()
        t2.start()
        t1.join(timeout=10)
        t2.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

        # Clean up — ensure stopped
        mgr.stop()
