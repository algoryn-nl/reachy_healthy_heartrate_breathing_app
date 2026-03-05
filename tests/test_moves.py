"""Tests for MovementManager thread lifecycle and multi-threaded safety."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations

import time
import threading
from typing import Callable
from unittest.mock import MagicMock
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from reachy_mini.utils import create_head_pose
from reachy_mini.motion.move import Move
from healthy_heartrate_breathing.moves import MovementManager


@pytest.fixture
def manager_factory() -> Callable[[], MovementManager]:
    """Factory fixture: creates a MovementManager with a mocked robot (no hardware)."""
    created: list[MovementManager] = []

    def _create() -> MovementManager:
        robot = MagicMock()
        robot.set_target = MagicMock()
        mgr = MovementManager(robot)
        created.append(mgr)
        return mgr

    yield _create

    # Teardown: ensure all managers are stopped
    for mgr in created:
        mgr.stop()


class TestStartStop:
    """Thread lifecycle tests for MovementManager.start() / stop()."""

    def test_start_creates_alive_thread(self, manager_factory: Callable[[], MovementManager]) -> None:
        mgr = manager_factory()
        mgr.start()
        assert mgr._thread is not None
        assert mgr._thread.is_alive()

    def test_stop_when_not_started_is_noop(self, manager_factory: Callable[[], MovementManager]) -> None:
        mgr = manager_factory()
        assert mgr._thread is None
        mgr.stop()  # should not raise
        assert mgr._thread is None

    def test_start_twice_does_not_leak_thread(self, manager_factory: Callable[[], MovementManager]) -> None:
        mgr = manager_factory()
        mgr.start()
        first_thread = mgr._thread
        mgr.start()  # second call should be ignored
        assert mgr._thread is first_thread

    def test_start_after_stop_creates_new_thread(self, manager_factory: Callable[[], MovementManager]) -> None:
        mgr = manager_factory()
        mgr.start()
        first_thread = mgr._thread
        mgr.stop()
        assert mgr._thread is None

        mgr.start()
        assert mgr._thread is not None
        assert mgr._thread is not first_thread
        assert mgr._thread.is_alive()

    def test_stop_joins_thread(self, manager_factory: Callable[[], MovementManager]) -> None:
        mgr = manager_factory()
        mgr.start()
        assert mgr._thread is not None
        assert mgr._thread.is_alive()
        mgr.stop()
        assert mgr._thread is None

    def test_concurrent_starts_create_exactly_one_thread(
        self, manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Two threads calling start() simultaneously must produce only one worker."""
        mgr = manager_factory()
        barrier = threading.Barrier(2, timeout=5)

        def guarded_start() -> None:
            barrier.wait()
            mgr.start()

        with ThreadPoolExecutor(max_workers=2) as pool:
            f1 = pool.submit(guarded_start)
            f2 = pool.submit(guarded_start)
            f1.result(timeout=5)
            f2.result(timeout=5)

        # Exactly one thread should exist
        assert mgr._thread is not None
        assert mgr._thread.is_alive()

    def test_concurrent_start_and_stop_no_crash(self, manager_factory: Callable[[], MovementManager]) -> None:
        """Interleaved start/stop from different threads must not crash or deadlock."""
        mgr = manager_factory()
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


# ---------------------------------------------------------------------------
# Helpers for multi-threaded tests
# ---------------------------------------------------------------------------

class _DummyMove(Move):  # type: ignore[misc]
    """A trivial finite move for testing."""

    def __init__(self, duration: float = 0.1) -> None:
        self._duration = duration

    @property
    def duration(self) -> float:
        return self._duration

    def evaluate(self, t: float) -> tuple:
        head = create_head_pose(0, 0, 0, 0, 0, 0, degrees=True)
        antennas = np.array([0.0, 0.0])
        return (head, antennas, 0.0)


@pytest.fixture
def running_manager_factory(
    manager_factory: Callable[[], MovementManager],
) -> Callable[[], MovementManager]:
    """Factory fixture: creates and starts a MovementManager."""

    def _create() -> MovementManager:
        mgr = manager_factory()
        mgr.start()
        # Let the control loop spin up
        time.sleep(0.02)
        return mgr

    return _create


# ---------------------------------------------------------------------------
# Multi-threaded stress tests
# ---------------------------------------------------------------------------


class TestConcurrentQueueMove:
    """Concurrent queue_move() from multiple threads."""

    def test_concurrent_queue_move_all_enqueued(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Moves queued from N threads must all be accepted without error."""
        mgr = running_manager_factory()
        n_threads = 8
        moves_per_thread = 20
        barrier = threading.Barrier(n_threads, timeout=5)
        errors: list[Exception] = []

        def enqueue_moves() -> None:
            try:
                barrier.wait()
                for _ in range(moves_per_thread):
                    mgr.queue_move(_DummyMove(duration=0.01))
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=enqueue_moves) for _ in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

        # Give the manager time to drain the moves
        time.sleep(0.5)

    def test_concurrent_queue_move_no_data_corruption(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """get_status() called during concurrent enqueuing must not raise."""
        mgr = running_manager_factory()
        n_threads = 4
        barrier = threading.Barrier(n_threads + 1, timeout=5)
        errors: list[Exception] = []

        def enqueue_moves() -> None:
            try:
                barrier.wait()
                for _ in range(30):
                    mgr.queue_move(_DummyMove(duration=0.01))
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        def poll_status() -> None:
            try:
                barrier.wait()
                for _ in range(30):
                    mgr.get_status()
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=enqueue_moves) for _ in range(n_threads)]
        threads.append(threading.Thread(target=poll_status))
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"


class TestConcurrentClearAndQueue:
    """Concurrent clear_move_queue() + queue_move()."""

    def test_clear_and_queue_interleaved(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Interleaved clear + queue from two threads must not deadlock or crash."""
        mgr = running_manager_factory()
        barrier = threading.Barrier(2, timeout=5)
        errors: list[Exception] = []

        def do_queues() -> None:
            try:
                barrier.wait()
                for _ in range(50):
                    mgr.queue_move(_DummyMove(duration=0.05))
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        def do_clears() -> None:
            try:
                barrier.wait()
                for _ in range(50):
                    mgr.clear_move_queue()
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        t1 = threading.Thread(target=do_queues)
        t2 = threading.Thread(target=do_clears)
        t1.start()
        t2.start()
        t1.join(timeout=10)
        t2.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

    def test_clear_and_queue_with_get_status(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Status polling alongside clear+queue must not raise."""
        mgr = running_manager_factory()
        barrier = threading.Barrier(3, timeout=5)
        errors: list[Exception] = []

        def do_queues() -> None:
            try:
                barrier.wait()
                for _ in range(30):
                    mgr.queue_move(_DummyMove(duration=0.02))
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        def do_clears() -> None:
            try:
                barrier.wait()
                for _ in range(30):
                    mgr.clear_move_queue()
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        def do_status() -> None:
            try:
                barrier.wait()
                for _ in range(30):
                    status = mgr.get_status()
                    assert "queue_size" in status
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=do_queues),
            threading.Thread(target=do_clears),
            threading.Thread(target=do_status),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"


class TestConcurrentSecondaryOffsets:
    """Secondary offset updates from multiple threads."""

    def test_concurrent_speech_offsets(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Multiple threads setting speech offsets must not corrupt state."""
        mgr = running_manager_factory()
        n_threads = 6
        barrier = threading.Barrier(n_threads, timeout=5)
        errors: list[Exception] = []

        def set_offsets(thread_id: int) -> None:
            try:
                barrier.wait()
                for i in range(50):
                    val = float(thread_id * 100 + i) * 0.001
                    mgr.set_speech_offsets((val, val, 0.0, 0.0, val, 0.0))
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=set_offsets, args=(tid,)) for tid in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

        # Verify the final offsets are a valid 6-tuple (from any thread)
        with mgr._speech_offsets_lock:
            final = mgr._pending_speech_offsets
        assert len(final) == 6

    def test_concurrent_face_tracking_offsets(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Multiple threads setting face tracking offsets via the lock must not crash."""
        mgr = running_manager_factory()
        n_threads = 4
        barrier = threading.Barrier(n_threads, timeout=5)
        errors: list[Exception] = []

        def set_face(thread_id: int) -> None:
            try:
                barrier.wait()
                for i in range(50):
                    val = float(thread_id * 100 + i) * 0.001
                    with mgr._face_offsets_lock:
                        mgr._pending_face_offsets = (0.0, val, 0.0, 0.0, val, 0.0)
                        mgr._face_offsets_dirty = True
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=set_face, args=(tid,)) for tid in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

    def test_concurrent_speech_and_face_offsets(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Speech and face offsets set simultaneously must not interfere."""
        mgr = running_manager_factory()
        barrier = threading.Barrier(2, timeout=5)
        errors: list[Exception] = []

        def set_speech() -> None:
            try:
                barrier.wait()
                for i in range(80):
                    val = float(i) * 0.001
                    mgr.set_speech_offsets((val, 0.0, 0.0, 0.0, 0.0, val))
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        def set_face() -> None:
            try:
                barrier.wait()
                for i in range(80):
                    val = float(i) * 0.001
                    with mgr._face_offsets_lock:
                        mgr._pending_face_offsets = (0.0, val, 0.0, 0.0, val, 0.0)
                        mgr._face_offsets_dirty = True
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)

        t1 = threading.Thread(target=set_speech)
        t2 = threading.Thread(target=set_face)
        t1.start()
        t2.start()
        t1.join(timeout=10)
        t2.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"


class TestRapidStartStopStress:
    """Stress variant of lifecycle tests with higher iteration count."""

    def test_rapid_start_stop_cycles(self, manager_factory: Callable[[], MovementManager]) -> None:
        """Rapid start/stop cycles from a single thread must not leak or deadlock."""
        mgr = manager_factory()
        errors: list[Exception] = []

        try:
            for _ in range(20):
                mgr.start()
                time.sleep(0.005)
                mgr.stop()
        except Exception as e:
            errors.append(e)

        assert not errors, f"Unexpected errors: {errors}"
        assert mgr._thread is None

    def test_concurrent_rapid_start_stop(self, manager_factory: Callable[[], MovementManager]) -> None:
        """Two threads rapidly toggling start/stop must not deadlock."""
        mgr = manager_factory()
        barrier = threading.Barrier(2, timeout=5)
        errors: list[Exception] = []

        def toggle(do_start_first: bool) -> None:
            try:
                barrier.wait()
                for _ in range(15):
                    if do_start_first:
                        mgr.start()
                        time.sleep(0.003)
                        mgr.stop()
                    else:
                        mgr.stop()
                        time.sleep(0.003)
                        mgr.start()
            except Exception as e:
                errors.append(e)

        t1 = threading.Thread(target=toggle, args=(True,))
        t2 = threading.Thread(target=toggle, args=(False,))
        t1.start()
        t2.start()
        t1.join(timeout=15)
        t2.join(timeout=15)

        assert not t1.is_alive(), "Thread t1 appears deadlocked"
        assert not t2.is_alive(), "Thread t2 appears deadlocked"
        assert not errors, f"Unexpected errors: {errors}"


class TestConcurrentListening:
    """Concurrent set_listening and queue_move interactions."""

    def test_concurrent_listening_toggles(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Multiple threads toggling listening must not crash."""
        mgr = running_manager_factory()
        n_threads = 4
        barrier = threading.Barrier(n_threads, timeout=5)
        errors: list[Exception] = []

        def toggle_listening(thread_id: int) -> None:
            try:
                barrier.wait()
                for i in range(30):
                    mgr.set_listening(i % 2 == 0)
                    time.sleep(0.005)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=toggle_listening, args=(tid,)) for tid in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"

    def test_listening_and_queue_move_concurrent(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Listening toggles + queue_move from separate threads must not deadlock."""
        mgr = running_manager_factory()
        barrier = threading.Barrier(2, timeout=5)
        errors: list[Exception] = []

        def toggle_listening() -> None:
            try:
                barrier.wait()
                for i in range(40):
                    mgr.set_listening(i % 2 == 0)
                    time.sleep(0.003)
            except Exception as e:
                errors.append(e)

        def enqueue_moves() -> None:
            try:
                barrier.wait()
                for _ in range(40):
                    mgr.queue_move(_DummyMove(duration=0.02))
                    time.sleep(0.003)
            except Exception as e:
                errors.append(e)

        t1 = threading.Thread(target=toggle_listening)
        t2 = threading.Thread(target=enqueue_moves)
        t1.start()
        t2.start()
        t1.join(timeout=10)
        t2.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"


class TestIsIdleConcurrent:
    """is_idle() called from external threads during control loop."""

    def test_is_idle_concurrent_reads(
        self, running_manager_factory: Callable[[], MovementManager]
    ) -> None:
        """Multiple threads reading is_idle() must not crash or deadlock."""
        mgr = running_manager_factory()
        n_threads = 4
        barrier = threading.Barrier(n_threads, timeout=5)
        errors: list[Exception] = []
        results: list[bool] = []

        def check_idle() -> None:
            try:
                barrier.wait()
                for _ in range(50):
                    results.append(mgr.is_idle())
                    time.sleep(0.002)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=check_idle) for _ in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)

        assert not errors, f"Unexpected errors: {errors}"
        assert len(results) == n_threads * 50
