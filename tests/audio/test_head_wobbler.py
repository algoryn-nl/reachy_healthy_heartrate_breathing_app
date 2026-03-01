"""Regression tests for the audio-driven head wobble behaviour."""

import math
import time
import base64
import threading
from typing import Any, List, Tuple
from collections.abc import Callable

import numpy as np

from healthy_heartrate_breathing.audio.head_wobbler import HeadWobbler


def _make_audio_chunk(duration_s: float = 0.3, frequency_hz: float = 220.0) -> str:
    """Generate a base64-encoded mono PCM16 sine wave."""
    sample_rate = 24000
    sample_count = int(sample_rate * duration_s)
    t = np.linspace(0, duration_s, sample_count, endpoint=False)
    wave = 0.6 * np.sin(2 * math.pi * frequency_hz * t)
    pcm = np.clip(wave * np.iinfo(np.int16).max, -32768, 32767).astype(np.int16)
    return base64.b64encode(pcm.tobytes()).decode("ascii")


def _wait_for(predicate: Callable[[], bool], timeout: float = 0.6) -> bool:
    """Poll `predicate` until true or timeout."""
    end_time = time.time() + timeout
    while time.time() < end_time:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _start_wobbler() -> Tuple[HeadWobbler, List[Tuple[float, Tuple[float, float, float, float, float, float]]]]:
    captured: List[Tuple[float, Tuple[float, float, float, float, float, float]]] = []

    def capture(offsets: Tuple[float, float, float, float, float, float]) -> None:
        captured.append((time.time(), offsets))

    wobbler = HeadWobbler(set_speech_offsets=capture)
    wobbler.start()
    return wobbler, captured


def test_reset_drops_pending_offsets() -> None:
    """Reset should stop wobble output derived from pre-reset audio."""
    wobbler, captured = _start_wobbler()
    try:
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert _wait_for(lambda: len(captured) > 0), "wobbler did not emit initial offsets"

        pre_reset_count = len(captured)
        wobbler.reset()
        time.sleep(0.3)
        assert len(captured) == pre_reset_count, "offsets continued after reset without new audio"
    finally:
        wobbler.stop()


def test_reset_allows_future_offsets() -> None:
    """After reset, fresh audio must still produce wobble offsets."""
    wobbler, captured = _start_wobbler()
    try:
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert _wait_for(lambda: len(captured) > 0), "wobbler did not emit initial offsets"

        wobbler.reset()
        pre_second_count = len(captured)

        wobbler.feed(_make_audio_chunk(duration_s=0.35, frequency_hz=440.0))
        assert _wait_for(lambda: len(captured) > pre_second_count), "no offsets after reset"
        assert wobbler._thread is not None and wobbler._thread.is_alive()
    finally:
        wobbler.stop()


def test_reset_during_inflight_chunk_keeps_worker(monkeypatch: Any) -> None:
    """Simulate reset during chunk processing to ensure the worker survives."""
    wobbler, captured = _start_wobbler()
    ready = threading.Event()
    release = threading.Event()

    original_feed = wobbler.sway.feed

    def blocking_feed(pcm, sr):  # type: ignore[no-untyped-def]
        ready.set()
        release.wait(timeout=2.0)
        return original_feed(pcm, sr)

    monkeypatch.setattr(wobbler.sway, "feed", blocking_feed)

    try:
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert ready.wait(timeout=1.0), "worker thread did not dequeue audio"

        wobbler.reset()
        release.set()

        # Allow the worker to finish processing the first chunk (which should be discarded)
        time.sleep(0.1)

        assert wobbler._thread is not None and wobbler._thread.is_alive(), "worker thread died after reset"

        pre_second = len(captured)
        wobbler.feed(_make_audio_chunk(duration_s=0.35, frequency_hz=440.0))
        assert _wait_for(lambda: len(captured) > pre_second), "no offsets emitted after in-flight reset"
        assert wobbler._thread.is_alive()
    finally:
        wobbler.stop()


# --- Thread-safety tests (PY-HIGH-4) ---


def test_generation_not_bumped_while_sway_feed_blocked(monkeypatch: Any) -> None:
    """Generation must not be bumped while sway.feed() holds _sway_lock.

    With the fix, reset() acquires _sway_lock before bumping generation,
    so generation stays stable until sway.feed() completes.  With the old
    code, reset() bumps generation outside _sway_lock — this test fails.
    """
    wobbler, captured = _start_wobbler()
    feed_started = threading.Event()
    allow_feed = threading.Event()
    original = wobbler.sway.feed

    def slow_feed(pcm: Any, sr: Any) -> Any:
        feed_started.set()
        allow_feed.wait(timeout=2.0)
        return original(pcm, sr)

    monkeypatch.setattr(wobbler.sway, "feed", slow_feed)

    try:
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert feed_started.wait(timeout=1.0), "worker didn't reach sway.feed"

        with wobbler._state_lock:
            gen_before = wobbler._generation

        # Start reset in background — should block on _sway_lock
        reset_done = threading.Event()

        def do_reset() -> None:
            wobbler.reset()
            reset_done.set()

        threading.Thread(target=do_reset, daemon=True).start()
        time.sleep(0.1)  # give reset thread time to reach _sway_lock

        # While sway.feed is blocked, generation must NOT have changed
        with wobbler._state_lock:
            gen_during = wobbler._generation
        assert gen_during == gen_before, f"generation bumped while sway.feed blocked: {gen_before} -> {gen_during}"

        # Release sway.feed -> reset completes
        allow_feed.set()
        assert reset_done.wait(timeout=2.0), "reset didn't complete"

        # NOW generation should be bumped
        with wobbler._state_lock:
            gen_after = wobbler._generation
        assert gen_after == gen_before + 1
    finally:
        wobbler.stop()


def test_concurrent_reset_feed_no_deadlock() -> None:
    """Rapid interleaving of feed() and reset() must not deadlock.

    Verifies the lock ordering (_sway_lock -> _state_lock) doesn't
    create circular dependencies under concurrent access.
    """
    wobbler, captured = _start_wobbler()
    stop = threading.Event()
    errors: List[str] = []

    def feeder() -> None:
        while not stop.is_set():
            try:
                wobbler.feed(_make_audio_chunk(duration_s=0.05))
            except Exception as e:
                errors.append(f"feed: {e}")
            time.sleep(0.005)

    def resetter() -> None:
        while not stop.is_set():
            try:
                wobbler.reset()
            except Exception as e:
                errors.append(f"reset: {e}")
            time.sleep(0.01)

    try:
        threads = [
            threading.Thread(target=feeder, daemon=True),
            threading.Thread(target=resetter, daemon=True),
        ]
        for t in threads:
            t.start()

        time.sleep(0.5)
        stop.set()

        for t in threads:
            t.join(timeout=2.0)
            assert not t.is_alive(), "thread hung — possible deadlock"

        assert not errors, f"concurrent errors: {errors}"
        assert wobbler._thread is not None and wobbler._thread.is_alive()
    finally:
        wobbler.stop()


def test_reset_during_sway_feed_leaves_clean_state(monkeypatch: Any) -> None:
    """After reset during in-flight sway.feed(), sway state must be clean."""
    wobbler, captured = _start_wobbler()
    feed_started = threading.Event()
    allow_feed = threading.Event()
    original = wobbler.sway.feed

    def slow_feed(pcm: Any, sr: Any) -> Any:
        feed_started.set()
        allow_feed.wait(timeout=2.0)
        return original(pcm, sr)

    monkeypatch.setattr(wobbler.sway, "feed", slow_feed)

    try:
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert feed_started.wait(timeout=1.0), "worker didn't reach sway.feed"

        reset_done = threading.Event()

        def do_reset() -> None:
            wobbler.reset()
            reset_done.set()

        threading.Thread(target=do_reset, daemon=True).start()
        time.sleep(0.05)
        allow_feed.set()
        assert reset_done.wait(timeout=2.0), "reset didn't complete"

        # After reset, all mutable state must be clean
        with wobbler._sway_lock:
            assert wobbler.sway.t == 0.0, f"sway.t contaminated: {wobbler.sway.t}"
            assert wobbler.sway.carry.size == 0
            assert len(wobbler.sway.samples) == 0
        with wobbler._state_lock:
            assert wobbler._base_ts is None
            assert wobbler._hops_done == 0
    finally:
        wobbler.stop()


def test_rapid_reset_cycles_preserve_worker() -> None:
    """Worker thread survives many rapid reset cycles and processes new audio."""
    wobbler, captured = _start_wobbler()
    try:
        for i in range(20):
            wobbler.feed(_make_audio_chunk(duration_s=0.05))
            wobbler.reset()
            assert wobbler._thread is not None and wobbler._thread.is_alive(), f"worker died at cycle {i}"

        pre = len(captured)
        wobbler.feed(_make_audio_chunk(duration_s=0.35))
        assert _wait_for(lambda: len(captured) > pre), "no offsets after rapid reset cycles"
    finally:
        wobbler.stop()
