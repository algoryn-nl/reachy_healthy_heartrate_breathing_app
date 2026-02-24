"""Tests for IdlePolicy state machine."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations

from healthy_heartrate_breathing.idle_policy import IdlePolicy


def _policy(**overrides: object) -> IdlePolicy:
    defaults = {
        "interval_s": 15.0,
        "probe_interval_s": 40.0,
        "probe_duration_s": 5.0,
        "misses_before_sweep": 3,
        "sweep_cooldown_s": 150.0,
        "post_focus_quiet_s": 45.0,
    }
    defaults.update(overrides)
    return IdlePolicy(**defaults)


class TestSweepAllowed:
    def test_not_enough_misses(self) -> None:
        p = _policy(misses_before_sweep=3)
        p.consecutive_misses = 2
        assert p.sweep_allowed(now=100.0) is False

    def test_enough_misses_no_prior_sweep(self) -> None:
        p = _policy(misses_before_sweep=3)
        p.consecutive_misses = 3
        assert p.sweep_allowed(now=100.0) is True

    def test_enough_misses_within_cooldown(self) -> None:
        p = _policy(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=100.0) is False  # 50s < 150s cooldown

    def test_enough_misses_past_cooldown(self) -> None:
        p = _policy(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=250.0) is True  # 200s > 150s cooldown


class TestRecordResults:
    def test_record_target_found_resets_misses(self) -> None:
        p = _policy()
        p.consecutive_misses = 5
        p.record_target_found(now=100.0)
        assert p.consecutive_misses == 0
        assert p.last_focus_time == 100.0

    def test_record_no_target_increments_misses(self) -> None:
        p = _policy()
        p.consecutive_misses = 1
        p.record_no_target(sweep_was_used=False)
        assert p.consecutive_misses == 2

    def test_record_no_target_after_sweep_resets(self) -> None:
        p = _policy()
        p.consecutive_misses = 5
        p.record_no_target(sweep_was_used=True)
        assert p.consecutive_misses == 0

    def test_record_inconclusive_no_change(self) -> None:
        p = _policy()
        p.consecutive_misses = 3
        p.record_inconclusive()
        assert p.consecutive_misses == 3


class TestShouldTrigger:
    def test_not_enough_idle_time(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=10.0, is_moving=False, now=100.0) is False

    def test_enough_idle_time_not_moving(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True

    def test_enough_idle_time_but_moving(self) -> None:
        p = _policy(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=True, now=100.0) is False

    def test_suppressed_during_post_focus_quiet(self) -> None:
        p = _policy(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 80.0
        # now=100, so only 20s since focus < 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is False

    def test_allowed_after_post_focus_quiet(self) -> None:
        p = _policy(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 50.0
        # now=100, so 50s since focus > 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True


class TestErrorTracking:
    def test_record_error_increments_counter(self) -> None:
        p = _policy()
        assert p.consecutive_errors == 0
        p.record_error(now=100.0)
        assert p.consecutive_errors == 1
        assert p.last_error_time == 100.0

    def test_record_error_marks_suppressed_after_threshold(self) -> None:
        p = _policy(errors_before_suppression=3)
        for t in range(3):
            p.record_error(now=100.0 + t)
        assert p.consecutive_errors == 3
        assert p.sensor_suppressed is True

    def test_not_suppressed_below_threshold(self) -> None:
        p = _policy(errors_before_suppression=3)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is False

    def test_should_trigger_suppressed_during_backoff(self) -> None:
        p = _policy(probe_interval_s=40.0, errors_before_suppression=2, error_backoff_s=60.0)
        p.record_error(now=90.0)
        p.record_error(now=95.0)
        # now=100, last_error=95, backoff=60 -> 5s < 60s -> suppressed
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is False

    def test_should_trigger_allowed_after_backoff_expires(self) -> None:
        p = _policy(probe_interval_s=40.0, errors_before_suppression=2, error_backoff_s=60.0)
        p.record_error(now=10.0)
        p.record_error(now=15.0)
        # now=100, last_error=15, backoff=60 -> 85s > 60s -> retry allowed
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True

    def test_record_target_found_resets_errors(self) -> None:
        p = _policy(errors_before_suppression=2)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is True
        p.record_target_found(now=200.0)
        assert p.consecutive_errors == 0
        assert p.last_error_time is None
        assert p.sensor_suppressed is False

    def test_record_no_target_resets_errors(self) -> None:
        p = _policy(errors_before_suppression=2)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is True
        p.record_no_target(sweep_was_used=False)
        assert p.consecutive_errors == 0
        assert p.last_error_time is None
        assert p.sensor_suppressed is False


class TestBuildStrategyMessage:
    def test_returns_nonempty_string(self) -> None:
        p = _policy(probe_duration_s=5.0)
        msg = p.build_strategy_message(sweep_allowed=False)
        assert isinstance(msg, str) and len(msg) > 0

    def test_sweep_allowed_message(self) -> None:
        p = _policy()
        msg = p.build_strategy_message(sweep_allowed=True)
        assert "sweep" in msg.lower()

    def test_no_sweep_message(self) -> None:
        p = _policy()
        msg = p.build_strategy_message(sweep_allowed=False)
        assert "passive" in msg.lower() or "still" in msg.lower()
