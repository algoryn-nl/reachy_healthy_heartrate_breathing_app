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
