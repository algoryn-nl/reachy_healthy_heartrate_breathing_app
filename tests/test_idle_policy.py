"""Tests for IdlePolicy state machine."""
# ruff: noqa: D101, D102, D103

from __future__ import annotations
from typing import Any
from collections.abc import Callable

import pytest

from healthy_heartrate_breathing.idle_policy import IdlePolicy


@pytest.fixture
def policy_factory() -> Callable[..., IdlePolicy]:
    """Return a factory that builds an IdlePolicy with sane defaults."""

    def _make(**overrides: Any) -> IdlePolicy:
        defaults: dict[str, Any] = {
            "interval_s": 15.0,
            "probe_interval_s": 40.0,
            "probe_duration_s": 5.0,
            "misses_before_sweep": 3,
            "sweep_cooldown_s": 150.0,
            "post_focus_quiet_s": 45.0,
            "multi_target_interval_multiplier": 2.0,
        }
        defaults.update(overrides)
        return IdlePolicy(**defaults)

    return _make


class TestSweepAllowed:
    def test_not_enough_misses(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(misses_before_sweep=3)
        p.consecutive_misses = 2
        assert p.sweep_allowed(now=100.0) is False

    def test_enough_misses_no_prior_sweep(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(misses_before_sweep=3)
        p.consecutive_misses = 3
        assert p.sweep_allowed(now=100.0) is True

    def test_enough_misses_within_cooldown(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=100.0) is False  # 50s < 150s cooldown

    def test_enough_misses_past_cooldown(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(misses_before_sweep=3, sweep_cooldown_s=150.0)
        p.consecutive_misses = 3
        p.last_sweep_time = 50.0
        assert p.sweep_allowed(now=250.0) is True  # 200s > 150s cooldown


class TestRecordResults:
    def test_record_target_found_resets_misses(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.consecutive_misses = 5
        p.record_target_found(now=100.0)
        assert p.consecutive_misses == 0
        assert p.last_focus_time == 100.0

    def test_record_no_target_increments_misses(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.consecutive_misses = 1
        p.record_no_target(sweep_was_used=False)
        assert p.consecutive_misses == 2

    def test_record_no_target_after_sweep_resets(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.consecutive_misses = 5
        p.record_no_target(sweep_was_used=True)
        assert p.consecutive_misses == 0

    def test_record_inconclusive_no_change(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.consecutive_misses = 3
        p.record_inconclusive()
        assert p.consecutive_misses == 3


class TestShouldTrigger:
    def test_not_enough_idle_time(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=10.0, is_moving=False, now=100.0) is False

    def test_enough_idle_time_not_moving(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True

    def test_enough_idle_time_but_moving(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0)
        assert p.should_trigger(idle_duration=50.0, is_moving=True, now=100.0) is False

    def test_suppressed_during_post_focus_quiet(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 80.0
        # now=100, so only 20s since focus < 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is False

    def test_allowed_after_post_focus_quiet(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0, post_focus_quiet_s=45.0)
        p.last_focus_time = 50.0
        # now=100, so 50s since focus > 45s quiet window
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True


class TestErrorTracking:
    def test_record_error_increments_counter(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        assert p.consecutive_errors == 0
        p.record_error(now=100.0)
        assert p.consecutive_errors == 1
        assert p.last_error_time == 100.0

    def test_record_error_marks_suppressed_after_threshold(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(errors_before_suppression=3)
        for t in range(3):
            p.record_error(now=100.0 + t)
        assert p.consecutive_errors == 3
        assert p.sensor_suppressed is True

    def test_not_suppressed_below_threshold(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(errors_before_suppression=3)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is False

    def test_should_trigger_suppressed_during_backoff(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0, errors_before_suppression=2, error_backoff_s=60.0)
        p.record_error(now=90.0)
        p.record_error(now=95.0)
        # now=100, last_error=95, backoff=60 -> 5s < 60s -> suppressed
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is False

    def test_should_trigger_allowed_after_backoff_expires(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0, errors_before_suppression=2, error_backoff_s=60.0)
        p.record_error(now=10.0)
        p.record_error(now=15.0)
        # now=100, last_error=15, backoff=60 -> 85s > 60s -> retry allowed
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True

    def test_record_target_found_resets_errors(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(errors_before_suppression=2)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is True
        p.record_target_found(now=200.0)
        assert p.consecutive_errors == 0
        assert p.last_error_time is None
        assert p.sensor_suppressed is False

    def test_record_no_target_resets_errors(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(errors_before_suppression=2)
        p.record_error(now=100.0)
        p.record_error(now=101.0)
        assert p.sensor_suppressed is True
        p.record_no_target(sweep_was_used=False)
        assert p.consecutive_errors == 0
        assert p.last_error_time is None
        assert p.sensor_suppressed is False


class TestBuildStrategyMessage:
    def test_returns_nonempty_string(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_duration_s=5.0)
        msg = p.build_strategy_message(sweep_allowed=False)
        assert isinstance(msg, str) and len(msg) > 0

    def test_sweep_allowed_message(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        msg = p.build_strategy_message(sweep_allowed=True)
        assert "sweep" in msg.lower()

    def test_no_sweep_message(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        msg = p.build_strategy_message(sweep_allowed=False)
        assert "passive" in msg.lower() or "still" in msg.lower()


class TestMultiTarget:
    def test_record_multi_target_resets_misses(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.consecutive_misses = 5
        p.record_multi_target(now=100.0)
        assert p.consecutive_misses == 0
        assert p.last_focus_time is None  # multi-target does NOT set focus

    def test_record_multi_target_resets_errors(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(errors_before_suppression=2)
        p.record_error(now=90.0)
        p.record_error(now=95.0)
        assert p.sensor_suppressed is True
        p.record_multi_target(now=100.0)
        assert p.consecutive_errors == 0
        assert p.sensor_suppressed is False

    def test_record_multi_target_sets_timestamp(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        assert p.last_multi_target_time is None
        p.record_multi_target(now=100.0)
        assert p.last_multi_target_time == 100.0

    def test_multi_target_multiplies_probe_interval(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory(probe_interval_s=40.0, multi_target_interval_multiplier=2.0)
        # Without multi-target: triggers at idle_duration > 40
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=100.0) is True
        # Record multi-target
        p.record_multi_target(now=100.0)
        # Now needs idle_duration > 40 * 2.0 = 80
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=101.0) is False
        assert p.should_trigger(idle_duration=90.0, is_moving=False, now=101.0) is True

    def test_multi_target_decays_to_normal_after_single_target(
        self, policy_factory: Callable[..., IdlePolicy]
    ) -> None:
        p = policy_factory(probe_interval_s=40.0, multi_target_interval_multiplier=2.0)
        p.record_multi_target(now=100.0)
        # Multiplied interval active
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=101.0) is False
        # Single target clears multi-target
        p.record_target_found(now=200.0)
        # After post-focus quiet window expires, normal interval applies
        assert p.should_trigger(idle_duration=50.0, is_moving=False, now=300.0) is True

    def test_suggest_scan_only_true_when_multi_target(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True

    def test_suggest_scan_only_false_initially(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        assert p.suggest_scan_only is False

    def test_suggest_scan_only_false_after_single_target(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True
        p.record_target_found(now=200.0)
        assert p.suggest_scan_only is False

    def test_suggest_scan_only_false_after_no_target(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        p = policy_factory()
        p.record_multi_target(now=100.0)
        assert p.suggest_scan_only is True
        p.record_no_target(sweep_was_used=False)
        assert p.suggest_scan_only is False


class TestConstructorValidation:
    """PY-LOW-1: Constructor rejects invalid parameters with ValueError."""

    def test_interval_s_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="interval_s"):
            policy_factory(interval_s=0.5)

    def test_probe_interval_s_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="probe_interval_s"):
            policy_factory(probe_interval_s=0.0)

    def test_probe_duration_s_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="probe_duration_s"):
            policy_factory(probe_duration_s=0.1)

    def test_misses_before_sweep_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="misses_before_sweep"):
            policy_factory(misses_before_sweep=0)

    def test_sweep_cooldown_s_negative(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="sweep_cooldown_s"):
            policy_factory(sweep_cooldown_s=-1.0)

    def test_post_focus_quiet_s_negative(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="post_focus_quiet_s"):
            policy_factory(post_focus_quiet_s=-0.1)

    def test_errors_before_suppression_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="errors_before_suppression"):
            policy_factory(errors_before_suppression=0)

    def test_error_backoff_s_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="error_backoff_s"):
            policy_factory(error_backoff_s=0.5)

    def test_multi_target_interval_multiplier_too_low(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        with pytest.raises(ValueError, match="multi_target_interval_multiplier"):
            policy_factory(multi_target_interval_multiplier=0.5)

    def test_minimum_valid_values_succeed(self) -> None:
        """Edge case: all minimum allowed values should construct successfully."""
        p = IdlePolicy(
            interval_s=1.0,
            probe_interval_s=1.0,
            probe_duration_s=0.5,
            misses_before_sweep=1,
            sweep_cooldown_s=0.0,
            post_focus_quiet_s=0.0,
            errors_before_suppression=1,
            error_backoff_s=1.0,
            multi_target_interval_multiplier=1.0,
        )
        assert p.interval_s == 1.0

    def test_existing_helper_defaults_are_valid(self, policy_factory: Callable[..., IdlePolicy]) -> None:
        """The policy_factory() with all defaults constructs successfully."""
        p = policy_factory()
        assert p.probe_interval_s == 40.0
