"""Idle scanning policy for mmWave-based wellness probes."""

from __future__ import annotations
import logging


logger = logging.getLogger(__name__)


class IdlePolicy:
    """Pure state machine managing idle mmWave scanning decisions.

    Tracks consecutive misses, sweep cooldowns, and post-focus quiet windows
    to decide when and how to probe.
    """

    def __init__(  # noqa: D107
        self,
        *,
        interval_s: float = 15.0,
        probe_interval_s: float = 40.0,
        probe_duration_s: float = 5.0,
        misses_before_sweep: int = 3,
        sweep_cooldown_s: float = 150.0,
        post_focus_quiet_s: float = 45.0,
    ) -> None:
        self.interval_s = interval_s
        self.probe_interval_s = probe_interval_s
        self.probe_duration_s = probe_duration_s
        self.misses_before_sweep = misses_before_sweep
        self.sweep_cooldown_s = sweep_cooldown_s
        self.post_focus_quiet_s = post_focus_quiet_s

        # Mutable state
        self.consecutive_misses: int = 0
        self.last_sweep_time: float | None = None
        self.last_focus_time: float | None = None

    def sweep_allowed(self, now: float) -> bool:
        """Return True if a sweep is allowed given miss count and cooldown."""
        if self.consecutive_misses < self.misses_before_sweep:
            return False
        if self.last_sweep_time is None:
            return True
        return (now - self.last_sweep_time) >= self.sweep_cooldown_s

    def should_trigger(self, idle_duration: float, is_moving: bool, now: float) -> bool:
        """Return True if an idle probe should fire now."""
        if idle_duration <= self.probe_interval_s:
            return False
        if is_moving:
            return False
        if self.last_focus_time is not None:
            since_focus = now - self.last_focus_time
            if since_focus < self.post_focus_quiet_s:
                logger.debug(
                    "Idle probe suppressed: post-focus quiet (%.1fs remaining)",
                    self.post_focus_quiet_s - since_focus,
                )
                return False
        return True

    def record_target_found(self, now: float) -> None:
        """Record that mmWave detected a target."""
        self.consecutive_misses = 0
        self.last_focus_time = now
        logger.info("Idle mmWave detected target; miss counter reset.")

    def record_no_target(self, sweep_was_used: bool) -> None:
        """Record that mmWave found no target."""
        if sweep_was_used:
            self.consecutive_misses = 0
            logger.info("Idle mmWave sweep found no target; miss counter reset.")
        else:
            self.consecutive_misses += 1
            logger.info(
                "Idle mmWave no target (miss %d/%d before sweep).",
                self.consecutive_misses,
                self.misses_before_sweep,
            )

    def record_inconclusive(self) -> None:
        """Record an inconclusive mmWave result (no state change)."""
        logger.info("Idle mmWave inconclusive; miss counter unchanged at %d.", self.consecutive_misses)

    def record_sweep_used(self, now: float) -> None:
        """Record that a sweep was performed."""
        self.last_sweep_time = now

    def build_strategy_message(self, sweep_allowed: bool) -> str:
        """Build the strategy fragment for the idle signal instructions."""
        if sweep_allowed:
            return "run one slow scan sweep if no target is found"
        return "do a passive check only and stay still if no target is found"
