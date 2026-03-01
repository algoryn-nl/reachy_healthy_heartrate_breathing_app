"""Idle scanning policy for mmWave-based wellness probes.

State Machine
=============

IdlePolicy is an implicit state machine — three mutable counters and
timestamps combine with timing parameters to gate idle-probe decisions.
No explicit state enum exists; the "current state" is the combination of
``consecutive_misses``, ``last_sweep_time``, and ``last_focus_time``.

States (implicit)
-----------------

::

    WAITING
        idle_duration <= probe_interval_s, or robot is moving.
        Nothing happens; normal conversation continues.

    POST_FOCUS_QUIET
        last_focus_time is set AND now - last_focus_time < post_focus_quiet_s.
        All probing suppressed so the robot can interact with a detected person
        without interruption.

    PASSIVE_PROBE
        All trigger gates pass AND consecutive_misses < misses_before_sweep.
        A passive mmWave probe fires (mode=locate_and_measure, no head sweep).

    SWEEP_ELIGIBLE
        All trigger gates pass AND consecutive_misses >= misses_before_sweep
        AND (no prior sweep OR sweep cooldown expired).
        An active probe fires with sweep_if_unseen=True (head rotates L/C/R).

    SWEEP_ON_COOLDOWN
        consecutive_misses >= misses_before_sweep BUT a sweep was performed
        recently (now - last_sweep_time < sweep_cooldown_s).
        Falls back to a passive probe until cooldown expires.

Transition diagram
------------------

::

    ┌──────────────────────────────────────────────────────────────┐
    │                         WAITING                              │
    │  idle_duration <= probe_interval_s  OR  robot is moving      │
    └──────────┬───────────────────────────────────────────────────┘
               │ idle_duration > probe_interval_s
               │ AND not is_moving
               ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                    should_trigger() gate                     │
    │  post-focus quiet active?  ─── Yes ──►  suppress (WAITING)  │
    └──────────┬───────────────────────────────────────────────────┘
               │ No (all gates pass)
               ▼
    ┌──────────────────────────────────────────────────────────────┐
    │                    sweep_allowed() gate                      │
    │  misses >= N AND cooldown expired?                           │
    └───┬──────────────────────────────────────┬───────────────────┘
        │ No                                   │ Yes
        ▼                                      ▼
    PASSIVE PROBE                         ACTIVE SWEEP
    mode=locate_and_measure               mode=locate_and_measure
    sweep_if_unseen=False                 sweep_if_unseen=True
        │                                      │
        └──────────────┬───────────────────────┘
                       ▼
              ┌─── result? ───────────────────┐
              │               │               │
         has_target      no_target       error (disconnect)
              │               │               │
              ▼               ▼               ▼
    record_target_found  record_no_target   record_error
    · misses = 0         (passive):         · errors += 1
    · errors = 0         · misses += 1      · last_error_time = now
    · last_focus_time    · errors = 0       IF errors >= threshold:
      = now              (after sweep):       suppress probes for
                         · misses = 0         error_backoff_s seconds
                         record_sweep_used:
                         · last_sweep_time = now

              inconclusive
                  │
                  ▼
           record_inconclusive
           · no state change

Trigger gates (should_trigger)
------------------------------

All must be True for a probe to fire:

1. ``idle_duration > probe_interval_s``  (enough idle time)
2. ``not is_moving``  (robot stationary)
3. ``last_focus_time is None``  OR  ``now - last_focus_time >= post_focus_quiet_s``

Escalation gates (sweep_allowed)
--------------------------------

Both must be True for a sweep:

1. ``consecutive_misses >= misses_before_sweep``
2. ``last_sweep_time is None``  OR  ``now - last_sweep_time >= sweep_cooldown_s``

Parameters
----------
All configurable via environment variables (read at handler init):

=====================================  =======  =============================================  =========
Parameter                              Default  Env Variable                                   Constraint
=====================================  =======  =============================================  =========
interval_s                              15.0    HEALTHY_MM_WAVE_IDLE_DEFAULT_INTERVAL_S         >= 1.0
probe_interval_s                        40.0    HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S            >= 1.0
probe_duration_s                         5.0    HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S            >= 0.5
misses_before_sweep                      3      HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP              >= 1
sweep_cooldown_s                       150.0    HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S                 >= 1.0
post_focus_quiet_s                      45.0    HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S               >= 0.0
errors_before_suppression                3      HEALTHY_MM_WAVE_ERRORS_BEFORE_SUPPRESSION        >= 1
error_backoff_s                        120.0    HEALTHY_MM_WAVE_ERROR_BACKOFF_S                  >= 1.0
multi_target_interval_multiplier        2.0    HEALTHY_MM_WAVE_MULTI_TARGET_INTERVAL_MULTIPLIER >= 1.0
=====================================  =======  =============================================  =========

Integration
-----------

- **emit()** in ``openai_realtime.py`` checks ``should_trigger()`` each audio frame.
- **send_idle_signal()** queries ``sweep_allowed()`` and ``build_strategy_message()``
  to instruct the LLM which mmWave call to make.
- **ToolDispatcher._run_tool()** overrides mmWave args for idle calls, then calls
  ``record_target_found()``, ``record_no_target()``, or ``record_inconclusive()``
  based on the result.

"""

from __future__ import annotations
import logging


logger = logging.getLogger(__name__)


class IdlePolicy:
    """Pure state machine managing idle mmWave scanning decisions.

    Tracks consecutive misses, sweep cooldowns, post-focus quiet windows,
    and consecutive errors to decide when and how to probe.

    When the sensor reports errors (e.g. USB disconnect), the policy tracks
    ``consecutive_errors`` and suppresses probing after
    ``errors_before_suppression`` consecutive failures. Suppression lasts
    ``error_backoff_s`` seconds, after which a single retry probe is allowed.
    A successful result resets the error counter.
    """

    # Default thresholds for error-based suppression
    DEFAULT_ERRORS_BEFORE_SUPPRESSION: int = 3
    DEFAULT_ERROR_BACKOFF_S: float = 120.0
    DEFAULT_MULTI_TARGET_INTERVAL_MULTIPLIER: float = 2.0

    def __init__(  # noqa: D107
        self,
        *,
        interval_s: float = 15.0,
        probe_interval_s: float = 40.0,
        probe_duration_s: float = 5.0,
        misses_before_sweep: int = 3,
        sweep_cooldown_s: float = 150.0,
        post_focus_quiet_s: float = 45.0,
        errors_before_suppression: int = DEFAULT_ERRORS_BEFORE_SUPPRESSION,
        error_backoff_s: float = DEFAULT_ERROR_BACKOFF_S,
        multi_target_interval_multiplier: float = DEFAULT_MULTI_TARGET_INTERVAL_MULTIPLIER,
    ) -> None:
        if interval_s < 1.0:
            raise ValueError(f"interval_s must be >= 1.0, got {interval_s}")
        if probe_interval_s < 1.0:
            raise ValueError(f"probe_interval_s must be >= 1.0, got {probe_interval_s}")
        if probe_duration_s < 0.5:
            raise ValueError(f"probe_duration_s must be >= 0.5, got {probe_duration_s}")
        if misses_before_sweep < 1:
            raise ValueError(f"misses_before_sweep must be >= 1, got {misses_before_sweep}")
        if sweep_cooldown_s < 0.0:
            raise ValueError(f"sweep_cooldown_s must be >= 0.0, got {sweep_cooldown_s}")
        if post_focus_quiet_s < 0.0:
            raise ValueError(f"post_focus_quiet_s must be >= 0.0, got {post_focus_quiet_s}")
        if errors_before_suppression < 1:
            raise ValueError(f"errors_before_suppression must be >= 1, got {errors_before_suppression}")
        if error_backoff_s < 1.0:
            raise ValueError(f"error_backoff_s must be >= 1.0, got {error_backoff_s}")
        if multi_target_interval_multiplier < 1.0:
            raise ValueError(
                f"multi_target_interval_multiplier must be >= 1.0, got {multi_target_interval_multiplier}"
            )
        self.interval_s = interval_s
        self.probe_interval_s = probe_interval_s
        self.probe_duration_s = probe_duration_s
        self.misses_before_sweep = misses_before_sweep
        self.sweep_cooldown_s = sweep_cooldown_s
        self.post_focus_quiet_s = post_focus_quiet_s
        self.errors_before_suppression = errors_before_suppression
        self.error_backoff_s = error_backoff_s
        self.multi_target_interval_multiplier = multi_target_interval_multiplier

        # Mutable state
        self.consecutive_misses: int = 0
        self.last_sweep_time: float | None = None
        self.last_focus_time: float | None = None
        self.consecutive_errors: int = 0
        self.last_error_time: float | None = None
        self.last_multi_target_time: float | None = None

    def sweep_allowed(self, now: float) -> bool:
        """Return True if a sweep is allowed given miss count and cooldown."""
        if self.consecutive_misses < self.misses_before_sweep:
            return False
        if self.last_sweep_time is None:
            return True
        return (now - self.last_sweep_time) >= self.sweep_cooldown_s

    @property
    def sensor_suppressed(self) -> bool:
        """Return True when probing is suppressed due to repeated errors."""
        return self.consecutive_errors >= self.errors_before_suppression

    @property
    def suggest_scan_only(self) -> bool:
        """Return True when the last result was multi-target (measure would fail)."""
        return self.last_multi_target_time is not None

    def should_trigger(self, idle_duration: float, is_moving: bool, now: float) -> bool:
        """Return True if an idle probe should fire now."""
        effective_interval = self.probe_interval_s
        if self.last_multi_target_time is not None:
            effective_interval *= self.multi_target_interval_multiplier
        if idle_duration <= effective_interval:
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
        # After repeated sensor errors, back off and only allow occasional retries
        if self.sensor_suppressed:
            if self.last_error_time is not None and (now - self.last_error_time) < self.error_backoff_s:
                logger.debug(
                    "Idle probe suppressed: sensor error backoff (errors=%d, %.1fs remaining)",
                    self.consecutive_errors,
                    self.error_backoff_s - (now - self.last_error_time),
                )
                return False
            # Backoff expired — allow one retry probe
            logger.info("Idle probe: error backoff expired, allowing retry probe (errors=%d)", self.consecutive_errors)
        return True

    def record_target_found(self, now: float) -> None:
        """Record that mmWave detected a target."""
        self.consecutive_misses = 0
        self.last_focus_time = now
        self.last_multi_target_time = None  # single target clears multi-target
        if self.consecutive_errors > 0:
            logger.info("Idle mmWave sensor recovered after %d consecutive errors.", self.consecutive_errors)
        self.consecutive_errors = 0
        self.last_error_time = None
        logger.info("Idle mmWave detected target; miss counter reset.")

    def record_multi_target(self, now: float) -> None:
        """Record that mmWave detected multiple targets.

        Resets miss and error counters (the sensor is working), but does NOT
        set ``last_focus_time`` — multi-target is not a usable focus event.
        """
        self.consecutive_misses = 0
        self.last_multi_target_time = now
        if self.consecutive_errors > 0:
            logger.info("Idle mmWave sensor recovered after %d consecutive errors.", self.consecutive_errors)
        self.consecutive_errors = 0
        self.last_error_time = None
        logger.info("Idle mmWave detected multiple targets; backing off probing.")

    def record_no_target(self, sweep_was_used: bool) -> None:
        """Record that mmWave found no target."""
        if self.consecutive_errors > 0:
            logger.info("Idle mmWave sensor recovered after %d consecutive errors.", self.consecutive_errors)
        self.consecutive_errors = 0
        self.last_error_time = None
        self.last_multi_target_time = None
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

    def record_error(self, now: float) -> None:
        """Record an mmWave error (serial disconnect, timeout, etc.).

        Increments the consecutive error counter and records the timestamp.
        After ``errors_before_suppression`` consecutive errors, idle probing
        is suppressed for ``error_backoff_s`` seconds before retrying.
        """
        self.consecutive_errors += 1
        self.last_error_time = now
        if self.consecutive_errors >= self.errors_before_suppression:
            logger.warning(
                "Idle mmWave sensor appears disconnected (%d consecutive errors); suppressing probes for %.0fs.",
                self.consecutive_errors,
                self.error_backoff_s,
            )
        else:
            logger.warning(
                "Idle mmWave error (%d/%d before suppression).",
                self.consecutive_errors,
                self.errors_before_suppression,
            )

    def record_sweep_used(self, now: float) -> None:
        """Record that a sweep was performed."""
        self.last_sweep_time = now

    def build_strategy_message(self, sweep_allowed: bool) -> str:
        """Build the strategy fragment for the idle signal instructions."""
        if sweep_allowed:
            return "run one slow scan sweep if no target is found"
        return "do a passive check only and stay still if no target is found"
