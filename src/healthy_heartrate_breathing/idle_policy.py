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
              ┌─── result? ───┐
              │               │
         has_target      no_target         inconclusive / error
              │               │                    │
              ▼               ▼                    ▼
    record_target_found  record_no_target   record_inconclusive
    · misses = 0         (passive):         · no state change
    · last_focus_time    · misses += 1
      = now              (after sweep):
                         · misses = 0
                         record_sweep_used:
                         · last_sweep_time = now

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

==================================  =======  =======================================  =========
Parameter                           Default  Env Variable                             Constraint
==================================  =======  =======================================  =========
interval_s                          15.0     HEALTHY_MM_WAVE_IDLE_DEFAULT_INTERVAL_S  >= 1.0
probe_interval_s                    40.0     HEALTHY_MM_WAVE_IDLE_PROBE_INTERVAL_S    >= 1.0
probe_duration_s                     5.0     HEALTHY_MM_WAVE_IDLE_PROBE_DURATION_S    >= 0.5
misses_before_sweep                  3       HEALTHY_MM_WAVE_MISSES_BEFORE_SWEEP      >= 1
sweep_cooldown_s                   150.0     HEALTHY_MM_WAVE_SWEEP_COOLDOWN_S         >= 1.0
post_focus_quiet_s                  45.0     HEALTHY_MM_WAVE_POST_FOCUS_QUIET_S       >= 0.0
==================================  =======  =======================================  =========

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
