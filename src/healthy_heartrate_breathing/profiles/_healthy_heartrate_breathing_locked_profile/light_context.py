"""Ambient-light context policy tool for conversation behavior guidance."""

from __future__ import annotations
import os
import logging
from typing import Any, Dict
from datetime import datetime

from healthy_heartrate_breathing.env_utils import (
    env_flag,
    coerce_int,
    coerce_bool,
    coerce_float,
    extract_lux_from_mmwave_result,
)
from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies


logger = logging.getLogger(__name__)


def _resolve_local_hour(local_hour: Any, local_time: Any) -> int:
    """Resolve local hour from direct input, ISO datetime, or system time."""
    if isinstance(local_hour, int) and 0 <= local_hour <= 23:
        return local_hour

    if isinstance(local_time, str) and local_time.strip():
        candidate = local_time.strip()
        if candidate.endswith("Z"):
            candidate = candidate[:-1] + "+00:00"
        try:
            parsed = datetime.fromisoformat(candidate)
            return int(parsed.hour)
        except ValueError:
            pass

    return datetime.now().hour


class LightContext(Tool):
    """Map lux/time/preferences into a conversation context state."""

    name = "light_context"
    description = (
        "Infer ambient-light conversation context from lux, time, and user preferences. "
        "Returns a context state with recommended conversation mode/actions."
    )
    parameters_schema = {
        "type": "object",
        "properties": {
            "lux": {"type": "number", "description": "Current ambient light in lux"},
            "previous_lux": {"type": "number", "description": "Recent previous lux value (for change detection)"},
            "lux_delta_60s": {"type": "number", "description": "Lux change over ~60s (negative means getting darker)"},
            "presence_detected": {"type": "boolean", "description": "Whether a person is currently detected nearby"},
            "active_interaction": {"type": "boolean", "description": "Whether conversation is actively in progress"},
            "low_light_duration_min": {
                "type": "number",
                "minimum": 0,
                "description": "How long ambient light has stayed low during the current period",
            },
            "local_hour": {"type": "integer", "minimum": 0, "maximum": 23},
            "local_time": {
                "type": "string",
                "description": "Optional local timestamp (ISO-8601) used to derive local hour",
            },
            "prefers_dim": {"type": "boolean", "description": "User preference: comfortable in dim rooms"},
            "light_sensitive": {"type": "boolean", "description": "User preference: light-sensitive / migraine-prone"},
            "allow_wellness_nudges": {"type": "boolean", "description": "Whether optional comfort nudges are allowed"},
            "user_typical_day_low_lux": {
                "type": "number",
                "minimum": 0,
                "description": "User-specific daytime low-light baseline for personalization",
            },
            "mmwave_result": {
                "type": "object",
                "description": "Optional raw mmWave tool result to auto-extract latest lux",
            },
        },
        "required": [],
    }

    DEFAULT_LOW_LUX_THRESHOLD = 40.0
    DEFAULT_BRIGHT_LUX_THRESHOLD = 250.0
    DEFAULT_SHARP_DROP_LUX = 80.0
    DEFAULT_PROLONGED_LOW_LIGHT_MIN = 45.0
    DEFAULT_DAY_START_HOUR = 7
    DEFAULT_NIGHT_START_HOUR = 20

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Classify current light context and return recommended conversation policy."""
        del deps  # no hardware control needed

        if not env_flag("HEALTHY_LIGHT_CONTEXT_ENABLED", True):
            return {
                "enabled": False,
                "status": "disabled",
                "context_state": "neutral",
                "recommended_mode": "balanced",
                "recommended_actions": ["standard_conversation_policy"],
                "confidence": 0.0,
                "cooldown_hint_s": 120,
                "reason_codes": ["tool_disabled"],
                "reason": "HEALTHY_LIGHT_CONTEXT_ENABLED=false",
            }

        low_lux_threshold = coerce_float(
            kwargs.get("low_lux_threshold"),
            coerce_float(os.getenv("HEALTHY_LIGHT_LOW_LUX_THRESHOLD"), self.DEFAULT_LOW_LUX_THRESHOLD),
        )
        bright_lux_threshold = coerce_float(
            kwargs.get("bright_lux_threshold"),
            coerce_float(os.getenv("HEALTHY_LIGHT_BRIGHT_LUX_THRESHOLD"), self.DEFAULT_BRIGHT_LUX_THRESHOLD),
        )
        sharp_drop_lux = coerce_float(
            kwargs.get("sharp_drop_lux"),
            coerce_float(os.getenv("HEALTHY_LIGHT_SHARP_DROP_LUX"), self.DEFAULT_SHARP_DROP_LUX),
        )
        prolonged_low_light_min = coerce_float(
            kwargs.get("prolonged_low_light_min"),
            coerce_float(os.getenv("HEALTHY_LIGHT_PROLONGED_MIN"), self.DEFAULT_PROLONGED_LOW_LIGHT_MIN),
        )
        day_start_hour = coerce_int(kwargs.get("day_start_hour"), self.DEFAULT_DAY_START_HOUR)
        night_start_hour = coerce_int(kwargs.get("night_start_hour"), self.DEFAULT_NIGHT_START_HOUR)

        lux = coerce_float(kwargs.get("lux"))
        mmwave_lux = extract_lux_from_mmwave_result(kwargs.get("mmwave_result"))
        if lux is None:
            lux = mmwave_lux

        previous_lux = coerce_float(kwargs.get("previous_lux"))
        lux_delta_60s = coerce_float(kwargs.get("lux_delta_60s"))
        if lux_delta_60s is None and lux is not None and previous_lux is not None:
            lux_delta_60s = lux - previous_lux

        local_hour = _resolve_local_hour(kwargs.get("local_hour"), kwargs.get("local_time"))
        is_night = local_hour >= night_start_hour or local_hour < day_start_hour
        prefers_dim = coerce_bool(kwargs.get("prefers_dim"), False)
        light_sensitive = coerce_bool(kwargs.get("light_sensitive"), False)
        allow_wellness_nudges = coerce_bool(kwargs.get("allow_wellness_nudges"), True)
        active_interaction = coerce_bool(kwargs.get("active_interaction"), True)
        presence_detected_raw = kwargs.get("presence_detected")
        presence_detected = bool(presence_detected_raw) if isinstance(presence_detected_raw, bool) else None
        low_light_duration_min = coerce_float(kwargs.get("low_light_duration_min"), 0.0) or 0.0
        user_typical_day_low_lux = coerce_float(kwargs.get("user_typical_day_low_lux"))

        reason_codes: list[str] = []
        context_state = "neutral"
        recommended_mode = "balanced"
        recommended_actions = ["standard_conversation_policy"]
        confidence = 0.55
        cooldown_hint_s = 120

        if lux is None:
            reason_codes.append("lux_missing")
            recommended_actions = ["use_standard_policy_without_light_nudges"]
            confidence = 0.35
        else:
            is_low_lux = lux <= (low_lux_threshold or self.DEFAULT_LOW_LUX_THRESHOLD)
            is_bright_lux = lux >= (bright_lux_threshold or self.DEFAULT_BRIGHT_LUX_THRESHOLD)
            sharp_drop = lux_delta_60s is not None and lux_delta_60s <= -(
                sharp_drop_lux or self.DEFAULT_SHARP_DROP_LUX
            )
            presence_ok = presence_detected is not False

            intentional_dim = False
            if not is_night and is_low_lux:
                if prefers_dim:
                    intentional_dim = True
                    reason_codes.append("prefers_dim")
                if light_sensitive:
                    intentional_dim = True
                    reason_codes.append("light_sensitive")
                if user_typical_day_low_lux is not None and lux <= user_typical_day_low_lux:
                    intentional_dim = True
                    reason_codes.append("user_baseline_low_lux")

            # -----------------------------------------------------------------
            # Classification decision tree (order matters!)
            #
            # The if/elif chain below is evaluated top-to-bottom.  The FIRST
            # matching branch wins.  Precedence rationale:
            #
            #   1. unexpected_darkening   — sharp lux drop + active + presence
            #      Checked first because a sudden blackout overrides everything,
            #      including nighttime (a power cut at 23:00 is not "wind-down").
            #
            #   2. night_wind_down        — nighttime + low lux + presence
            #      After ruling out a sharp drop, low lux at night is the most
            #      common quiet scenario.
            #
            #   3. intentional_dim_work   — daytime + low lux + user preference
            #      User explicitly prefers dim or is light-sensitive.  Checked
            #      before strain risk so we never nudge users who chose low light.
            #
            #   4. prolonged_low_light_strain_risk — daytime + low lux + duration
            #      Only fires when no preference flag explains the low light AND
            #      the user has stayed in it long enough.  Gated by
            #      allow_wellness_nudges.
            #
            #   5. bright_active          — daytime + bright lux + presence
            #      High-energy daytime environment.
            #
            #   6. neutral (else)         — everything that didn't match above
            #
            # Overlap notes:
            #   • sharp_drop can co-occur with is_night; precedence ensures
            #     unexpected_darkening wins.
            #   • intentional_dim and prolonged_strain share is_low_lux during
            #     daytime; preference flags disambiguate.
            #   • bright_active and neutral are mutually exclusive by threshold.
            # -----------------------------------------------------------------
            if sharp_drop and active_interaction and presence_ok:
                context_state = "unexpected_darkening"
                recommended_mode = "quiet"
                recommended_actions = [
                    "offer_gentle_check_in",
                    "pause_unsolicited_prompts",
                    "rate_limit_follow_up_checks",
                ]
                confidence = 0.9
                cooldown_hint_s = 300
                reason_codes.append("sharp_lux_drop")
            elif is_night and is_low_lux and presence_ok:
                context_state = "night_wind_down"
                recommended_mode = "quiet"
                recommended_actions = [
                    "use_short_calm_prompts",
                    "reduce_interruptions",
                    "avoid_high_energy_suggestions",
                ]
                confidence = 0.82
                cooldown_hint_s = 180
                reason_codes.append("night_low_lux")
            elif not is_night and is_low_lux and intentional_dim:
                context_state = "intentional_dim_work"
                recommended_mode = "quiet" if light_sensitive else "balanced"
                recommended_actions = [
                    "respect_dim_environment",
                    "avoid_increase_light_nudges",
                    "keep_low_stimulus_prompting",
                ]
                confidence = 0.8
                cooldown_hint_s = 180
                reason_codes.append("intentional_dim")
            elif (
                not is_night
                and is_low_lux
                and allow_wellness_nudges
                and low_light_duration_min >= (prolonged_low_light_min or self.DEFAULT_PROLONGED_LOW_LIGHT_MIN)
            ):
                context_state = "prolonged_low_light_strain_risk"
                recommended_mode = "quiet"
                recommended_actions = [
                    "offer_optional_comfort_break",
                    "offer_optional_posture_or_breathing_reset",
                    "limit_to_one_nudge_per_window",
                ]
                confidence = 0.72
                cooldown_hint_s = 900
                reason_codes.append("prolonged_daytime_low_lux")
            elif not is_night and is_bright_lux and presence_ok:
                context_state = "bright_active"
                recommended_mode = "active"
                recommended_actions = [
                    "use_action_oriented_prompts",
                    "normal_interruption_policy",
                ]
                confidence = 0.7
                cooldown_hint_s = 120
                reason_codes.append("bright_daytime")
            else:
                if is_low_lux:
                    reason_codes.append("low_lux_unclassified")
                else:
                    reason_codes.append("neutral_conditions")

        return {
            "enabled": True,
            "policy_version": "light_context_v1",
            "context_state": context_state,
            "recommended_mode": recommended_mode,
            "recommended_actions": recommended_actions,
            "confidence": round(confidence, 2),
            "cooldown_hint_s": cooldown_hint_s,
            "reason_codes": reason_codes,
            "observations": {
                "lux": lux,
                "lux_delta_60s": lux_delta_60s,
                "local_hour": local_hour,
                "is_night": is_night,
                "presence_detected": presence_detected,
                "active_interaction": active_interaction,
                "low_light_duration_min": low_light_duration_min,
                "prefers_dim": prefers_dim,
                "light_sensitive": light_sensitive,
                "allow_wellness_nudges": allow_wellness_nudges,
            },
        }
