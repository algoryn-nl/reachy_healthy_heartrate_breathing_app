"""Proximity/occlusion context policy tool for conversation behavior guidance.

The BH1750 lux sensor sits behind/below the person at the Reachy Mini robot.
When someone sits down, their body occludes the sensor and lux drops dramatically.
This tool reframes lux as a proximity/occlusion signal that complements mmWave
radar data, rather than treating it as an ambient light measurement.
"""

from __future__ import annotations
import os
import logging
from typing import Any, Dict

from healthy_heartrate_breathing.env_utils import (
    env_flag,
    coerce_float,
    extract_lux_from_mmwave_result,
)
from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies, tool_ok


logger = logging.getLogger(__name__)


class LightContext(Tool):
    """Map lux + mmWave presence into a proximity/occlusion context state."""

    name = "light_context"
    description = (
        "Infer proximity/occlusion context from lux sensor and mmWave presence data. "
        "The lux sensor sits behind the user — low lux means someone is close, not that "
        "the room is dark. Returns a context state with recommended conversation mode/actions."
    )
    parameters_schema = {
        "type": "object",
        "properties": {
            "lux": {"type": "number", "description": "Current lux reading from BH1750 sensor"},
            "previous_lux": {"type": "number", "description": "Recent previous lux value (for change detection)"},
            "lux_delta_60s": {
                "type": "number",
                "description": "Lux change over ~60s (negative means increasing occlusion)",
            },
            "presence_detected": {
                "type": "boolean",
                "description": "Whether a person is currently detected by mmWave",
            },
            "target_distance_cm": {
                "type": "number",
                "minimum": 0,
                "description": "Distance to nearest target in cm (from mmWave)",
            },
            "mmwave_result": {
                "type": "object",
                "description": "Optional raw mmWave tool result to auto-extract latest lux",
            },
        },
        "required": [],
    }

    DEFAULT_LOW_LUX_THRESHOLD = 40.0
    DEFAULT_MODERATE_LUX_THRESHOLD = 120.0
    DEFAULT_SHARP_DROP_LUX = 60.0

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Classify proximity/occlusion context and return recommended conversation policy."""
        del deps  # no hardware control needed

        if not env_flag("HEALTHY_LIGHT_CONTEXT_ENABLED", True):
            return tool_ok(
                "disabled",
                enabled=False,
                context_state="neutral",
                recommended_mode="balanced",
                recommended_actions=["standard_conversation_policy"],
                confidence=0.0,
                cooldown_hint_s=120,
                reason_codes=["tool_disabled"],
                reason="HEALTHY_LIGHT_CONTEXT_ENABLED=false",
            )

        low_lux_threshold = coerce_float(
            kwargs.get("low_lux_threshold"),
            coerce_float(os.getenv("HEALTHY_LIGHT_LOW_LUX_THRESHOLD"), self.DEFAULT_LOW_LUX_THRESHOLD),
        )
        moderate_lux_threshold = coerce_float(
            kwargs.get("moderate_lux_threshold"),
            coerce_float(os.getenv("HEALTHY_LIGHT_MODERATE_LUX_THRESHOLD"), self.DEFAULT_MODERATE_LUX_THRESHOLD),
        )
        sharp_drop_lux = coerce_float(
            kwargs.get("sharp_drop_lux"),
            coerce_float(os.getenv("HEALTHY_LIGHT_SHARP_DROP_LUX"), self.DEFAULT_SHARP_DROP_LUX),
        )

        lux = coerce_float(kwargs.get("lux"))
        mmwave_lux = extract_lux_from_mmwave_result(kwargs.get("mmwave_result"))
        if lux is None:
            lux = mmwave_lux

        previous_lux = coerce_float(kwargs.get("previous_lux"))
        lux_delta_60s = coerce_float(kwargs.get("lux_delta_60s"))
        if lux_delta_60s is None and lux is not None and previous_lux is not None:
            lux_delta_60s = lux - previous_lux

        presence_detected_raw = kwargs.get("presence_detected")
        presence_detected = bool(presence_detected_raw) if isinstance(presence_detected_raw, bool) else None
        target_distance_cm = coerce_float(kwargs.get("target_distance_cm"))

        reason_codes: list[str] = []
        context_state = "clear_path"
        recommended_mode = "balanced"
        recommended_actions = ["standard_conversation_policy"]
        confidence = 0.55
        cooldown_hint_s = 120

        if lux is None:
            context_state = "neutral"
            reason_codes.append("lux_missing")
            recommended_actions = ["use_standard_policy_without_light_nudges"]
            confidence = 0.35
        else:
            is_low_lux = lux <= low_lux_threshold
            is_moderate_drop = lux <= moderate_lux_threshold
            sharp_drop = lux_delta_60s is not None and lux_delta_60s <= -sharp_drop_lux
            presence_ok = presence_detected is True

            # -----------------------------------------------------------------
            # Classification decision tree (order matters!)
            #
            #   1. sudden_occlusion  — sharp lux drop + presence detected
            #      Someone just sat down or moved closer.
            #
            #   2. close_presence    — low lux + presence detected
            #      Person is seated close, occluding the sensor.
            #
            #   3. partial_occlusion — moderate lux drop + presence detected
            #      Someone is nearby but not fully occluding.
            #
            #   4. clear_path (else) — high/moderate lux OR no presence
            #      No one blocking the sensor.
            # -----------------------------------------------------------------
            if sharp_drop and presence_ok:
                context_state = "sudden_occlusion"
                recommended_mode = "engaged"
                recommended_actions = [
                    "note_proximity_change",
                    "consider_greeting_if_new",
                ]
                confidence = 0.9
                cooldown_hint_s = 60
                reason_codes.append("sharp_lux_drop")
            elif is_low_lux and presence_ok:
                context_state = "close_presence"
                recommended_mode = "engaged"
                recommended_actions = [
                    "vitals_likely_available",
                    "use_warm_conversational_tone",
                ]
                confidence = 0.85
                cooldown_hint_s = 120
                reason_codes.append("low_lux_with_presence")
            elif is_moderate_drop and presence_ok:
                context_state = "partial_occlusion"
                recommended_mode = "balanced"
                recommended_actions = [
                    "someone_may_be_nearby",
                ]
                confidence = 0.65
                cooldown_hint_s = 120
                reason_codes.append("moderate_lux_with_presence")
            else:
                context_state = "clear_path"
                recommended_mode = "balanced"
                recommended_actions = ["standard_conversation_policy"]
                confidence = 0.55
                cooldown_hint_s = 120
                if not presence_ok:
                    reason_codes.append("no_presence")
                else:
                    reason_codes.append("high_lux")

        return tool_ok(
            context_state,
            enabled=True,
            context_state=context_state,
            policy_version="proximity_context_v1",
            recommended_mode=recommended_mode,
            recommended_actions=recommended_actions,
            confidence=round(confidence, 2),
            cooldown_hint_s=cooldown_hint_s,
            reason_codes=reason_codes,
            observations={
                "lux": lux,
                "lux_delta_60s": lux_delta_60s,
                "presence_detected": presence_detected,
                "target_distance_cm": target_distance_cm,
            },
        )
