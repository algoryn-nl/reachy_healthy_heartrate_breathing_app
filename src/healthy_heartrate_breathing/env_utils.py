"""Shared value-coercion and environment-variable parsing helpers.

Two function groups:

- ``coerce_*`` — convert arbitrary values (tool kwargs, config dicts) to typed
  Python values with a fallback default. They never touch ``os.getenv``.
- ``env_*`` — read an environment variable by name, then coerce the raw string.

Plus ``extract_lux_from_mmwave_result`` for lux extraction from mmWave tool
responses (used by both ``openai_realtime`` and ``light_context``).
"""

from __future__ import annotations
import os
import logging
from typing import Any


logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Value coercion (for tool kwargs / arbitrary input)
# ---------------------------------------------------------------------------

_TRUTHY = {"1", "true", "yes", "on"}
_FALSY = {"0", "false", "no", "off"}


def coerce_bool(value: Any, default: bool) -> bool:
    """Convert *value* to bool with a fallback.

    Accepts ``bool``, or a string in {"1","true","yes","on"} / {"0","false","no","off"}.
    Everything else (including ``None``) returns *default*.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        text = value.strip().lower()
        if text in _TRUTHY:
            return True
        if text in _FALSY:
            return False
    return default


def coerce_float(value: Any, default: float | None = None) -> float | None:
    """Convert *value* to float, returning *default* on failure or ``None``."""
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def coerce_float_nonneg(value: Any, default: float) -> float:
    """Like ``coerce_float`` but clamps the result to >= 0."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return max(0.0, parsed)


def coerce_int(value: Any, default: int) -> int:
    """Convert *value* to int, returning *default* on failure."""
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def coerce_ms(value: Any, default: int) -> int:
    """Coerce *value* to a positive int (min 1), for millisecond cadences."""
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return max(1, parsed)


# ---------------------------------------------------------------------------
# Environment variable readers
# ---------------------------------------------------------------------------


def env_flag(name: str, default: bool = False) -> bool:
    """Read a boolean flag from the environment."""
    raw = os.getenv(name)
    if raw is None:
        return default
    text = raw.strip().lower()
    if text in _TRUTHY:
        return True
    if text in _FALSY:
        return False
    logger.warning("Invalid boolean value for %s=%r, using default=%s", name, raw, default)
    return default


def env_float(
    name: str,
    default: float,
    *,
    min_value: float | None = None,
    max_value: float | None = None,
) -> float:
    """Read a float from the environment, with optional bounds clamping."""
    raw = os.getenv(name)
    if raw is None:
        return default

    try:
        value = float(raw.strip())
    except ValueError:
        logger.warning("Invalid float for %s=%r, using default=%.3f", name, raw, default)
        return default

    if min_value is not None and value < min_value:
        logger.warning("Environment value for %s=%r below minimum %s; using %.3f", name, raw, min_value, min_value)
        return float(min_value)
    if max_value is not None and value > max_value:
        logger.warning("Environment value for %s=%r above maximum %s; using %.3f", name, raw, max_value, max_value)
        return float(max_value)
    return value


def env_int(
    name: str,
    default: int,
    *,
    min_value: int | None = None,
    max_value: int | None = None,
) -> int:
    """Read an int from the environment, with optional bounds clamping."""
    raw = os.getenv(name)
    if raw is None:
        return default

    try:
        value = int(float(raw.strip()))
    except ValueError:
        logger.warning("Invalid int for %s=%r, using default=%s", name, raw, default)
        return default

    if min_value is not None and value < min_value:
        logger.warning("Environment value for %s=%r below minimum %s; using %s", name, raw, min_value, min_value)
        return min_value
    if max_value is not None and value > max_value:
        logger.warning("Environment value for %s=%r above maximum %s; using %s", name, raw, max_value, max_value)
        return max_value
    return value


# ---------------------------------------------------------------------------
# Shared domain helpers
# ---------------------------------------------------------------------------


def extract_lux_from_mmwave_result(result: Any) -> float | None:
    """Extract the latest lux value from an mmWave tool result dict.

    Searches known nested paths in priority order and returns the first
    numeric value found, or ``None``.
    """
    if not isinstance(result, dict):
        return None

    candidates = (
        ("measure", "latest_light", "lux"),
        ("scan", "latest_light", "lux"),
        ("measure", "light_summary", "latest_lux"),
        ("scan", "light_summary", "latest_lux"),
    )
    for path in candidates:
        current: Any = result
        for key in path:
            if not isinstance(current, dict):
                current = None
                break
            current = current.get(key)
        if isinstance(current, (int, float)):
            return float(current)
    return None
