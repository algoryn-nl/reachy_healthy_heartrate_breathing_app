"""On-demand vitals trends and wellness summary tool."""

from __future__ import annotations
import logging
from typing import Any, Dict
from dataclasses import asdict

from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies, tool_ok, tool_error


logger = logging.getLogger(__name__)


class VitalsTrendsTool(Tool):
    """Retrieve wellness trends and vitals history summary."""

    name = "vitals_trends"
    description = (
        "Retrieve a summary of vitals trends over the past days. "
        "Shows average heart rate and breathing rate, trend direction, "
        "resting time, and any recent notable observations. "
        "Use when the user asks about their vitals history or wellness trends."
    )
    parameters_schema = {
        "type": "object",
        "properties": {
            "days": {
                "type": "integer",
                "description": "Number of days to look back (default 7, max 30)",
                "minimum": 1,
                "maximum": 30,
            },
        },
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Return vitals trend summary and recent insights."""
        if deps.trend_analyzer is None:
            return tool_error("Trend analysis not available")

        days = min(int(kwargs.get("days", 7)), 30)

        try:
            summary = deps.trend_analyzer.get_summary(days=days)
            recent = deps.trend_analyzer.recent_insights(limit=5)
            return tool_ok(
                summary=asdict(summary),
                recent_insights=recent,
            )
        except Exception as e:
            logger.warning("vitals_trends failed: %s", e, exc_info=True)
            return tool_error(f"Failed to retrieve trends: {e}")
