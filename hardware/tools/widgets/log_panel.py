"""Log panel widget — scrolling event log with notable/firehose toggle.

Collapsed (3 lines) by default, expandable via keybinding.  Supports
notable-only filtering (state transitions, new bio, significant lux
changes, errors) and a firehose mode showing every event.  Color-coded
by event type.  Rendered as Rich Text in a Static widget.
"""

from __future__ import annotations
import time
from datetime import datetime
from collections import deque
from dataclasses import field, dataclass

from rich.text import Text
from textual.widgets import Static
from textual.reactive import reactive


# ---------------------------------------------------------------------------
# Event type color map
# ---------------------------------------------------------------------------

_EVENT_COLORS: dict[str, str] = {
    "state": "#4ade80",
    "bio": "#60a5fa",
    "light": "#fbbf24",
    "targets": "#a78bfa",
    "diag": "#888888",
    "ack": "#4ade80",
    "err": "#ef4444",
    "pong": "#4ade80",
    "hello": "#60a5fa",
}


def _event_color(event_type: str) -> str:
    return _EVENT_COLORS.get(event_type, "#888888")


# ---------------------------------------------------------------------------
# Notable filter (standalone — will delegate to sensor_models later)
# ---------------------------------------------------------------------------


class NotableFilter:
    """Determine whether an event is notable (worth showing in notable mode).

    Notable events:
    - State transitions (state changed from previous)
    - Bio readings with new values (hr_new=1 or br_new=1)
    - Light readings with significant change (>threshold lux delta)
    - Error events
    - Ack / hello events
    """

    def __init__(self, lux_delta_threshold: float = 5.0) -> None:
        """Create a notable filter with the given lux threshold."""
        self._last_state: str | None = None
        self._last_lux: float | None = None
        self._lux_threshold = lux_delta_threshold

    def is_notable(self, event_type: str, data: dict[str, object]) -> bool:
        """Return True if this event is notable."""
        if event_type in ("err", "ack", "hello"):
            return True

        if event_type == "state":
            prev = self._last_state
            new_state = str(data.get("state", ""))
            self._last_state = new_state
            return prev is not None and new_state != prev

        if event_type == "bio":
            return bool(data.get("hr_new") or data.get("br_new"))

        if event_type == "light":
            lux = data.get("lux")
            if lux is None or not isinstance(lux, (int, float)):
                return False
            prev = self._last_lux
            self._last_lux = float(lux)
            if prev is None:
                return False
            return abs(float(lux) - prev) >= self._lux_threshold

        return False


# ---------------------------------------------------------------------------
# Log entry data
# ---------------------------------------------------------------------------


@dataclass
class LogEntry:
    """A single log line."""

    timestamp: float
    event_type: str
    message: str
    data: dict[str, object] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Log formatting
# ---------------------------------------------------------------------------


def format_log_entry(entry: LogEntry) -> Text:
    """Format a log entry as colored Rich Text."""
    dt = datetime.fromtimestamp(entry.timestamp)  # noqa: DTZ006
    time_str = dt.strftime("%H:%M:%S")

    color = _event_color(entry.event_type)
    result = Text()
    result.append(time_str, style="dim")
    result.append(" ")
    result.append(entry.event_type.ljust(7), style=f"bold {color}")
    result.append(" ")
    result.append(entry.message)
    return result


def format_state_event(data: dict[str, object]) -> str:
    """Format a state event for the log."""
    state = data.get("state", "?")
    pose = data.get("pose", "?")
    n = data.get("n_targets", "?")
    dist = data.get("dist_cm")
    dist_str = f" d={dist:.1f}cm" if isinstance(dist, (int, float)) else ""
    hm = data.get("head_moving", "?")
    return f"{state} pose={pose} h={data.get('human', '?')} n={n}{dist_str} hm={hm}"


def format_bio_event(data: dict[str, object]) -> str:
    """Format a bio event for the log."""
    parts: list[str] = []
    a = data.get("allowed", 0)
    v = data.get("valid", 0)
    parts.append(f"{'ok' if a else 'fail'}/{'ok' if v else 'fail'}")
    br = data.get("br")
    hr = data.get("hr")
    if br is not None:
        parts.append(f"br={br}")
    if hr is not None:
        parts.append(f"hr={hr}")
    if data.get("br_new"):
        parts.append("+br")
    if data.get("hr_new"):
        parts.append("+hr")
    return " ".join(parts)


def format_light_event(data: dict[str, object]) -> str:
    """Format a light event for the log."""
    lux = data.get("lux")
    if lux is not None and isinstance(lux, (int, float)):
        return f"lux={lux:.1f}"
    return "lux=--"


def format_targets_event(data: dict[str, object]) -> str:
    """Format a targets event for the log."""
    n = data.get("n_targets", data.get("n", "?"))
    trunc = data.get("targets_truncated", False)
    focus = data.get("focus")
    parts = [f"n={n}"]
    if trunc:
        parts.append("TRUNCATED")
    if focus and isinstance(focus, dict):
        r = focus.get("r", "?")
        b = focus.get("bearing", "?")
        parts.append(f"focus={r}m/{b}\u00b0")
    return " ".join(parts)


def format_event_data(event_type: str, data: dict[str, object]) -> str:
    """Format event data into a human-readable log message."""
    formatters: dict[str, object] = {
        "state": format_state_event,
        "bio": format_bio_event,
        "light": format_light_event,
        "targets": format_targets_event,
    }
    fn = formatters.get(event_type)
    if fn is not None and callable(fn):
        return fn(data)
    # Generic fallback
    return " ".join(f"{k}={v}" for k, v in data.items() if k != "type")


# ---------------------------------------------------------------------------
# LogPanel Widget
# ---------------------------------------------------------------------------

_COLLAPSED_LINES = 3
_EXPANDED_LINES = 15
_MAX_BUFFER = 500


class LogPanel(Static):
    """Scrolling event log rendered as Rich Text in a Static widget."""

    is_expanded: reactive[bool] = reactive(False)
    is_firehose: reactive[bool] = reactive(False)

    def __init__(
        self,
        filter_types: set[str] | None = None,
        **kwargs: object,
    ) -> None:
        """Create a log panel with optional event type filtering."""
        super().__init__("", **kwargs)
        self._filter_types = filter_types
        self._notable_filter = NotableFilter()
        self._entries: deque[Text] = deque(maxlen=_MAX_BUFFER)

    def on_mount(self) -> None:
        """Initialize with header."""
        self._refresh_content()

    def add_event(self, event_type: str, data: dict[str, object]) -> None:
        """Add an event to the log, respecting filter and notable mode."""
        # Apply type filter
        if self._filter_types is not None and event_type not in self._filter_types:
            return

        # Apply notable filter in notable mode
        if not self.is_firehose:
            if not self._notable_filter.is_notable(event_type, data):
                return

        message = format_event_data(event_type, data)
        entry = LogEntry(
            timestamp=time.time(),
            event_type=event_type,
            message=message,
            data=data,
        )
        self._entries.append(format_log_entry(entry))
        self._refresh_content()

    def toggle_expanded(self) -> None:
        """Toggle between collapsed and expanded mode."""
        self.is_expanded = not self.is_expanded

    def toggle_firehose(self) -> None:
        """Toggle between notable-only and firehose mode."""
        self.is_firehose = not self.is_firehose

    def watch_is_expanded(self, value: bool) -> None:
        """React to expansion state change."""
        self._refresh_content()

    def watch_is_firehose(self, value: bool) -> None:
        """React to firehose toggle."""
        self._refresh_content()

    def _refresh_content(self) -> None:
        """Rebuild the display from the entry buffer."""
        content = Text()

        # Header
        content.append("LOG", style="bold")
        if self.is_expanded:
            content.append(" (expanded \u2014 l to collapse)", style="dim")
        else:
            content.append(f" ({_COLLAPSED_LINES} lines \u2014 l to expand)", style="dim")
        content.append("  ")
        if self.is_firehose:
            content.append("firehose", style="bold yellow")
        else:
            content.append("notable", style="bold green")
        content.append(" \u00b7 [f] toggle", style="dim")

        # Log lines
        visible = _EXPANDED_LINES if self.is_expanded else _COLLAPSED_LINES
        recent = list(self._entries)[-visible:]
        for entry_text in recent:
            content.append("\n")
            content.append_text(entry_text)

        self.update(content)
