"""Vitals panel widget — current values, acceptance status, and state annotation bar.

Renders as a Rich Text block inside a Static widget.  PlotextPlot charts
are mounted separately by the app and updated via :func:`render_chart`.
"""

from __future__ import annotations
from collections import deque
from dataclasses import field, dataclass

from rich.text import Text
from textual.widgets import Static


try:
    from textual_plotext import PlotextPlot

    _HAS_PLOTEXT = True
except ImportError:  # pragma: no cover
    _HAS_PLOTEXT = False


# ---------------------------------------------------------------------------
# State color map for the annotation bar (block characters)
# ---------------------------------------------------------------------------

_STATE_BAR_COLORS: dict[str, str] = {
    "STILL_NEAR": "green",
    "MOVING": "yellow",
    "PRESENT_FAR": "yellow",
    "RESTING_VITALS": "blue",
    "NO_TARGET": "bright_black",
    "MULTI_TARGET": "magenta",
}


def _state_bar_color(state: str) -> str:
    return _STATE_BAR_COLORS.get(state, "bright_black")


# ---------------------------------------------------------------------------
# Time series data buffer (local stub - will map to sensor_models later)
# ---------------------------------------------------------------------------


@dataclass
class VitalsTimeSeries:
    """Ring buffer for a single vital sign's time series."""

    max_size: int = 300
    timestamps: deque[float] = field(default_factory=deque)
    values: deque[float] = field(default_factory=deque)
    states: deque[str] = field(default_factory=deque)

    def __post_init__(self) -> None:
        """Apply max_size to deques."""
        self.timestamps = deque(self.timestamps, maxlen=self.max_size)
        self.values = deque(self.values, maxlen=self.max_size)
        self.states = deque(self.states, maxlen=self.max_size)

    def append(self, timestamp: float, value: float, state: str) -> None:
        """Add a data point."""
        self.timestamps.append(timestamp)
        self.values.append(value)
        self.states.append(state)

    def __len__(self) -> int:
        """Return number of data points in the buffer."""
        return len(self.values)

    def get_series(self) -> tuple[list[float], list[float], list[str]]:
        """Return (timestamps, values, states) as plain lists."""
        return list(self.timestamps), list(self.values), list(self.states)


@dataclass
class VitalsData:
    """Snapshot of vitals data for the panel."""

    hr_current: float | None = None
    br_current: float | None = None
    hr_allowed: int = 0
    hr_valid: int = 0
    br_allowed: int = 0
    br_valid: int = 0
    hr_series: VitalsTimeSeries = field(default_factory=VitalsTimeSeries)
    br_series: VitalsTimeSeries = field(default_factory=VitalsTimeSeries)


# ---------------------------------------------------------------------------
# State annotation bar rendering
# ---------------------------------------------------------------------------


def render_state_bar(states: list[str], width: int) -> Text:
    """Render a single-row state annotation bar of colored block characters.

    Each character maps to a time bucket.  Color determined by device state
    at that bucket.  Uses U+2588 (full block).
    """
    if not states:
        return Text("\u2588" * width, style="bright_black")

    result = Text()
    n = len(states)
    for i in range(width):
        # Map bar position to state index
        idx = min(int(i * n / width), n - 1)
        color = _state_bar_color(states[idx])
        result.append("\u2588", style=color)
    return result


# ---------------------------------------------------------------------------
# Acceptance status label
# ---------------------------------------------------------------------------


def _acceptance_label(allowed: int, valid: int) -> Text:
    """Return a colored ok/fail label for bio acceptance gate."""
    parts = Text()
    a_style = "green" if allowed else "red"
    v_style = "green" if valid else "red"
    parts.append("ok" if allowed else "fail", style=a_style)
    parts.append("/")
    parts.append("ok" if valid else "fail", style=v_style)
    return parts


# ---------------------------------------------------------------------------
# VitalsPanel Widget
# ---------------------------------------------------------------------------

_BAR_WIDTH = 40


class VitalsPanel(Static):
    """Vitals status display rendered as Rich Text.

    Shows header with acceptance status, HR/BR current values, and
    state annotation bars.  This is a leaf Static widget -- no children.
    """

    def __init__(self, **kwargs: object) -> None:
        """Create the vitals panel."""
        super().__init__("", **kwargs)
        self._data = VitalsData()

    def update_vitals(self, data: VitalsData) -> None:
        """Push new vitals data and re-render."""
        self._data = data
        self._refresh_content()

    def _refresh_content(self) -> None:
        """Rebuild the Rich Text content from current data."""
        data = self._data
        content = Text()

        # Header line
        content.append("VITALS", style="bold")
        content.append("  ")
        content.append_text(_acceptance_label(data.hr_allowed, data.hr_valid))
        content.append("\n")

        # HR line
        content.append("  HR ", style="bold")
        if data.hr_current is not None:
            content.append(f"{data.hr_current:.0f}", style="bold red")
            content.append(" bpm  ")
        else:
            content.append("-- bpm  ", style="dim")

        # BR on same line
        content.append("BR ", style="bold")
        if data.br_current is not None:
            content.append(f"{data.br_current:.0f}", style="bold cyan")
            content.append(" bpm")
        else:
            content.append("-- bpm", style="dim")
        content.append("\n")

        # State annotation bars
        _, _, hr_states = data.hr_series.get_series()
        _, _, br_states = data.br_series.get_series()
        content.append("  HR ")
        content.append_text(render_state_bar(hr_states, _BAR_WIDTH))
        content.append("\n")
        content.append("  BR ")
        content.append_text(render_state_bar(br_states, _BAR_WIDTH))

        self.update(content)


def render_chart(chart_widget: object, series: VitalsTimeSeries, color: str) -> None:
    """Render a time series into a PlotextPlot widget.

    Call this from the app's event handler when using PlotextPlot charts.
    Safe to call when plotext is unavailable (no-op).
    """
    if not _HAS_PLOTEXT:
        return
    if not isinstance(chart_widget, PlotextPlot):
        return

    timestamps, values, _states = series.get_series()

    plt = chart_widget.plt
    plt.clear_figure()
    plt.theme("dark")

    if len(values) < 2:
        plt.title("waiting for data...")
        chart_widget.refresh()
        return

    # Use relative time (seconds ago)
    if timestamps:
        t_max = timestamps[-1]
        x_data = [t - t_max for t in timestamps]
    else:
        x_data = list(range(len(values)))

    plt.plot(x_data, values, color=color)
    plt.xlabel("time")
    chart_widget.refresh()
