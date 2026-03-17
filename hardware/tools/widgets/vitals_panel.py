"""Vitals panel widget — current values, sparkline charts, and state annotation bar.

Uses PlotextPlot for rolling HR/BR charts with a Rich Text header and
state annotation bars rendered as colored block characters.
"""

from __future__ import annotations
from collections import deque
from dataclasses import field, dataclass

from rich.text import Text
from textual.app import ComposeResult
from textual.widgets import Static
from textual.containers import Vertical, Horizontal


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
    """Return Rich color name for a device state."""
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
# VitalsPanel Widget (container with charts)
# ---------------------------------------------------------------------------

_BAR_WIDTH = 40


class _VitalsHeader(Static):
    """Top line: VITALS label + acceptance status + current HR/BR values."""

    def __init__(self, **kwargs: object) -> None:
        """Create header with default empty state."""
        super().__init__("", **kwargs)

    def refresh_data(self, data: VitalsData) -> None:
        """Update the header text."""
        content = Text()
        content.append("VITALS", style="bold")
        content.append("  ")
        content.append_text(_acceptance_label(data.hr_allowed, data.hr_valid))
        content.append("   ")

        content.append("HR ", style="bold")
        if data.hr_current is not None:
            content.append(f"{data.hr_current:.0f}", style="bold red")
            content.append(" bpm  ")
        else:
            content.append("-- bpm  ", style="dim")

        content.append("BR ", style="bold")
        if data.br_current is not None:
            content.append(f"{data.br_current:.0f}", style="bold cyan")
            content.append(" bpm")
        else:
            content.append("-- bpm", style="dim")

        self.update(content)


class _StateBar(Static):
    """Single-row state annotation bar under a chart."""

    def __init__(self, label: str, **kwargs: object) -> None:
        """Create bar with label prefix."""
        super().__init__("", **kwargs)
        self._label = label

    def refresh_states(self, states: list[str]) -> None:
        """Redraw the bar from state list."""
        content = Text()
        content.append(f"  {self._label} ", style="dim")
        content.append_text(render_state_bar(states, _BAR_WIDTH))
        self.update(content)


class VitalsPanel(Vertical):
    """Vitals display with header, two PlotextPlot charts, and state annotation bars."""

    DEFAULT_CSS = """
    VitalsPanel {
        height: 1fr;
        min-height: 10;
        border: solid $primary;
        padding: 0 1;
    }
    VitalsPanel > Horizontal {
        height: 1fr;
        min-height: 6;
    }
    VitalsPanel PlotextPlot {
        width: 1fr;
        height: 100%;
        min-height: 5;
    }
    VitalsPanel > _VitalsHeader {
        height: 1;
    }
    VitalsPanel > _StateBar {
        height: 1;
    }
    """

    def __init__(self, **kwargs: object) -> None:
        """Create the vitals panel with chart widgets."""
        super().__init__(**kwargs)
        self._data = VitalsData()

    def compose(self) -> ComposeResult:
        """Build layout: header, two charts side by side, state bars."""
        yield _VitalsHeader(id="vitals-header")
        if _HAS_PLOTEXT:
            with Horizontal():
                yield PlotextPlot(id="hr-chart")
                yield PlotextPlot(id="br-chart")
        else:
            yield Static("(install textual-plotext for charts)", id="no-charts")
        yield _StateBar("HR", id="hr-state-bar")
        yield _StateBar("BR", id="br-state-bar")

    def update_vitals(self, data: VitalsData) -> None:
        """Push new vitals data and re-render all sub-widgets."""
        self._data = data

        try:
            self.query_one("#vitals-header", _VitalsHeader).refresh_data(data)
        except Exception:
            pass

        # Update charts
        if _HAS_PLOTEXT:
            try:
                _render_chart(self.query_one("#hr-chart", PlotextPlot), data.hr_series, "red", "Heart Rate (bpm)")
            except Exception:
                pass
            try:
                _render_chart(self.query_one("#br-chart", PlotextPlot), data.br_series, "cyan", "Breathing (bpm)")
            except Exception:
                pass

        # Update state bars
        _, _, hr_states = data.hr_series.get_series()
        _, _, br_states = data.br_series.get_series()
        try:
            self.query_one("#hr-state-bar", _StateBar).refresh_states(hr_states)
        except Exception:
            pass
        try:
            self.query_one("#br-state-bar", _StateBar).refresh_states(br_states)
        except Exception:
            pass


def _render_chart(chart: PlotextPlot, series: VitalsTimeSeries, color: str, title: str) -> None:  # type: ignore[type-arg]
    """Render a time series into a PlotextPlot widget."""
    timestamps, values, _states = series.get_series()

    plt = chart.plt
    plt.clear_figure()
    plt.theme("dark")
    plt.title(title)

    if len(values) < 2:
        chart.refresh()
        return

    t_max = timestamps[-1]
    x_data = [t - t_max for t in timestamps]

    plt.plot(x_data, values, color=color)
    plt.xlabel("time (s)")
    chart.refresh()


def render_chart(chart_widget: object, series: VitalsTimeSeries, color: str) -> None:
    """Render a time series into a PlotextPlot widget.

    Kept for backwards compatibility.  Safe to call when plotext is unavailable.
    """
    if not _HAS_PLOTEXT:
        return
    if not isinstance(chart_widget, PlotextPlot):
        return
    _render_chart(chart_widget, series, color, "")
