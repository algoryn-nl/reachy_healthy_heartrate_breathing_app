"""Diagnostics tab panel for the mmWave TUI monitor.

Displays protocol event log, hardware counters, connection info,
and computed statistics. Accessible via the 'd' key or [Diag] tab.
"""

from __future__ import annotations
from datetime import datetime
from dataclasses import field, dataclass

from rich.text import Text
from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static, RichLog
from textual.containers import Vertical


# ---------------------------------------------------------------------------
# Local stand-in types — replaced by sensor_models imports after wiring
# ---------------------------------------------------------------------------


@dataclass
class DiagCounters:
    """EVT_DIAG fields. Names match protocol wire format exactly."""

    mmwave_fail_count: int = 0
    mmwave_consecutive_fails: int = 0
    tx_drop_count: int = 0


@dataclass
class ConnectionInfo:
    """Sensor connection metadata."""

    port: str = ""
    baud: int = 115200
    proto_version: int = 0
    feature_bits: int = 0


@dataclass
class FrameStats:
    """Locally computed frame statistics."""

    frames_per_sec: float = 0.0
    bytes_per_sec: float = 0.0
    bad_frame_count: int = 0


@dataclass
class EventRates:
    """Per-event-type rates (events/sec)."""

    state: float = 0.0
    bio: float = 0.0
    targets: float = 0.0
    light: float = 0.0
    diag: float = 0.0


@dataclass
class DiagData:
    """Aggregate container for all diagnostics data fed into DiagPanel."""

    counters: DiagCounters = field(default_factory=DiagCounters)
    connection: ConnectionInfo = field(default_factory=ConnectionInfo)
    frame_stats: FrameStats = field(default_factory=FrameStats)
    event_rates: EventRates = field(default_factory=EventRates)
    bio_acceptance_rate: float = 0.0
    uptime_s: float = 0.0


# ---------------------------------------------------------------------------
# Sub-widgets
# ---------------------------------------------------------------------------

_PROTOCOL_LOG_EVENT_TYPES = frozenset({"ack", "err", "pong", "hello"})


class ProtocolLog(RichLog):
    """Scrolling event log for protocol-level events (ack, err, pong, hello).

    Color-coded: green for ack/pong, red for err, cyan for hello.
    """

    DEFAULT_CSS = """
    ProtocolLog {
        height: 1fr;
        border: solid $surface-lighten-2;
        border-title-color: $text;
    }
    """

    def on_mount(self) -> None:
        """Set the border title after mount."""
        self.border_title = "Protocol Event Log"

    def append_event(self, event_type: str, seq: int, detail: str) -> None:
        """Append a protocol event to the log with color coding."""
        if event_type not in _PROTOCOL_LOG_EVENT_TYPES:
            return

        ts = datetime.now().strftime("%H:%M:%S")  # noqa: DTZ005
        line = Text()
        line.append(f"{ts} ", style="dim")
        line.append(f"{seq:>5} ", style="dim")

        if event_type in ("ack", "pong"):
            line.append(f"{event_type:<5} ", style="green")
        elif event_type == "err":
            line.append(f"{event_type:<5} ", style="bold red")
        elif event_type == "hello":
            line.append(f"{event_type:<5} ", style="cyan")
        else:
            line.append(f"{event_type:<5} ", style="white")

        line.append(detail)
        self.write(line)


class CountersPanel(Static):
    """Shows mmWave hardware diagnostic counters and local bad-frame count."""

    DEFAULT_CSS = """
    CountersPanel {
        height: auto;
        padding: 0 1;
        border: solid $surface-lighten-2;
        border-title-color: $text;
    }
    """

    def on_mount(self) -> None:
        """Set the border title after mount."""
        self.border_title = "Counters"
        self._render_counters(DiagCounters(), bad_frame_count=0)

    def update_counters(self, counters: DiagCounters, bad_frame_count: int = 0) -> None:
        """Refresh displayed counter values."""
        self._render_counters(counters, bad_frame_count)

    def _render_counters(self, counters: DiagCounters, bad_frame_count: int) -> None:
        text = Text()
        text.append("mmWave fails     ", style="dim")
        text.append(f"{counters.mmwave_fail_count}\n")
        text.append("Consecutive fails", style="dim")
        text.append(f" {counters.mmwave_consecutive_fails}\n")
        text.append("TX drops         ", style="dim")
        text.append(f"{counters.tx_drop_count}\n")
        text.append("Bad frames (local)", style="dim")
        text.append(f" {bad_frame_count}")
        self.update(text)


class ConnectionPanel(Static):
    """Shows sensor connection metadata and uptime."""

    DEFAULT_CSS = """
    ConnectionPanel {
        height: auto;
        padding: 0 1;
        border: solid $surface-lighten-2;
        border-title-color: $text;
    }
    """

    def on_mount(self) -> None:
        """Set the border title after mount."""
        self.border_title = "Connection"
        self._render_info(ConnectionInfo(), uptime_s=0.0)

    def update_info(self, info: ConnectionInfo, uptime_s: float = 0.0) -> None:
        """Refresh displayed connection info."""
        self._render_info(info, uptime_s)

    def _render_info(self, info: ConnectionInfo, uptime_s: float) -> None:
        minutes, secs = divmod(int(uptime_s), 60)
        hours, minutes = divmod(minutes, 60)

        text = Text()
        text.append("Port    ", style="dim")
        text.append(f"{info.port or '(none)'}\n")
        text.append("Baud    ", style="dim")
        text.append(f"{info.baud}\n")
        text.append("Proto   ", style="dim")
        text.append(f"v{info.proto_version}\n")
        text.append("Features", style="dim")
        text.append(f" 0x{info.feature_bits:04X}\n")
        text.append("Uptime  ", style="dim")
        text.append(f"{hours}h {minutes:02d}m {secs:02d}s")
        self.update(text)


class ComputedStatsPanel(Static):
    """Shows computed frame rates, event rates, and bio acceptance rate."""

    DEFAULT_CSS = """
    ComputedStatsPanel {
        height: auto;
        padding: 0 1;
        border: solid $surface-lighten-2;
        border-title-color: $text;
    }
    """

    def on_mount(self) -> None:
        """Set the border title after mount."""
        self.border_title = "Stats"
        self._render_stats(FrameStats(), EventRates(), bio_acceptance_rate=0.0)

    def update_stats(
        self,
        frame_stats: FrameStats,
        event_rates: EventRates,
        bio_acceptance_rate: float = 0.0,
    ) -> None:
        """Refresh displayed computed statistics."""
        self._render_stats(frame_stats, event_rates, bio_acceptance_rate)

    def _render_stats(
        self,
        frame_stats: FrameStats,
        event_rates: EventRates,
        bio_acceptance_rate: float,
    ) -> None:
        text = Text()
        # Frame throughput
        text.append("Frames/sec ", style="dim")
        text.append(f"{frame_stats.frames_per_sec:.1f}\n")
        text.append("Bytes/sec  ", style="dim")
        text.append(f"{frame_stats.bytes_per_sec:.0f}\n")
        text.append("\n")

        # Event rate breakdown
        text.append("Event rates (per sec)\n", style="bold")
        text.append("  state   ", style="dim")
        text.append(f"{event_rates.state:.2f}\n")
        text.append("  bio     ", style="dim")
        text.append(f"{event_rates.bio:.2f}\n")
        text.append("  targets ", style="dim")
        text.append(f"{event_rates.targets:.2f}\n")
        text.append("  light   ", style="dim")
        text.append(f"{event_rates.light:.2f}\n")
        text.append("  diag    ", style="dim")
        text.append(f"{event_rates.diag:.2f}\n")
        text.append("\n")

        # Bio acceptance
        text.append("Bio accept ", style="dim")
        pct = bio_acceptance_rate * 100
        style = "green" if pct >= 50 else ("yellow" if pct >= 20 else "red")
        text.append(f"{pct:.0f}%", style=style)
        self.update(text)


# ---------------------------------------------------------------------------
# Main panel container
# ---------------------------------------------------------------------------


class DiagPanel(Widget):
    """Diagnostics tab content: protocol log, counters, connection info, stats.

    Public API:
        - ``append_protocol_event(event_type, seq, detail)`` — add to log
        - ``refresh_data(data)`` — update all static panels from DiagData
    """

    DEFAULT_CSS = """
    DiagPanel {
        layout: grid;
        grid-size: 2 2;
        grid-columns: 1fr 1fr;
        grid-rows: 1fr auto;
        height: 1fr;
        width: 1fr;
    }

    DiagPanel > ProtocolLog {
        column-span: 1;
        row-span: 2;
    }

    DiagPanel > #diag-right {
        column-span: 1;
        row-span: 2;
        height: 1fr;
    }
    """

    def compose(self) -> ComposeResult:
        """Build the diagnostics layout: log on left, info panels stacked on right."""
        yield ProtocolLog(id="diag-protocol-log")
        with Vertical(id="diag-right"):
            yield ConnectionPanel(id="diag-connection")
            yield CountersPanel(id="diag-counters")
            yield ComputedStatsPanel(id="diag-stats")

    def append_protocol_event(self, event_type: str, seq: int, detail: str) -> None:
        """Add a protocol event (ack/err/pong/hello) to the scrolling log."""
        log = self.query_one("#diag-protocol-log", ProtocolLog)
        log.append_event(event_type, seq, detail)

    def refresh_data(self, data: DiagData) -> None:
        """Update all static panels from a DiagData snapshot."""
        counters_panel = self.query_one("#diag-counters", CountersPanel)
        counters_panel.update_counters(data.counters, data.frame_stats.bad_frame_count)

        connection_panel = self.query_one("#diag-connection", ConnectionPanel)
        connection_panel.update_info(data.connection, data.uptime_s)

        stats_panel = self.query_one("#diag-stats", ComputedStatsPanel)
        stats_panel.update_stats(data.frame_stats, data.event_rates, data.bio_acceptance_rate)
