"""Textual TUI for mmWave sensor monitoring.

Provides a real-time dashboard with state display, radar view, vitals
charts, and scrolling event log.  Launched via ``mmwave_decode.py --format tui``
(default) or directly as a module.

The app uses mock/placeholder data until wired to the serial reader
in Task 12.
"""

from __future__ import annotations

from widgets import LogPanel, RadarPanel, StatePanel, VitalsPanel
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Header, Static, TabPane, TabbedContent
from textual.containers import Horizontal


class MmwaveMonitorApp(App):
    """mmWave sensor monitor TUI."""

    TITLE = "mmWave Monitor"

    CSS = """
    Screen {
        background: $surface;
    }

    #main-top-row {
        width: 100%;
        height: auto;
        min-height: 8;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("l", "toggle_log", "Toggle Log"),
        Binding("f", "toggle_firehose", "Firehose/Notable"),
        Binding("d", "show_diag", "Diagnostics"),
        Binding("m", "show_main", "Main"),
        Binding("tab", "next_tab", "Next Tab", show=False),
        Binding("question_mark", "show_help", "Help"),
    ]

    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        filter_types: set[str] | None = None,
    ) -> None:
        """Create the monitor app.

        Parameters
        ----------
        port:
            Serial port path (None for mock/demo mode).
        baud:
            Serial baud rate.
        filter_types:
            If set, only these event types appear in the log panel.

        """
        super().__init__()
        self._port = port
        self._baud = baud
        self._filter_types = filter_types

    def compose(self) -> ComposeResult:
        """Build the app layout with Main and Diag tabs."""
        yield Header()
        with TabbedContent(initial="main"):
            with TabPane("Main", id="main"):
                with Horizontal(id="main-top-row"):
                    yield StatePanel(id="state-panel")
                    yield RadarPanel(id="radar-panel")
                yield VitalsPanel(id="vitals-panel")
                yield LogPanel(
                    filter_types=self._filter_types,
                    id="log-panel",
                )
            with TabPane("Diag", id="diag"):
                yield Static("Diagnostics tab placeholder", id="diag-content")
        yield Footer()

    def on_mount(self) -> None:
        """Set header subtitle with connection info."""
        if self._port:
            self.sub_title = f"{self._port} @ {self._baud}"
        else:
            self.sub_title = "no sensor connected"

    # ----- Key actions --------------------------------------------------------

    def action_toggle_log(self) -> None:
        """Toggle log panel between collapsed and expanded."""
        try:
            log_panel = self.query_one("#log-panel", LogPanel)
            log_panel.toggle_expanded()
        except Exception:
            pass

    def action_toggle_firehose(self) -> None:
        """Toggle log between notable-only and firehose mode."""
        try:
            log_panel = self.query_one("#log-panel", LogPanel)
            log_panel.toggle_firehose()
        except Exception:
            pass

    def action_show_diag(self) -> None:
        """Switch to diagnostics tab."""
        self.query_one(TabbedContent).active = "diag"

    def action_show_main(self) -> None:
        """Switch to main tab."""
        self.query_one(TabbedContent).active = "main"

    def action_next_tab(self) -> None:
        """Cycle to the next tab."""
        tc = self.query_one(TabbedContent)
        if tc.active == "main":
            tc.active = "diag"
        else:
            tc.active = "main"

    def action_show_help(self) -> None:
        """Show help overlay (placeholder)."""
        self.notify(
            "Keybindings: q=quit  l=log  f=firehose  d=diag  m=main  Tab=cycle  ?=help",
            title="Help",
            timeout=5,
        )


if __name__ == "__main__":
    app = MmwaveMonitorApp()
    app.run()
