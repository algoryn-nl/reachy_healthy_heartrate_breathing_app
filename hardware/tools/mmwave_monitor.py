"""Textual TUI for mmWave sensor monitoring.

Provides a real-time dashboard with state display, radar view, vitals
charts, and scrolling event log.  Launched via ``mmwave_decode.py --format tui``
(default) or directly as a module.
"""

from __future__ import annotations
import math
import time
import asyncio
from pathlib import Path
from dataclasses import asdict

from widgets import LogPanel, DiagPanel, RadarPanel, StatePanel, VitalsPanel
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Header, Static, TabPane, TabbedContent
from textual.containers import Horizontal
from widgets.diag_panel import DiagData
from widgets.diag_panel import EventRates as DiagEventRates
from widgets.diag_panel import FrameStats as DiagFrameStats
from widgets.diag_panel import DiagCounters as DiagDiagCounters
from widgets.diag_panel import ConnectionInfo as DiagConnectionInfo
from widgets.radar_panel import RadarData, RadarTarget
from widgets.state_panel import StateData
from widgets.vitals_panel import VitalsData

from healthy_heartrate_breathing.sensor_models import (
    BioReading,
    EventRates,
    FrameStats,
    DecodedEvent,
    DiagCounters,
    LightReading,
    TargetsEvent,
    SensorSnapshot,
    TargetSmoother,
    BioAcceptanceTracker,
    event_from_dict,
)


_HELP_TEXT = """\
[bold]LEGEND[/bold]

[bold]Device States[/bold]
  [#555555]NO_TARGET[/]       Nobody detected
  [#fbbf24]PRESENT_FAR[/]     Person outside vitals zone (35–150 cm)
  [#fbbf24]MOVING[/]          Person or robot head in motion
  [#4ade80]STILL_NEAR[/]      In vitals zone, still, warming up
  [#60a5fa]RESTING_VITALS[/]  Vitals confirmed — measuring
  [#a78bfa]MULTI_TARGET[/]    Multiple people (vitals unreliable)

[bold]Vitals Header[/bold]
  [dim]gate:[/]  ✓ conditions allow vitals (single target, head still)
         ✗ blocked (moving, multi-target, or no target)
  [dim]qual:[/]  ✓ HR and BR both pass guard-rail validation
         ✗ one or both rates outside physiological range

  [bold]HR[/bold]  Heart Rate     [dim]unit:[/] bpm (beats per minute)
  [bold]BR[/bold]  Breathing Rate [dim]unit:[/] rpm (respirations per minute)

[bold]Radar[/bold]
  [#4ade80]●[/]  Target < 1 m    [#a78bfa]●[/]  Focus target
  [#fbbf24]●[/]  Target 1–3 m    [#fb923c]●[/]  Target > 3 m
  [#c084fc]Dashed arcs[/] = vitals zone (0.35–1.5 m)
  [#888888]Solid arcs[/]  = range rings
  θ  Bearing angle from sensor boresight

[bold]State Bar[/bold]  (under vitals charts)
  Colored blocks show device state over time, aligned to chart x-axis.
  Each block = one time bucket. Current state name shown at the end.
  [#4ade80]████[/] STILL_NEAR   [#60a5fa]████[/] RESTING_VITALS
  [#fbbf24]████[/] MOVING/FAR   [#a78bfa]████[/] MULTI_TARGET
  [#555555]████[/] NO_TARGET

[bold]Keys[/bold]
  [bold]q[/]  Quit    [bold]l[/]  Toggle log    [bold]f[/]  Firehose / notable
  [bold]d[/]  Diag    [bold]m[/]  Main          [bold]?[/]  Help
  [bold]r[/]  Retry   [bold]Tab[/]  Cycle tabs
"""


class MmwaveMonitorApp(App):
    """mmWave sensor monitor TUI."""

    TITLE = "mmWave Monitor"

    CSS = """
    Screen {
        background: $surface;
    }

    TabbedContent {
        height: 1fr;
    }

    TabPane {
        height: 1fr;
    }

    #main-top-row {
        width: 100%;
        height: 1fr;
    }

    #state-panel {
        width: 1fr;
        min-width: 28;
        height: 100%;
    }

    #radar-panel {
        width: 2fr;
        min-width: 40;
        height: 100%;
        overflow-y: hidden;
    }

    #vitals-panel {
        height: auto;
        max-height: 16;
        min-height: 8;
    }

    #log-panel {
        height: auto;
        max-height: 6;
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
        Binding("r", "retry_connect", "Retry", show=False),
    ]

    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        filter_types: set[str] | None = None,
        input_file: Path | None = None,
    ) -> None:
        """Create monitor app with serial port or file replay."""
        super().__init__()
        self._port = port
        self._baud = baud
        self._filter_types = filter_types
        self._input_file = input_file
        # Tracking state
        self._event_rates = EventRates(window_s=10.0)
        self._frame_stats = FrameStats()
        self._bio_tracker = BioAcceptanceTracker()
        self._start_time = time.monotonic()
        self._last_state: str = "NO_TARGET"
        self._last_pose: str = ""
        self._last_dist_cm: float | None = None
        self._last_human: int = 0
        self._last_n_targets: int = 0
        self._vitals_data = VitalsData()
        self._target_smoother = TargetSmoother(alpha=0.35, stale_s=2.0)
        self._connected = False

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
                yield DiagPanel(id="diag-panel")
            with TabPane("Help", id="help"):
                yield Static(_HELP_TEXT, id="help-panel")
        yield Footer()

    def on_mount(self) -> None:
        """Set header subtitle and start serial reader."""
        if self._port:
            self.sub_title = f"{self._port} @ {self._baud}"
        elif self._input_file:
            self.sub_title = f"replay: {self._input_file.name}"
        else:
            self._try_auto_detect()
            if not self._port:
                self.sub_title = "no sensor — press r to retry"
                return

        self._start_reader()

    def _try_auto_detect(self) -> None:
        """Attempt serial port auto-detection."""
        try:
            import serial.tools.list_ports
        except ImportError:
            return
        for p in serial.tools.list_ports.comports():
            if p.vid == 0x303A and p.pid == 0x1001:
                self._port = p.device
                return
        import glob

        candidates = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*")
        if candidates:
            self._port = candidates[0]

    def _start_reader(self) -> None:
        """Launch the background serial reader worker."""
        if self._input_file:
            self.run_worker(self._file_reader_worker(), name="file-reader", exclusive=True)
        elif self._port:
            self.sub_title = f"{self._port} @ {self._baud}"
            self.run_worker(self._serial_reader_worker(), name="serial-reader", exclusive=True)

    async def _serial_reader_worker(self) -> None:
        """Background worker reading serial events."""
        from healthy_heartrate_breathing.sensor_models import read_events

        self._connected = True
        try:
            async for event in read_events(self._port, self._baud):
                self._handle_event(event)
        except Exception as exc:
            self._connected = False
            self.sub_title = f"disconnected: {exc}"

    async def _file_reader_worker(self) -> None:
        """Replay events from a capture file."""
        from healthy_heartrate_breathing.profiles._healthy_heartrate_breathing_locked_profile.mmwave_protocol import (
            PROTO_VERSION,
            ProtocolError,
            decode_event,
            decode_frame,
            extract_encoded_frames,
        )

        path = self._input_file
        assert path is not None
        raw = path.read_bytes()
        buf = bytearray(raw)
        seq = 0
        for encoded in extract_encoded_frames(buf):
            try:
                version, msg_type, _seq_wire, payload = decode_frame(encoded)
                if version != PROTO_VERSION:
                    continue
                raw_event = decode_event(msg_type, payload)
            except ProtocolError:
                self._frame_stats.bad_frame_count += 1
                continue
            event = event_from_dict(raw_event, seq=seq)
            self._handle_event(event)
            seq += 1
            await asyncio.sleep(0)  # yield to UI

        self.sub_title = f"replay complete: {path.name} ({seq} frames)"

    def _handle_event(self, event: DecodedEvent) -> None:
        """Route a decoded event to the appropriate widgets."""
        now = time.monotonic()
        self._event_rates.record(event.event_type, now)
        self._frame_stats.frames_per_sec = self._event_rates.rate("_total", now)

        # State panel
        if event.event_type == "state" and isinstance(event.data, SensorSnapshot):
            snap = event.data
            self._last_state = snap.state
            self._last_pose = snap.pose
            self._last_dist_cm = snap.dist_cm
            self._last_human = snap.human
            self._last_n_targets = snap.n_targets
            try:
                panel = self.query_one("#state-panel", StatePanel)
                panel.update_state(
                    StateData(
                        state=snap.state,
                        pose=snap.pose,
                        human=snap.human,
                        n_targets=snap.n_targets,
                        dist_cm=snap.dist_cm,
                        head_moving=snap.head_moving,
                        dist_new=snap.dist_new,
                    )
                )
            except Exception:
                pass

        # Bio / vitals
        elif event.event_type == "bio" and isinstance(event.data, BioReading):
            bio = event.data
            self._bio_tracker.record(bio.allowed, bio.valid)
            vd = self._vitals_data
            if bio.hr is not None and bio.hr_new:
                vd.hr_current = bio.hr
                vd.hr_series.append(now, bio.hr, self._last_state)
            if bio.br is not None and bio.br_new:
                vd.br_current = bio.br
                vd.br_series.append(now, bio.br, self._last_state)
            vd.hr_allowed = bio.allowed
            vd.hr_valid = bio.valid
            vd.br_allowed = bio.allowed
            vd.br_valid = bio.valid
            try:
                panel = self.query_one("#vitals-panel", VitalsPanel)
                panel.update_vitals(vd)
            except Exception:
                pass

        # Light → state panel lux
        elif event.event_type == "light" and isinstance(event.data, LightReading):
            try:
                panel = self.query_one("#state-panel", StatePanel)
                panel.update_lux(event.data.lux)
            except Exception:
                pass

        # Targets → radar (with position smoothing)
        elif event.event_type == "targets" and isinstance(event.data, TargetsEvent):
            te = event.data
            smoother = self._target_smoother
            radar_targets = []
            for t in te.targets:
                sx, sy = smoother.smooth(t.cluster, t.x, t.y, now)
                sr = math.sqrt(sx * sx + sy * sy)
                sb = math.degrees(math.atan2(sx, sy)) if sr > 0 else 0.0
                radar_targets.append(
                    RadarTarget(
                        x=sx,
                        y=sy,
                        r=sr,
                        bearing=sb,
                        is_focus=(te.focus is not None and t.cluster == te.focus.cluster),
                        cluster=t.cluster,
                    )
                )
            if te.focus and not any(rt.is_focus for rt in radar_targets):
                sx, sy = smoother.smooth(te.focus.cluster, te.focus.x, te.focus.y, now)
                sr = math.sqrt(sx * sx + sy * sy)
                sb = math.degrees(math.atan2(sx, sy)) if sr > 0 else 0.0
                radar_targets.append(
                    RadarTarget(
                        x=sx,
                        y=sy,
                        r=sr,
                        bearing=sb,
                        is_focus=True,
                        cluster=te.focus.cluster,
                    )
                )
            smoother.prune(now)
            try:
                panel = self.query_one("#radar-panel", RadarPanel)
                panel.update_targets(
                    RadarData(
                        targets=radar_targets,
                        n_targets=te.n_targets,
                        device_state=self._last_state,
                        device_pose=self._last_pose,
                        dist_cm=self._last_dist_cm,
                        human=self._last_human,
                    )
                )
            except Exception:
                pass

        # Diag counters
        elif event.event_type == "diag" and isinstance(event.data, DiagCounters):
            dc = event.data
            try:
                panel = self.query_one("#diag-panel", DiagPanel)
                panel.refresh_data(
                    DiagData(
                        counters=DiagDiagCounters(
                            mmwave_fail_count=dc.mmwave_fail_count,
                            mmwave_consecutive_fails=dc.mmwave_consecutive_fails,
                            tx_drop_count=dc.tx_drop_count,
                        ),
                        connection=DiagConnectionInfo(
                            port=self._port or "",
                            baud=self._baud,
                        ),
                        frame_stats=DiagFrameStats(
                            frames_per_sec=self._frame_stats.frames_per_sec,
                            bad_frame_count=self._frame_stats.bad_frame_count,
                        ),
                        event_rates=DiagEventRates(
                            state=self._event_rates.rate("state", now),
                            bio=self._event_rates.rate("bio", now),
                            targets=self._event_rates.rate("targets", now),
                            light=self._event_rates.rate("light", now),
                            diag=self._event_rates.rate("diag", now),
                        ),
                        bio_acceptance_rate=self._bio_tracker.acceptance_rate(),
                        uptime_s=now - self._start_time,
                    )
                )
            except Exception:
                pass

        # Protocol events → diag log
        if event.event_type in ("ack", "err", "pong", "hello"):
            detail = ""
            if isinstance(event.data, dict):
                detail = " ".join(f"{k}={v}" for k, v in event.data.items() if k != "type")
            try:
                panel = self.query_one("#diag-panel", DiagPanel)
                panel.append_protocol_event(event.event_type, event.seq, detail)
            except Exception:
                pass

        # Log panel (all events)
        log_data: dict[str, object] = {}
        if hasattr(event.data, "__dataclass_fields__"):
            log_data = asdict(event.data)  # type: ignore[arg-type]
        elif isinstance(event.data, dict):
            log_data = event.data
        try:
            panel = self.query_one("#log-panel", LogPanel)
            panel.add_event(event.event_type, log_data)
        except Exception:
            pass

    # ----- Key actions --------------------------------------------------------

    def action_toggle_log(self) -> None:
        """Toggle log panel between collapsed and expanded."""
        try:
            self.query_one("#log-panel", LogPanel).toggle_expanded()
        except Exception:
            pass

    def action_toggle_firehose(self) -> None:
        """Toggle log between notable-only and firehose mode."""
        try:
            self.query_one("#log-panel", LogPanel).toggle_firehose()
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
        order = ["main", "diag", "help"]
        idx = order.index(tc.active) if tc.active in order else 0
        tc.active = order[(idx + 1) % len(order)]

    def action_show_help(self) -> None:
        """Switch to the Help tab."""
        self.query_one(TabbedContent).active = "help"

    def action_retry_connect(self) -> None:
        """Retry serial port auto-detection."""
        self._try_auto_detect()
        if self._port:
            self._start_reader()
        else:
            self.notify("No sensor found", severity="warning")


if __name__ == "__main__":
    app = MmwaveMonitorApp()
    app.run()
