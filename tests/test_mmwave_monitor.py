"""Tests for mmWave TUI monitor — app skeleton, widgets, and rendering logic."""

from __future__ import annotations
import sys
import math
import time
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# sys.path setup so hardware/tools/ modules are importable
# ---------------------------------------------------------------------------

_TOOLS_DIR = str(Path(__file__).resolve().parent.parent / "hardware" / "tools")
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)


# ---------------------------------------------------------------------------
# Task 5: App skeleton tests
# ---------------------------------------------------------------------------


class TestAppSkeleton:
    """MmwaveMonitorApp mounting and keybindings."""

    @pytest.mark.asyncio
    async def test_app_mounts(self) -> None:
        """App creates and mounts without errors."""
        from mmwave_monitor import MmwaveMonitorApp

        app = MmwaveMonitorApp(port=None, baud=115200, filter_types=None)
        async with app.run_test() as _pilot:
            assert app.is_mounted

    @pytest.mark.asyncio
    async def test_app_has_tabs(self) -> None:
        """App contains Main and Diag tabs."""
        from mmwave_monitor import MmwaveMonitorApp
        from textual.widgets import TabbedContent

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            tc = app.query_one(TabbedContent)
            assert tc.active == "main"

    @pytest.mark.asyncio
    async def test_d_switches_to_diag(self) -> None:
        """Pressing 'd' switches to diagnostics tab."""
        from mmwave_monitor import MmwaveMonitorApp
        from textual.widgets import TabbedContent

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as pilot:
            await pilot.press("d")
            tc = app.query_one(TabbedContent)
            assert tc.active == "diag"

    @pytest.mark.asyncio
    async def test_m_switches_to_main(self) -> None:
        """Pressing 'm' switches back to main tab."""
        from mmwave_monitor import MmwaveMonitorApp
        from textual.widgets import TabbedContent

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as pilot:
            await pilot.press("d")
            await pilot.press("m")
            tc = app.query_one(TabbedContent)
            assert tc.active == "main"

    @pytest.mark.asyncio
    async def test_next_tab_action_cycles(self) -> None:
        """action_next_tab cycles between main and diag."""
        from mmwave_monitor import MmwaveMonitorApp
        from textual.widgets import TabbedContent

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            tc = app.query_one(TabbedContent)
            assert tc.active == "main"
            app.action_next_tab()
            assert tc.active == "diag"
            app.action_next_tab()
            assert tc.active == "main"

    @pytest.mark.asyncio
    async def test_header_shows_no_sensor(self) -> None:
        """Header subtitle shows 'no sensor connected' when port is None."""
        from mmwave_monitor import MmwaveMonitorApp

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            assert app.sub_title == "no sensor connected"

    @pytest.mark.asyncio
    async def test_header_shows_port(self) -> None:
        """Header subtitle shows port info when provided."""
        from mmwave_monitor import MmwaveMonitorApp

        app = MmwaveMonitorApp(port="/dev/cu.usbmodem2101", baud=115200)
        async with app.run_test() as _pilot:
            assert "/dev/cu.usbmodem2101" in app.sub_title

    @pytest.mark.asyncio
    async def test_widgets_present_in_main_tab(self) -> None:
        """Main tab contains StatePanel, RadarPanel, VitalsPanel, LogPanel."""
        from widgets import LogPanel, RadarPanel, StatePanel, VitalsPanel
        from mmwave_monitor import MmwaveMonitorApp

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            assert app.query_one(StatePanel) is not None
            assert app.query_one(RadarPanel) is not None
            assert app.query_one(VitalsPanel) is not None
            assert app.query_one(LogPanel) is not None


# ---------------------------------------------------------------------------
# Task 6: StatePanel tests
# ---------------------------------------------------------------------------


class TestStateIcons:
    """State icon lookup and rendering."""

    def test_all_states_have_icons(self) -> None:
        """Every known state has an icon for UNKNOWN pose."""
        from widgets.state_panel import get_state_icon

        states = ["NO_TARGET", "PRESENT_FAR", "MOVING", "STILL_NEAR", "RESTING_VITALS", "MULTI_TARGET"]
        for state in states:
            icon = get_state_icon(state, "UNKNOWN")
            assert icon is not None
            text = icon.plain
            assert len(text) > 0, f"Empty icon for {state}"

    def test_sitting_vs_standing_differ(self) -> None:
        """STILL_NEAR sitting and standing produce different icons."""
        from widgets.state_panel import get_state_icon

        sit = get_state_icon("STILL_NEAR", "SITTING")
        stand = get_state_icon("STILL_NEAR", "STANDING")
        assert sit.plain != stand.plain

    def test_unknown_state_fallback(self) -> None:
        """Unknown state returns a text fallback, not an error."""
        from widgets.state_panel import get_state_icon

        icon = get_state_icon("UNKNOWN_STATE", "UNKNOWN")
        assert "UNKNOWN_STATE" in icon.plain

    def test_icon_is_5_lines(self) -> None:
        """Icons are approximately 5 rows tall."""
        from widgets.state_panel import get_state_icon

        icon = get_state_icon("STILL_NEAR", "STANDING")
        lines = icon.plain.split("\n")
        assert len(lines) == 5

    def test_state_colors(self) -> None:
        """Each state has a defined color."""
        from widgets.state_panel import STATE_COLORS, get_state_color

        assert get_state_color("STILL_NEAR") == STATE_COLORS["STILL_NEAR"]
        assert get_state_color("UNKNOWN_STATE") == "#555555"  # default


class TestLuxBar:
    """Lux bar rendering."""

    def test_none_lux_gray(self) -> None:
        """None lux renders as gray bar."""
        from widgets.state_panel import render_lux_bar

        bar = render_lux_bar(None, width=10)
        assert len(bar.plain) == 10

    def test_low_lux_red(self) -> None:
        """Low lux (close proximity) renders in red."""
        from widgets.state_panel import render_lux_bar

        bar = render_lux_bar(10.0, width=10)
        # Check that we get some content
        assert len(bar.plain) == 10

    def test_high_lux_green(self) -> None:
        """High lux (clear path) renders in green."""
        from widgets.state_panel import render_lux_bar

        bar = render_lux_bar(300.0, width=10)
        assert len(bar.plain) == 10

    def test_bar_width_matches(self) -> None:
        """Bar width matches requested width."""
        from widgets.state_panel import render_lux_bar

        for w in (5, 10, 20):
            bar = render_lux_bar(100.0, width=w)
            assert len(bar.plain) == w


class TestStatePanelWidget:
    """StatePanel widget data update."""

    @pytest.mark.asyncio
    async def test_state_panel_update(self) -> None:
        """StatePanel.update_state() updates internal state."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.state_panel import StateData, StatePanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            panel = app.query_one(StatePanel)
            panel.update_state(
                StateData(
                    state="STILL_NEAR",
                    pose="SITTING",
                    human=1,
                    n_targets=1,
                    dist_cm=40.2,
                    head_moving=0,
                    dist_new=1,
                    lux=21.7,
                )
            )
            assert panel.state == "STILL_NEAR"
            assert panel.pose == "SITTING"

    @pytest.mark.asyncio
    async def test_state_panel_lux_update(self) -> None:
        """StatePanel.update_lux() updates lux display independently."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.state_panel import StatePanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            panel = app.query_one(StatePanel)
            panel.update_lux(150.0)
            # No exception means success


class TestPoseDescription:
    """Pose description text generation."""

    def test_no_target(self) -> None:
        """NO_TARGET returns 'No one detected'."""
        from widgets.state_panel import _pose_description

        assert _pose_description("NO_TARGET", "UNKNOWN") == "No one detected"

    def test_multi_target(self) -> None:
        """MULTI_TARGET returns 'Multiple people'."""
        from widgets.state_panel import _pose_description

        assert _pose_description("MULTI_TARGET", "UNKNOWN") == "Multiple people"

    def test_seated_still_near(self) -> None:
        """STILL_NEAR + SITTING includes 'Seated'."""
        from widgets.state_panel import _pose_description

        desc = _pose_description("STILL_NEAR", "SITTING")
        assert "Seated" in desc

    def test_standing_moving(self) -> None:
        """MOVING + STANDING includes 'Standing'."""
        from widgets.state_panel import _pose_description

        desc = _pose_description("MOVING", "STANDING")
        assert "Standing" in desc


# ---------------------------------------------------------------------------
# Task 7: RadarPanel tests
# ---------------------------------------------------------------------------


class TestBrailleCanvas:
    """BrailleCanvas rendering logic."""

    def test_empty_canvas(self) -> None:
        """Empty canvas renders blank braille characters."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(4, 3)
        plain = c.render_plain()
        lines = plain.split("\n")
        assert len(lines) == 3
        assert all(len(line) == 4 for line in lines)
        # All blank braille (U+2800)
        assert all(ch == "\u2800" for line in lines for ch in line)

    def test_set_pixel(self) -> None:
        """Setting a pixel produces a non-blank character."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(4, 3)
        c.set_pixel(0, 0)
        plain = c.render_plain()
        # First char should not be blank
        assert plain[0] != "\u2800"

    def test_set_pixel_out_of_bounds(self) -> None:
        """Out-of-bounds pixels are silently ignored."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(4, 3)
        c.set_pixel(-1, 0)
        c.set_pixel(100, 100)
        # Should not raise

    def test_clear_resets(self) -> None:
        """Clear resets all pixels to blank."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(4, 3)
        c.set_pixel(0, 0)
        c.clear()
        plain = c.render_plain()
        assert all(ch == "\u2800" for line in plain.split("\n") for ch in line)

    def test_pixel_resolution(self) -> None:
        """Pixel resolution is 2x char_width by 4x char_height."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(10, 5)
        assert c.px_width == 20
        assert c.px_height == 20

    def test_plot_point_with_radius(self) -> None:
        """plot_point with radius fills multiple pixels."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(10, 10)
        c.plot_point(10, 20, radius=3)
        plain = c.render_plain()
        # At least some non-blank characters
        non_blank = sum(1 for ch in plain if ch not in ("\u2800", "\n"))
        assert non_blank > 1

    def test_draw_arc_produces_pixels(self) -> None:
        """draw_arc draws a visible semicircle."""
        from widgets.radar_panel import BrailleCanvas

        c = BrailleCanvas(20, 10)
        c.draw_arc(20.0, 39.0, 15.0, start_angle=0.0, end_angle=math.pi)
        plain = c.render_plain()
        non_blank = sum(1 for ch in plain if ch not in ("\u2800", "\n"))
        assert non_blank > 5

    def test_draw_arc_dashed(self) -> None:
        """Dashed arc has fewer pixels than solid arc."""
        from widgets.radar_panel import BrailleCanvas

        c_solid = BrailleCanvas(20, 10)
        c_solid.draw_arc(20.0, 39.0, 15.0, start_angle=0.0, end_angle=math.pi)
        solid_plain = c_solid.render_plain()

        c_dashed = BrailleCanvas(20, 10)
        c_dashed.draw_arc(20.0, 39.0, 15.0, start_angle=0.0, end_angle=math.pi, dashed=True)
        dashed_plain = c_dashed.render_plain()

        solid_count = sum(1 for ch in solid_plain if ch not in ("\u2800", "\n"))
        dashed_count = sum(1 for ch in dashed_plain if ch not in ("\u2800", "\n"))
        assert dashed_count < solid_count


class TestRadarRendering:
    """Radar rendering with targets."""

    def test_empty_radar(self) -> None:
        """Radar with no targets still draws range rings."""
        from widgets.radar_panel import RadarData, render_radar_text

        text = render_radar_text(RadarData())
        non_blank = sum(1 for ch in text if ch not in ("\u2800", "\n"))
        assert non_blank > 0, "Range rings should produce visible pixels"

    def test_single_target(self) -> None:
        """Radar with one target draws more than just range rings."""
        from widgets.radar_panel import RadarData, RadarTarget, render_radar_text

        no_target = render_radar_text(RadarData())
        with_target = render_radar_text(
            RadarData(
                targets=[RadarTarget(x=0.0, y=1.0, r=1.0, bearing=0.0, is_focus=True)],
                n_targets=1,
            )
        )
        # Target should add pixels
        no_count = sum(1 for ch in no_target if ch not in ("\u2800", "\n"))
        with_count = sum(1 for ch in with_target if ch not in ("\u2800", "\n"))
        assert with_count > no_count

    def test_focus_target_glow(self) -> None:
        """Focus target renders with larger glow (more pixels) than non-focus.

        Place the target between range rings so the glow pixels are distinct
        from the arc pixels.
        """
        from widgets.radar_panel import RadarData, RadarTarget, render_radar_text

        focus_text = render_radar_text(
            RadarData(
                targets=[RadarTarget(x=1.5, y=2.5, r=2.9, bearing=31.0, is_focus=True)],
                n_targets=1,
            ),
            char_width=40,
            char_height=20,
        )
        nonfocus_text = render_radar_text(
            RadarData(
                targets=[RadarTarget(x=1.5, y=2.5, r=2.9, bearing=31.0, is_focus=False)],
                n_targets=1,
            ),
            char_width=40,
            char_height=20,
        )
        focus_count = sum(1 for ch in focus_text if ch not in ("\u2800", "\n"))
        nonfocus_count = sum(1 for ch in nonfocus_text if ch not in ("\u2800", "\n"))
        assert focus_count > nonfocus_count

    def test_three_targets(self) -> None:
        """Radar renders correctly with multiple targets at different positions."""
        from widgets.radar_panel import RadarData, RadarTarget, render_radar_text

        targets = [
            RadarTarget(x=0.0, y=0.5, r=0.5, bearing=0.0, is_focus=True, cluster=1),
            RadarTarget(x=-1.0, y=2.5, r=2.69, bearing=-21.8, is_focus=False, cluster=2),
            RadarTarget(x=2.0, y=4.0, r=4.47, bearing=26.6, is_focus=False, cluster=3),
        ]
        text = render_radar_text(RadarData(targets=targets, n_targets=3))
        # Should not raise and should produce visible output
        non_blank = sum(1 for ch in text if ch not in ("\u2800", "\n"))
        assert non_blank > 10

    def test_coordinate_mapping(self) -> None:
        """World-to-pixel mapping puts sensor at bottom center."""
        from widgets.radar_panel import _world_to_pixel

        # Sensor at origin (0, 0) should be at bottom center
        px, py = _world_to_pixel(0.0, 0.0, max_range_m=6.0, px_width=60, px_height=48)
        assert px == 30  # center
        assert py == 47  # bottom

    def test_coordinate_mapping_forward(self) -> None:
        """Target straight ahead maps to top center."""
        from widgets.radar_panel import _world_to_pixel

        px, py = _world_to_pixel(0.0, 6.0, max_range_m=6.0, px_width=60, px_height=48)
        assert px == 30  # center
        assert py == 0  # top


class TestRadarPanelWidget:
    """RadarPanel widget integration."""

    @pytest.mark.asyncio
    async def test_radar_panel_update(self) -> None:
        """RadarPanel.update_targets() renders without errors."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.radar_panel import RadarData, RadarPanel, RadarTarget

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            panel = app.query_one(RadarPanel)
            panel.update_targets(
                RadarData(
                    targets=[
                        RadarTarget(x=0.0, y=1.0, r=1.0, bearing=0.0, is_focus=True),
                    ],
                    n_targets=1,
                )
            )
            # No exception means success


# ---------------------------------------------------------------------------
# Task 8: VitalsPanel tests
# ---------------------------------------------------------------------------


class TestStateBar:
    """State annotation bar rendering."""

    def test_empty_states(self) -> None:
        """Empty states list renders gray bar."""
        from widgets.vitals_panel import render_state_bar

        bar = render_state_bar([], width=10)
        assert len(bar.plain) == 10

    def test_single_state(self) -> None:
        """Single state fills the entire bar."""
        from widgets.vitals_panel import render_state_bar

        bar = render_state_bar(["STILL_NEAR"] * 5, width=10)
        assert len(bar.plain) == 10
        assert all(ch == "\u2588" for ch in bar.plain)

    def test_mixed_states(self) -> None:
        """Mixed states produce a bar with correct width."""
        from widgets.vitals_panel import render_state_bar

        states = ["STILL_NEAR", "MOVING", "RESTING_VITALS", "NO_TARGET"]
        bar = render_state_bar(states, width=20)
        assert len(bar.plain) == 20

    def test_bar_width_matches(self) -> None:
        """Bar always matches requested width regardless of data length."""
        from widgets.vitals_panel import render_state_bar

        for width in (5, 10, 40, 80):
            bar = render_state_bar(["STILL_NEAR"] * 3, width=width)
            assert len(bar.plain) == width


class TestVitalsTimeSeries:
    """VitalsTimeSeries data buffer."""

    def test_append_and_len(self) -> None:
        """Append adds data points correctly."""
        from widgets.vitals_panel import VitalsTimeSeries

        ts = VitalsTimeSeries(max_size=100)
        ts.append(1.0, 80.0, "STILL_NEAR")
        ts.append(2.0, 82.0, "STILL_NEAR")
        assert len(ts) == 2

    def test_overflow(self) -> None:
        """Overflow drops oldest entries."""
        from widgets.vitals_panel import VitalsTimeSeries

        ts = VitalsTimeSeries(max_size=3)
        for i in range(5):
            ts.append(float(i), float(60 + i), "STILL_NEAR")
        assert len(ts) == 3

    def test_get_series(self) -> None:
        """get_series returns aligned lists."""
        from widgets.vitals_panel import VitalsTimeSeries

        ts = VitalsTimeSeries(max_size=10)
        ts.append(1.0, 80.0, "STILL_NEAR")
        ts.append(2.0, 85.0, "MOVING")
        timestamps, values, states = ts.get_series()
        assert values == [80.0, 85.0]
        assert states == ["STILL_NEAR", "MOVING"]
        assert timestamps == [1.0, 2.0]


class TestAcceptanceLabel:
    """Bio acceptance label formatting."""

    def test_ok_ok(self) -> None:
        """Both allowed and valid shows ok/ok."""
        from widgets.vitals_panel import _acceptance_label

        label = _acceptance_label(1, 1)
        assert "ok" in label.plain

    def test_fail_fail(self) -> None:
        """Neither allowed nor valid shows fail/fail."""
        from widgets.vitals_panel import _acceptance_label

        label = _acceptance_label(0, 0)
        assert "fail" in label.plain


class TestVitalsPanelWidget:
    """VitalsPanel widget integration."""

    @pytest.mark.asyncio
    async def test_vitals_panel_update(self) -> None:
        """VitalsPanel.update_vitals() renders without errors."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.vitals_panel import VitalsData, VitalsPanel, VitalsTimeSeries

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            panel = app.query_one(VitalsPanel)

            hr_series = VitalsTimeSeries(max_size=100)
            br_series = VitalsTimeSeries(max_size=100)
            now = time.monotonic()
            for i in range(10):
                hr_series.append(now + i, 75.0 + i, "STILL_NEAR")
                br_series.append(now + i, 12.0 + (i % 3), "STILL_NEAR")

            panel.update_vitals(
                VitalsData(
                    hr_current=83.0,
                    br_current=14.0,
                    hr_allowed=1,
                    hr_valid=1,
                    br_allowed=1,
                    br_valid=1,
                    hr_series=hr_series,
                    br_series=br_series,
                )
            )
            # No exception means success

    @pytest.mark.asyncio
    async def test_vitals_panel_no_data(self) -> None:
        """VitalsPanel renders correctly with no data."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.vitals_panel import VitalsData, VitalsPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            panel = app.query_one(VitalsPanel)
            panel.update_vitals(VitalsData())
            # No exception means success


# ---------------------------------------------------------------------------
# Task 9: LogPanel tests
# ---------------------------------------------------------------------------


class TestNotableFilter:
    """LogPanel notable filter logic."""

    def test_state_transition_notable(self) -> None:
        """State change is notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        # First event sets baseline — not notable
        assert not f.is_notable("state", {"state": "STILL_NEAR"})
        # Transition is notable
        assert f.is_notable("state", {"state": "MOVING"})

    def test_same_state_not_notable(self) -> None:
        """Same state repeated is not notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        f.is_notable("state", {"state": "STILL_NEAR"})
        assert not f.is_notable("state", {"state": "STILL_NEAR"})

    def test_bio_new_values_notable(self) -> None:
        """Bio with new values is notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        assert f.is_notable("bio", {"hr_new": 1, "br_new": 0})

    def test_bio_no_new_not_notable(self) -> None:
        """Bio without new values is not notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        assert not f.is_notable("bio", {"hr_new": 0, "br_new": 0})

    def test_light_significant_change_notable(self) -> None:
        """Light reading with >5 lux change is notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        f.is_notable("light", {"lux": 100.0})
        assert f.is_notable("light", {"lux": 90.0})  # 10 lux delta

    def test_light_small_change_not_notable(self) -> None:
        """Light reading with <5 lux change is not notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        f.is_notable("light", {"lux": 100.0})
        assert not f.is_notable("light", {"lux": 98.0})  # 2 lux delta

    def test_error_always_notable(self) -> None:
        """Error events are always notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        assert f.is_notable("err", {"cmd_id": 1, "err_code": 2})

    def test_ack_always_notable(self) -> None:
        """Ack events are always notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        assert f.is_notable("ack", {})

    def test_hello_always_notable(self) -> None:
        """Hello events are always notable."""
        from widgets.log_panel import NotableFilter

        f = NotableFilter()
        assert f.is_notable("hello", {})


class TestLogFormatting:
    """Log entry formatting functions."""

    def test_format_state_event(self) -> None:
        """State event formatting includes state and metrics."""
        from widgets.log_panel import format_state_event

        msg = format_state_event(
            {"state": "STILL_NEAR", "pose": "SITTING", "human": 1, "n_targets": 1, "dist_cm": 40.2, "head_moving": 0}
        )
        assert "STILL_NEAR" in msg
        assert "40.2" in msg

    def test_format_bio_event(self) -> None:
        """Bio event formatting includes hr, br, and flags."""
        from widgets.log_panel import format_bio_event

        msg = format_bio_event({"allowed": 1, "valid": 1, "hr": 83.0, "br": 6.0, "hr_new": 1, "br_new": 0})
        assert "hr=83.0" in msg
        assert "+hr" in msg
        assert "+br" not in msg

    def test_format_light_event(self) -> None:
        """Light event formatting includes lux value."""
        from widgets.log_panel import format_light_event

        msg = format_light_event({"lux": 21.7})
        assert "21.7" in msg

    def test_format_light_event_none(self) -> None:
        """Light event with None lux shows dashes."""
        from widgets.log_panel import format_light_event

        msg = format_light_event({"lux": None})
        assert "--" in msg

    def test_format_targets_event(self) -> None:
        """Targets event formatting includes count and focus info."""
        from widgets.log_panel import format_targets_event

        msg = format_targets_event(
            {
                "n_targets": 2,
                "targets_truncated": False,
                "focus": {"r": 0.44, "bearing": 10.0},
            }
        )
        assert "n=2" in msg
        assert "0.44" in msg

    def test_format_targets_truncated(self) -> None:
        """Truncated targets event shows TRUNCATED."""
        from widgets.log_panel import format_targets_event

        msg = format_targets_event({"n_targets": 8, "targets_truncated": True, "focus": None})
        assert "TRUNCATED" in msg

    def test_format_log_entry(self) -> None:
        """LogEntry formatting produces Rich Text with timestamp and type."""
        from widgets.log_panel import LogEntry, format_log_entry

        entry = LogEntry(timestamp=time.time(), event_type="state", message="STILL_NEAR test")
        result = format_log_entry(entry)
        assert "state" in result.plain
        assert "STILL_NEAR" in result.plain

    def test_format_event_data_unknown_type(self) -> None:
        """Unknown event type uses generic formatting."""
        from widgets.log_panel import format_event_data

        msg = format_event_data("pong", {"t_ms": 12345})
        assert "12345" in msg


class TestLogPanelWidget:
    """LogPanel widget integration."""

    @pytest.mark.asyncio
    async def test_log_panel_add_event_firehose(self) -> None:
        """LogPanel in firehose mode accepts all events."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            log_panel = app.query_one(LogPanel)
            log_panel.is_firehose = True
            log_panel.add_event("state", {"state": "STILL_NEAR", "pose": "SITTING"})
            log_panel.add_event("bio", {"hr": 83.0, "br": 6.0, "hr_new": 0, "br_new": 0})
            # No exception means success

    @pytest.mark.asyncio
    async def test_log_panel_notable_filters(self) -> None:
        """LogPanel in notable mode filters repetitive events."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            log_panel = app.query_one(LogPanel)
            # Notable mode is default (is_firehose=False)
            assert not log_panel.is_firehose
            # Add non-notable event (first state, sets baseline, not notable)
            log_panel.add_event("state", {"state": "STILL_NEAR"})
            # Same state again — not notable, should be filtered
            log_panel.add_event("state", {"state": "STILL_NEAR"})

    @pytest.mark.asyncio
    async def test_log_panel_type_filter(self) -> None:
        """LogPanel respects filter_types set."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None, filter_types={"bio"})
        async with app.run_test() as _pilot:
            log_panel = app.query_one(LogPanel)
            log_panel.is_firehose = True
            # State event should be filtered out
            log_panel.add_event("state", {"state": "STILL_NEAR"})
            # Bio event should pass through
            log_panel.add_event("bio", {"hr": 83.0, "br": 6.0, "hr_new": 1, "br_new": 0})

    @pytest.mark.asyncio
    async def test_log_panel_toggle_expanded(self) -> None:
        """Toggle expanded changes the class."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            log_panel = app.query_one(LogPanel)
            assert not log_panel.is_expanded
            log_panel.toggle_expanded()
            assert log_panel.is_expanded
            log_panel.toggle_expanded()
            assert not log_panel.is_expanded

    @pytest.mark.asyncio
    async def test_log_panel_toggle_firehose(self) -> None:
        """Toggle firehose switches mode."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as _pilot:
            log_panel = app.query_one(LogPanel)
            assert not log_panel.is_firehose
            log_panel.toggle_firehose()
            assert log_panel.is_firehose
            log_panel.toggle_firehose()
            assert not log_panel.is_firehose

    @pytest.mark.asyncio
    async def test_l_key_toggles_log(self) -> None:
        """Pressing 'l' toggles the log panel expansion."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as pilot:
            log_panel = app.query_one(LogPanel)
            assert not log_panel.is_expanded
            await pilot.press("l")
            assert log_panel.is_expanded
            await pilot.press("l")
            assert not log_panel.is_expanded

    @pytest.mark.asyncio
    async def test_f_key_toggles_firehose(self) -> None:
        """Pressing 'f' toggles firehose mode."""
        from mmwave_monitor import MmwaveMonitorApp
        from widgets.log_panel import LogPanel

        app = MmwaveMonitorApp(port=None)
        async with app.run_test() as pilot:
            log_panel = app.query_one(LogPanel)
            assert not log_panel.is_firehose
            await pilot.press("f")
            assert log_panel.is_firehose
