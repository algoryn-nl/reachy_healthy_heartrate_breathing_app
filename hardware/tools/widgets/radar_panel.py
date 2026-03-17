"""Radar panel widget — braille-character top-down radar view with target list.

Renders a semi-circle radar with range rings at 1m intervals (0-6m),
a dashed vitals zone arc at 0.35-1.5m, focus target highlighted in
purple with glow, other targets colored by distance.

Braille characters (Unicode U+2800-U+28FF) provide sub-character
resolution: each cell is 2 dots wide x 4 dots tall.

The widget layout is: radar canvas (left) + target/state info list (right).
"""

from __future__ import annotations
import math
from dataclasses import field, dataclass

from rich.text import Text
from textual.app import ComposeResult
from textual.widgets import Static
from textual.containers import Horizontal


# ---------------------------------------------------------------------------
# Braille canvas
# ---------------------------------------------------------------------------

_DOT_BITS: dict[tuple[int, int], int] = {
    (0, 0): 0x01,
    (1, 0): 0x08,
    (0, 1): 0x02,
    (1, 1): 0x10,
    (0, 2): 0x04,
    (1, 2): 0x20,
    (0, 3): 0x40,
    (1, 3): 0x80,
}

BRAILLE_BASE = 0x2800


class BrailleCanvas:
    """Pixel canvas rendered via braille characters.

    The pixel resolution is ``(char_width * 2, char_height * 4)`` because
    each braille character encodes a 2-wide x 4-tall dot matrix.
    """

    def __init__(self, char_width: int, char_height: int) -> None:
        """Create a canvas of *char_width* x *char_height* terminal cells."""
        self.char_width = char_width
        self.char_height = char_height
        self.px_width = char_width * 2
        self.px_height = char_height * 4
        self._grid: list[list[int]] = [[0] * char_width for _ in range(char_height)]

    def clear(self) -> None:
        """Reset all pixels."""
        for row in self._grid:
            for i in range(len(row)):
                row[i] = 0

    def set_pixel(self, px: int, py: int) -> None:
        """Set a single dot pixel at (px, py)."""
        if px < 0 or px >= self.px_width or py < 0 or py >= self.px_height:
            return
        cx = px // 2
        cy = py // 4
        dx = px % 2
        dy = py % 4
        self._grid[cy][cx] |= _DOT_BITS[(dx, dy)]

    def render_plain(self) -> str:
        """Return the canvas as a plain multi-line string of braille characters."""
        lines: list[str] = []
        for row in self._grid:
            lines.append("".join(chr(BRAILLE_BASE + v) for v in row))
        return "\n".join(lines)

    def get_char(self, cx: int, cy: int) -> str:
        """Return the braille character at character cell (cx, cy)."""
        if 0 <= cx < self.char_width and 0 <= cy < self.char_height:
            return chr(BRAILLE_BASE + self._grid[cy][cx])
        return " "

    def plot_point(self, px: int, py: int, radius: int = 0) -> None:
        """Plot a point, optionally with a filled radius for glow effect."""
        if radius <= 0:
            self.set_pixel(px, py)
            return
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    self.set_pixel(px + dx, py + dy)

    def draw_arc(
        self,
        cx: float,
        cy: float,
        radius_px: float,
        start_angle: float = 0.0,
        end_angle: float = math.pi,
        step: float = 0.02,
        dashed: bool = False,
    ) -> None:
        """Draw an arc from start_angle to end_angle (radians)."""
        theta = start_angle
        pixel_count = 0
        while theta <= end_angle:
            sx = cx + radius_px * math.cos(theta)
            sy = cy - radius_px * math.sin(theta)
            if not dashed or (pixel_count // 4) % 2 == 0:
                self.set_pixel(int(round(sx)), int(round(sy)))
            theta += step
            pixel_count += 1


# ---------------------------------------------------------------------------
# Target data
# ---------------------------------------------------------------------------


@dataclass
class RadarTarget:
    """Minimal target data for radar rendering."""

    x: float = 0.0
    y: float = 0.0
    r: float = 0.0
    bearing: float = 0.0
    is_focus: bool = False
    cluster: int = 0


@dataclass
class RadarData:
    """Snapshot of radar display data."""

    targets: list[RadarTarget] = field(default_factory=list)
    n_targets: int = 0
    max_range_m: float = 6.0
    vitals_inner_m: float = 0.35
    vitals_outer_m: float = 1.5
    device_state: str = ""
    device_pose: str = ""
    dist_cm: float | None = None
    human: int = 0

    @property
    def display_range_m(self) -> float:
        """Auto-zoom: pick a range that fits the farthest target with padding.

        Snaps to nice round values: 1, 1.5, 2, 3, 4, 6m.
        Falls back to max_range_m when no targets.
        """
        if not self.targets:
            return self.max_range_m
        farthest = max(t.r for t in self.targets)
        # Snap to next nice range with 30% padding
        padded = farthest * 1.3
        for nice in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
            if padded <= nice:
                return nice
        return self.max_range_m


# ---------------------------------------------------------------------------
# Coordinate mapping helpers
# ---------------------------------------------------------------------------


def _target_color(target: RadarTarget) -> str:
    """Color a target by distance: green=near, yellow=mid, orange=far."""
    if target.is_focus:
        return "#a78bfa"
    if target.r < 1.0:
        return "#4ade80"
    if target.r < 3.0:
        return "#fbbf24"
    return "#fb923c"


def _target_marker(target: RadarTarget) -> str:
    """Return a marker character for the target."""
    return "\u25cf" if target.is_focus else "\u2022"  # ● vs •


def _world_to_pixel(
    x_m: float,
    y_m: float,
    max_range_m: float,
    px_width: int,
    px_height: int,
) -> tuple[int, int]:
    """Convert world coordinates (meters) to pixel coords."""
    scale_x = px_width / (2.0 * max_range_m)
    scale_y = (px_height - 1) / max_range_m

    px = int(round(px_width / 2.0 + x_m * scale_x))
    py = int(round((px_height - 1) - y_m * scale_y))
    return px, py


def _world_to_char(
    x_m: float,
    y_m: float,
    max_range_m: float,
    char_width: int,
    char_height: int,
) -> tuple[int, int]:
    """Convert world coordinates to character cell position."""
    scale_x = char_width / (2.0 * max_range_m)
    scale_y = (char_height - 1) / max_range_m
    cx = int(round(char_width / 2.0 + x_m * scale_x))
    cy = int(round((char_height - 1) - y_m * scale_y))
    return cx, cy


def _nice_ring_step(display_range: float) -> float:
    """Pick a ring spacing that gives 3-5 rings for the display range."""
    if display_range <= 1.0:
        return 0.25
    if display_range <= 2.0:
        return 0.5
    if display_range <= 4.0:
        return 1.0
    return 2.0


def _range_to_radius_px(range_m: float, max_range_m: float, px_height: int) -> float:
    """Convert a range in meters to pixel radius for arc drawing."""
    return (range_m / max_range_m) * px_height


# ---------------------------------------------------------------------------
# Radar rendering (Rich Text with colors)
# ---------------------------------------------------------------------------


def render_radar_rich(
    data: RadarData,
    char_width: int,
    char_height: int,
) -> Text:
    """Render radar to Rich Text with colored targets overlaid on braille grid."""
    display_range = data.display_range_m
    canvas = BrailleCanvas(char_width, char_height)

    sensor_cx = canvas.px_width / 2.0
    sensor_cy = canvas.px_height - 1

    # Range rings — only show rings that fit in display range
    ring_step = _nice_ring_step(display_range)
    ring_m = ring_step
    while ring_m <= display_range:
        r_px = _range_to_radius_px(ring_m, display_range, canvas.px_height)
        canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi)
        ring_m += ring_step

    # Vitals zone arcs (dashed)
    for vz_m in (data.vitals_inner_m, data.vitals_outer_m):
        if vz_m <= display_range:
            r_px = _range_to_radius_px(vz_m, display_range, canvas.px_height)
            canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi, dashed=True)

    # Build target position map: char cell → (marker, color)
    target_cells: dict[tuple[int, int], tuple[str, str]] = {}
    for target in data.targets:
        cx, cy = _world_to_char(target.x, target.y, display_range, char_width, char_height)
        color = _target_color(target)
        marker = _target_marker(target)
        target_cells[(cx, cy)] = (marker, color)
        # Also plot the target as braille dots for glow effect on focus
        px, py = _world_to_pixel(target.x, target.y, display_range, canvas.px_width, canvas.px_height)
        glow = 4 if target.is_focus else 2
        canvas.plot_point(px, py, radius=glow)

    # Render as Rich Text: braille grid with target markers overlaid
    result = Text()
    for cy in range(char_height):
        for cx in range(char_width):
            if (cx, cy) in target_cells:
                marker, color = target_cells[(cx, cy)]
                result.append(marker, style=f"bold {color}")
            else:
                ch = canvas.get_char(cx, cy)
                result.append(ch, style="#555555")
        result.append("\n")

    return result


def render_radar_text(
    data: RadarData,
    char_width: int = 30,
    char_height: int = 12,
) -> str:
    """Render radar to plain braille string (for testing without widget mount)."""
    display_range = data.display_range_m
    canvas = BrailleCanvas(char_width, char_height)
    sensor_cx = canvas.px_width / 2.0
    sensor_cy = canvas.px_height - 1

    ring_step = _nice_ring_step(display_range)
    ring_m = ring_step
    while ring_m <= display_range:
        r_px = _range_to_radius_px(ring_m, display_range, canvas.px_height)
        canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi)
        ring_m += ring_step

    for vz_m in (data.vitals_inner_m, data.vitals_outer_m):
        if vz_m <= display_range:
            r_px = _range_to_radius_px(vz_m, display_range, canvas.px_height)
            canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi, dashed=True)

    for target in data.targets:
        px, py = _world_to_pixel(target.x, target.y, display_range, canvas.px_width, canvas.px_height)
        glow = 4 if target.is_focus else 2
        canvas.plot_point(px, py, radius=glow)

    return canvas.render_plain()


# ---------------------------------------------------------------------------
# Target info list rendering
# ---------------------------------------------------------------------------


def _render_target_list(data: RadarData) -> Text:
    """Render the target info list with state summary."""
    content = Text()

    # State summary
    if data.device_state:
        state_color = {
            "STILL_NEAR": "#4ade80",
            "RESTING_VITALS": "#60a5fa",
            "MOVING": "#fbbf24",
            "PRESENT_FAR": "#fbbf24",
            "NO_TARGET": "#555555",
            "MULTI_TARGET": "#a78bfa",
        }.get(data.device_state, "#888888")
        content.append(data.device_state, style=f"bold {state_color}")
        if data.device_pose:
            content.append(f"  {data.device_pose}", style="dim")
        content.append("\n")
        if data.dist_cm is not None:
            content.append(f"  dist {data.dist_cm:.1f}cm", style="dim")
            content.append(f"  human={data.human}\n", style="dim")
        content.append("\n")

    # Targets
    content.append("TARGETS", style="bold")
    content.append(f"  {data.n_targets}\n")

    if not data.targets:
        content.append("  (none detected)\n", style="dim")
    else:
        for i, t in enumerate(data.targets):
            color = _target_color(t)
            marker = _target_marker(t)
            label = "focus" if t.is_focus else f"t{t.cluster}"
            content.append(f"  {marker} ", style=f"bold {color}")
            content.append(f"{label:<6}", style=f"{color}")
            content.append(f" r={t.r:>5.2f}m", style="")
            content.append(f" \u03b8={t.bearing:>5.1f}\u00b0\n", style="dim")

    # Vitals zone indicator
    content.append("\n")
    in_zone = any(data.vitals_inner_m <= t.r <= data.vitals_outer_m for t in data.targets)
    if in_zone:
        content.append("  \u2665 in vitals zone\n", style="bold #f472b6")
    elif data.targets:
        content.append("  outside vitals zone\n", style="dim")

    return content


# ---------------------------------------------------------------------------
# RadarPanel Widget (container: radar canvas + target list)
# ---------------------------------------------------------------------------

_DEFAULT_CHAR_WIDTH = 40
_DEFAULT_CHAR_HEIGHT = 18


class _RadarCanvas(Static):
    """Braille radar canvas sub-widget."""

    pass


class _TargetList(Static):
    """Target info list sub-widget."""

    pass


class RadarPanel(Horizontal):
    """Radar view with braille canvas and target/state info list."""

    DEFAULT_CSS = """
    RadarPanel {
        height: 100%;
    }
    RadarPanel > _RadarCanvas {
        width: 2fr;
        height: 100%;
    }
    RadarPanel > _TargetList {
        width: 1fr;
        min-width: 22;
        height: 100%;
    }
    """

    def __init__(self, **kwargs: object) -> None:
        """Create a radar panel that scales to available space."""
        super().__init__(**kwargs)
        self._data = RadarData()

    def compose(self) -> ComposeResult:
        """Build layout: radar canvas (left) + target list (right)."""
        yield _RadarCanvas("", id="radar-canvas")
        yield _TargetList("", id="target-list")

    def on_mount(self) -> None:
        """Render initial empty radar on mount."""
        self._refresh_content()

    def on_resize(self) -> None:
        """Re-render when terminal resizes."""
        self._refresh_content()

    def update_targets(self, data: RadarData) -> None:
        """Push new target data and re-render the radar."""
        self._data = data
        self._refresh_content()

    def _get_canvas_size(self) -> tuple[int, int]:
        """Get the radar canvas character dimensions from actual widget size."""
        try:
            canvas_widget = self.query_one("#radar-canvas", _RadarCanvas)
            w = canvas_widget.size.width
            h = canvas_widget.size.height
            if w > 4 and h > 4:
                return w, h - 2  # reserve 2 rows for header + legend
        except Exception:
            pass
        return _DEFAULT_CHAR_WIDTH, _DEFAULT_CHAR_HEIGHT

    def _refresh_content(self) -> None:
        """Redraw both sub-widgets."""
        data = self._data
        char_width, char_height = self._get_canvas_size()

        # Radar canvas
        radar_content = Text()
        radar_content.append("RADAR", style="bold")
        if data.n_targets > 0:
            radar_content.append(f"  {data.n_targets} target{'s' if data.n_targets != 1 else ''}")
        radar_content.append("\n")
        radar_content.append_text(render_radar_rich(data, char_width, char_height))
        display_range = data.display_range_m
        ring_step = _nice_ring_step(display_range)
        legend_parts: list[str] = []
        ring_m = ring_step
        while ring_m <= display_range:
            legend_parts.append(f"{ring_m:g}m")
            ring_m += ring_step
        radar_content.append(" \u2500 ".join(legend_parts), style="dim")

        try:
            self.query_one("#radar-canvas", _RadarCanvas).update(radar_content)
        except Exception:
            pass

        # Target list
        try:
            self.query_one("#target-list", _TargetList).update(_render_target_list(data))
        except Exception:
            pass
