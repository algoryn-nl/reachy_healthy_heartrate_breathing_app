"""Radar panel widget — braille-character top-down radar view.

Renders a semi-circle radar with range rings at 1m intervals (0-6m),
a dashed vitals zone arc at 0.35-1.5m, focus target highlighted in
purple with glow, other targets dimmed.  Sensor icon at bottom center.

Braille characters (Unicode U+2800-U+28FF) provide sub-character
resolution: each cell is 2 dots wide x 4 dots tall.
"""

from __future__ import annotations
import math
from dataclasses import field, dataclass

from rich.text import Text
from textual.widgets import Static


# ---------------------------------------------------------------------------
# Braille canvas
# ---------------------------------------------------------------------------

# Braille base is U+2800.  Each cell is a 2x4 dot matrix.
# Dot numbering (col, row) -> bit:
#   (0,0)->0x01  (1,0)->0x08
#   (0,1)->0x02  (1,1)->0x10
#   (0,2)->0x04  (1,2)->0x20
#   (0,3)->0x40  (1,3)->0x80

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

    Parameters
    ----------
    char_width:
        Width of the canvas in terminal characters.
    char_height:
        Height of the canvas in terminal characters.

    The pixel resolution is ``(char_width * 2, char_height * 4)`` because
    each braille character encodes a 2-wide x 4-tall dot matrix.

    """

    def __init__(self, char_width: int, char_height: int) -> None:
        """Create a canvas of *char_width* x *char_height* terminal cells."""
        self.char_width = char_width
        self.char_height = char_height
        self.px_width = char_width * 2
        self.px_height = char_height * 4
        # Grid of braille offset values per character cell
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
        """Draw an arc from start_angle to end_angle (radians, 0=right, pi/2=up).

        For the radar, 0 degrees is straight ahead (up on screen), and the
        arc sweeps left-to-right as a semicircle.  Angles are measured from
        the positive-x axis of a standard math coordinate system, so we map:
          angle=0   -> screen-up
          angle=pi  -> screen-down (not used for semicircle)

        When *dashed*, every other segment of ~4 pixels is skipped.
        """
        theta = start_angle
        pixel_count = 0
        while theta <= end_angle:
            # Standard math -> screen coords:
            # x_screen = cx + r * cos(theta)
            # y_screen = cy - r * sin(theta)   (screen y inverted)
            sx = cx + radius_px * math.cos(theta)
            sy = cy - radius_px * math.sin(theta)
            if not dashed or (pixel_count // 4) % 2 == 0:
                self.set_pixel(int(round(sx)), int(round(sy)))
            theta += step
            pixel_count += 1


# ---------------------------------------------------------------------------
# Target data (stub — will be replaced by sensor_models.TargetInfo)
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


# ---------------------------------------------------------------------------
# Coordinate mapping helpers
# ---------------------------------------------------------------------------


def _target_color(target: RadarTarget) -> str:
    """Color a target by distance: green=near, yellow=mid, orange=far."""
    if target.is_focus:
        return "#a78bfa"  # purple for focus
    if target.r < 1.0:
        return "#4ade80"  # green — near
    if target.r < 3.0:
        return "#fbbf24"  # yellow — mid
    return "#fb923c"  # orange — far


def _world_to_pixel(
    x_m: float,
    y_m: float,
    max_range_m: float,
    px_width: int,
    px_height: int,
) -> tuple[int, int]:
    """Convert world coordinates (meters, sensor at origin, +y = forward) to pixel coords.

    The sensor sits at the bottom center of the canvas.
    +y in world = up on screen.
    """
    # Scale: full width maps to 2 * max_range_m (left-right)
    # y=0 maps to py=(px_height - 1) (bottom), y=max_range maps to py=0 (top)
    scale_x = px_width / (2.0 * max_range_m)
    scale_y = (px_height - 1) / max_range_m

    px = int(round(px_width / 2.0 + x_m * scale_x))
    py = int(round((px_height - 1) - y_m * scale_y))
    return px, py


def _range_to_radius_px(range_m: float, max_range_m: float, px_height: int) -> float:
    """Convert a range in meters to pixel radius for arc drawing."""
    return (range_m / max_range_m) * px_height


# ---------------------------------------------------------------------------
# RadarPanel Widget
# ---------------------------------------------------------------------------

_DEFAULT_CHAR_WIDTH = 30
_DEFAULT_CHAR_HEIGHT = 12


class RadarPanel(Static):
    """Top-down braille radar display rendered as Rich Text in a Static widget."""

    def __init__(
        self,
        char_width: int = _DEFAULT_CHAR_WIDTH,
        char_height: int = _DEFAULT_CHAR_HEIGHT,
        **kwargs: object,
    ) -> None:
        """Create a radar panel with the given character dimensions."""
        super().__init__("", **kwargs)
        self._char_width = char_width
        self._char_height = char_height
        self._data = RadarData()
        self._refresh_content()

    def update_targets(self, data: RadarData) -> None:
        """Push new target data and re-render the radar."""
        self._data = data
        self._refresh_content()

    def _refresh_content(self) -> None:
        """Redraw the radar as Rich Text content."""
        data = self._data

        content = Text()

        # Header
        content.append("RADAR", style="bold")
        if data.n_targets > 0:
            content.append(f"  {data.n_targets} target{'s' if data.n_targets != 1 else ''}")
        content.append("\n")

        # Braille canvas
        plain = render_radar_text(data, self._char_width, self._char_height)
        content.append(plain, style="#888888")
        content.append("\n")

        # Legend
        legend_parts: list[str] = []
        for ring_m in (1, 3, 6):
            if ring_m <= data.max_range_m:
                legend_parts.append(f"{ring_m}m")
        content.append(" \u2500\u2500\u2500 ".join(legend_parts), style="dim")

        self.update(content)


def render_radar_text(
    data: RadarData,
    char_width: int = _DEFAULT_CHAR_WIDTH,
    char_height: int = _DEFAULT_CHAR_HEIGHT,
) -> str:
    """Render radar to plain braille string (for testing without widget mount)."""
    canvas = BrailleCanvas(char_width, char_height)

    sensor_cx = canvas.px_width / 2.0
    sensor_cy = canvas.px_height - 1

    for ring_m in range(1, int(data.max_range_m) + 1):
        r_px = _range_to_radius_px(ring_m, data.max_range_m, canvas.px_height)
        canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi)

    for vz_m in (data.vitals_inner_m, data.vitals_outer_m):
        r_px = _range_to_radius_px(vz_m, data.max_range_m, canvas.px_height)
        canvas.draw_arc(sensor_cx, sensor_cy, r_px, start_angle=0.0, end_angle=math.pi, dashed=True)

    for target in data.targets:
        px, py = _world_to_pixel(target.x, target.y, data.max_range_m, canvas.px_width, canvas.px_height)
        glow = 2 if target.is_focus else 0
        canvas.plot_point(px, py, radius=glow)

    return canvas.render_plain()
