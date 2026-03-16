"""State panel widget — device state icon, pose, lux bar, and detail metrics.

Displays a Rich Text art icon per device state with color coding,
a lux proximity bar in the header, and firmware metrics below the icon.
All rendered as a single Rich Text block inside a Static widget.
"""

from __future__ import annotations
from dataclasses import dataclass

from rich.text import Text
from textual.widgets import Static
from textual.reactive import reactive


# ---------------------------------------------------------------------------
# State color map (matches spec)
# ---------------------------------------------------------------------------

STATE_COLORS: dict[str, str] = {
    "NO_TARGET": "#555555",
    "PRESENT_FAR": "#fbbf24",
    "MOVING": "#fbbf24",
    "STILL_NEAR": "#4ade80",
    "RESTING_VITALS": "#60a5fa",
    "MULTI_TARGET": "#a78bfa",
}

DEFAULT_STATE_COLOR = "#555555"


# ---------------------------------------------------------------------------
# State icons — pre-drawn multi-line Rich Text art (~8 chars wide x 5 rows)
# Keyed by (state, pose).  Falls back to (state, "UNKNOWN") then state only.
# ---------------------------------------------------------------------------


def _make_icon(lines: list[str], color: str) -> Text:
    """Build a Rich Text block from lines with a single color."""
    txt = Text()
    for i, line in enumerate(lines):
        txt.append(line, style=color)
        if i < len(lines) - 1:
            txt.append("\n")
    return txt


def _build_icon_table() -> dict[tuple[str, str], Text]:
    """Return lookup dict of (state, pose) -> Rich Text icon."""
    icons: dict[tuple[str, str], Text] = {}

    # NO_TARGET — dashed outline
    no_target = [
        "  \u2508\u2500\u2508  ",
        "  \u250a  \u250a  ",
        "  \u251c\u2508\u2508\u2524  ",
        "  \u250a  \u250a  ",
        "  \u250a  \u250a  ",
    ]
    for pose in ("UNKNOWN", "SITTING", "STANDING"):
        icons[("NO_TARGET", pose)] = _make_icon(no_target, STATE_COLORS["NO_TARGET"])

    # PRESENT_FAR — small standing figure
    present_far = [
        "  \u256d\u2500\u2500\u256e  ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524  ",
        "  \u2502  \u2502  ",
        "  \u2575  \u2575  ",
    ]
    icons[("PRESENT_FAR", "STANDING")] = _make_icon(present_far, STATE_COLORS["PRESENT_FAR"])
    icons[("PRESENT_FAR", "UNKNOWN")] = _make_icon(present_far, STATE_COLORS["PRESENT_FAR"])
    # sitting variant
    present_far_sit = [
        "  \u256d\u2500\u2500\u256e  ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524  ",
        " \u2500\u2524  \u2502  ",
        "  \u2570\u2500\u2500\u256f  ",
    ]
    icons[("PRESENT_FAR", "SITTING")] = _make_icon(present_far_sit, STATE_COLORS["PRESENT_FAR"])

    # MOVING — walking pose with motion lines
    moving_stand = [
        "  \u256d\u2500\u2500\u256e  ",
        " \u2500\u2524  \u251c\u2500 ",
        "  \u251c\u2500\u2500\u2524  ",
        " \u2571    \u2572 ",
        "\u2575      \u2575",
    ]
    icons[("MOVING", "STANDING")] = _make_icon(moving_stand, STATE_COLORS["MOVING"])
    icons[("MOVING", "UNKNOWN")] = _make_icon(moving_stand, STATE_COLORS["MOVING"])
    moving_sit = [
        "  \u256d\u2500\u2500\u256e  ",
        " \u2500\u2524  \u251c\u2500 ",
        "  \u251c\u2500\u2500\u2524  ",
        " \u2500\u2524  \u2502  ",
        "  \u2570\u2500\u2500\u256f  ",
    ]
    icons[("MOVING", "SITTING")] = _make_icon(moving_sit, STATE_COLORS["MOVING"])

    # STILL_NEAR — still figure, ready for vitals
    still_stand = [
        "  \u256d\u2500\u2500\u256e  ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524  ",
        "  \u2502  \u2502  ",
        "  \u2575  \u2575  ",
    ]
    icons[("STILL_NEAR", "STANDING")] = _make_icon(still_stand, STATE_COLORS["STILL_NEAR"])
    icons[("STILL_NEAR", "UNKNOWN")] = _make_icon(still_stand, STATE_COLORS["STILL_NEAR"])
    still_sit = [
        "  \u256d\u2500\u2500\u256e  ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524  ",
        " \u2500\u2524  \u2502  ",
        "  \u2570\u2500\u2500\u256f  ",
    ]
    icons[("STILL_NEAR", "SITTING")] = _make_icon(still_sit, STATE_COLORS["STILL_NEAR"])

    # RESTING_VITALS — seated figure + heart + pulse wave
    vitals_sit = [
        "  \u256d\u2500\u2500\u256e\u2665 ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524\u223f ",
        " \u2500\u2524  \u2502  ",
        "  \u2570\u2500\u2500\u256f  ",
    ]
    icons[("RESTING_VITALS", "SITTING")] = _make_icon(vitals_sit, STATE_COLORS["RESTING_VITALS"])
    icons[("RESTING_VITALS", "UNKNOWN")] = _make_icon(vitals_sit, STATE_COLORS["RESTING_VITALS"])
    vitals_stand = [
        "  \u256d\u2500\u2500\u256e\u2665 ",
        "  \u2502  \u2502  ",
        "  \u251c\u2500\u2500\u2524\u223f ",
        "  \u2502  \u2502  ",
        "  \u2575  \u2575  ",
    ]
    icons[("RESTING_VITALS", "STANDING")] = _make_icon(vitals_stand, STATE_COLORS["RESTING_VITALS"])

    # MULTI_TARGET — two figures + warning
    multi = [
        " \u256d\u2500\u256e\u256d\u2500\u256e ",
        " \u2502 \u2502\u2502 \u2502 ",
        " \u251c\u2500\u2524\u251c\u2500\u2524 ",
        " \u2502 \u2502\u2502 \u2502 ",
        " \u2575 \u2575\u2575 \u2575!",
    ]
    for pose in ("UNKNOWN", "SITTING", "STANDING"):
        icons[("MULTI_TARGET", pose)] = _make_icon(multi, STATE_COLORS["MULTI_TARGET"])

    return icons


_ICON_TABLE: dict[tuple[str, str], Text] = _build_icon_table()


def get_state_icon(state: str, pose: str = "UNKNOWN") -> Text:
    """Look up the state icon for a (state, pose) pair.

    Falls back to (state, 'UNKNOWN'), then a plain text label.
    """
    icon = _ICON_TABLE.get((state, pose))
    if icon is not None:
        return icon.copy()
    icon = _ICON_TABLE.get((state, "UNKNOWN"))
    if icon is not None:
        return icon.copy()
    # Ultimate fallback — plain text
    color = STATE_COLORS.get(state, DEFAULT_STATE_COLOR)
    return Text(f"  [{state}]  ", style=color)


def get_state_color(state: str) -> str:
    """Return the color string for a given device state."""
    return STATE_COLORS.get(state, DEFAULT_STATE_COLOR)


# ---------------------------------------------------------------------------
# Lux bar rendering
# ---------------------------------------------------------------------------

LUX_MAX = 500.0  # Lux ceiling for bar scaling


def render_lux_bar(lux: float | None, width: int = 12) -> Text:
    """Render a horizontal lux bar.

    Low lux = close proximity (red). High lux = clear path (green).
    """
    if lux is None:
        return Text("\u258a" * width, style="#555555")

    fraction = min(lux / LUX_MAX, 1.0)
    filled = max(1, int(fraction * width))

    if lux < 40.0:
        color = "#ef4444"  # close proximity — red
    elif lux < 120.0:
        color = "#fbbf24"  # partial occlusion — yellow
    else:
        color = "#4ade80"  # clear path — green

    bar = Text()
    bar.append("\u258a" * filled, style=color)
    bar.append("\u258a" * (width - filled), style="#333333")
    return bar


# ---------------------------------------------------------------------------
# StatePanel Widget
# ---------------------------------------------------------------------------


@dataclass
class StateData:
    """Snapshot of state panel data. Used as the update interface."""

    state: str = "NO_TARGET"
    pose: str = "UNKNOWN"
    human: int = 0
    n_targets: int = 0
    dist_cm: float | None = None
    head_moving: int = 0
    dist_new: int = 0
    lux: float | None = None


class StatePanel(Static):
    """Device state display rendered as Rich Text.

    Shows state icon, name, pose, lux bar, and firmware metrics.
    """

    state: reactive[str] = reactive("NO_TARGET")
    pose: reactive[str] = reactive("UNKNOWN")

    def __init__(self, **kwargs: object) -> None:
        """Create the state panel with default content."""
        super().__init__("", **kwargs)
        self._lux: float | None = None

    def update_state(self, data: StateData) -> None:
        """Push new state data into the panel."""
        self.state = data.state
        self.pose = data.pose
        self._lux = data.lux
        self._refresh_content()

    def update_lux(self, lux: float | None) -> None:
        """Update only the lux bar (called from light events)."""
        self._lux = lux
        self._refresh_content()

    def _refresh_content(self) -> None:
        """Rebuild the Rich Text content from current data."""
        content = Text()

        # Header: STATE + lux bar
        content.append("STATE", style="bold")
        content.append("  ")
        content.append("LUX ")
        content.append_text(render_lux_bar(self._lux))
        if self._lux is not None:
            content.append(f" {self._lux:.1f}")
        content.append("\n\n")

        # State icon
        icon = get_state_icon(self.state, self.pose)
        content.append_text(icon)
        content.append("\n\n")

        # State name with color
        color = get_state_color(self.state)
        content.append(self.state, style=f"bold {color}")
        content.append("  ")
        content.append(_pose_description(self.state, self.pose))

        self.update(content)


def _pose_description(state: str, pose: str) -> str:
    """Human-readable pose description for the info area."""
    if state == "NO_TARGET":
        return "No one detected"
    if state == "MULTI_TARGET":
        return "Multiple people"
    pose_label = {"SITTING": "Seated", "STANDING": "Standing"}.get(pose, "")
    desc_map: dict[str, str] = {
        "PRESENT_FAR": "Beyond vitals range",
        "MOVING": "In motion",
        "STILL_NEAR": "Vitals ready" if pose_label else "Still, in range",
        "RESTING_VITALS": "Measuring vitals",
    }
    desc = desc_map.get(state, state)
    if pose_label:
        return f"{pose_label} \u00b7 {desc}"
    return desc
