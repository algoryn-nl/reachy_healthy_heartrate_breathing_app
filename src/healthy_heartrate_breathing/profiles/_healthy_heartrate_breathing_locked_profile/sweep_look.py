"""Sweep-look tool: dramatic visual head sweep left-right-center.

The sweep angle (``SWEEP_MAX_ANGLE_RAD ≈ 162°``) is intentionally larger
than the mmWave sensor sweep (``~0.8 rad ≈ 46°``) because this is a
dramatic visual gesture, not a sensor scan.
"""

import logging
from typing import Any, Dict

import numpy as np

from reachy_mini.utils import create_head_pose
from healthy_heartrate_breathing.tools.core_tools import Tool, ToolDependencies
from healthy_heartrate_breathing.dance_emotion_moves import GotoQueueMove


logger = logging.getLogger(__name__)

# Sweep geometry and timing — intentionally dramatic (~162° arc)
SWEEP_MAX_ANGLE_RAD: float = 0.9 * np.pi
SWEEP_TRANSITION_S: float = 3.0
SWEEP_HOLD_S: float = 1.0


class SweepLook(Tool):
    """Sweep head from left to right and back to center, pausing at each position."""

    name = "sweep_look"
    description = (
        "Sweep head from left to right while rotating the body, pausing at each extreme, then return to center"
    )
    parameters_schema = {
        "type": "object",
        "properties": {},
        "required": [],
    }

    async def __call__(self, deps: ToolDependencies, **kwargs: Any) -> Dict[str, Any]:
        """Execute sweep look: left -> hold -> right -> hold -> center."""
        logger.info("Tool call: sweep_look")

        try:
            return self._build_and_queue_moves(deps)
        except Exception as e:
            logger.error("sweep_look failed: %s", e)
            return {"error": f"{type(e).__name__}: {e}"}

    def _build_and_queue_moves(self, deps: ToolDependencies) -> Dict[str, Any]:
        """Build the 6-move sweep sequence and queue it."""
        # Clear any existing moves
        deps.movement_manager.clear_move_queue()

        # Get current state
        current_head_pose = deps.reachy_mini.get_current_head_pose()
        head_joints, antenna_joints = deps.reachy_mini.get_current_joint_positions()

        # Extract body_yaw from head joints (first element of the 7 head joint positions)
        current_body_yaw = head_joints[0]
        current_antenna1 = antenna_joints[0]
        current_antenna2 = antenna_joints[1]

        antennas = (current_antenna1, current_antenna2)

        # Move 1: Sweep to the left (positive yaw for both body and head)
        left_head_pose = create_head_pose(0, 0, 0, 0, 0, SWEEP_MAX_ANGLE_RAD, degrees=False)
        move_to_left = GotoQueueMove(
            target_head_pose=left_head_pose,
            start_head_pose=current_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw + SWEEP_MAX_ANGLE_RAD,
            start_body_yaw=current_body_yaw,
            duration=SWEEP_TRANSITION_S,
        )

        # Move 2: Hold at left position
        hold_left = GotoQueueMove(
            target_head_pose=left_head_pose,
            start_head_pose=left_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw + SWEEP_MAX_ANGLE_RAD,
            start_body_yaw=current_body_yaw + SWEEP_MAX_ANGLE_RAD,
            duration=SWEEP_HOLD_S,
        )

        # Move 3: Return to center from left (to avoid crossing pi/-pi boundary)
        center_head_pose = create_head_pose(0, 0, 0, 0, 0, 0, degrees=False)
        return_to_center_from_left = GotoQueueMove(
            target_head_pose=center_head_pose,
            start_head_pose=left_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw,
            start_body_yaw=current_body_yaw + SWEEP_MAX_ANGLE_RAD,
            duration=SWEEP_TRANSITION_S,
        )

        # Move 4: Sweep to the right (negative yaw for both body and head)
        right_head_pose = create_head_pose(0, 0, 0, 0, 0, -SWEEP_MAX_ANGLE_RAD, degrees=False)
        move_to_right = GotoQueueMove(
            target_head_pose=right_head_pose,
            start_head_pose=center_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw - SWEEP_MAX_ANGLE_RAD,
            start_body_yaw=current_body_yaw,
            duration=SWEEP_TRANSITION_S,
        )

        # Move 5: Hold at right position
        hold_right = GotoQueueMove(
            target_head_pose=right_head_pose,
            start_head_pose=right_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw - SWEEP_MAX_ANGLE_RAD,
            start_body_yaw=current_body_yaw - SWEEP_MAX_ANGLE_RAD,
            duration=SWEEP_HOLD_S,
        )

        # Move 6: Return to center from right
        return_to_center_final = GotoQueueMove(
            target_head_pose=center_head_pose,
            start_head_pose=right_head_pose,
            target_antennas=antennas,
            start_antennas=antennas,
            target_body_yaw=current_body_yaw,
            start_body_yaw=current_body_yaw - SWEEP_MAX_ANGLE_RAD,
            duration=SWEEP_TRANSITION_S,
        )

        # Queue all moves in sequence
        deps.movement_manager.queue_move(move_to_left)
        deps.movement_manager.queue_move(hold_left)
        deps.movement_manager.queue_move(return_to_center_from_left)
        deps.movement_manager.queue_move(move_to_right)
        deps.movement_manager.queue_move(hold_right)
        deps.movement_manager.queue_move(return_to_center_final)

        # Calculate total duration and mark as moving
        total_duration = SWEEP_TRANSITION_S * 4 + SWEEP_HOLD_S * 2
        deps.movement_manager.set_moving_state(total_duration)

        return {"status": f"sweeping look left-right-center, total {total_duration:.1f}s"}
