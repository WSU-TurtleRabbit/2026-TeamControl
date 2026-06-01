"""kick_at skill — position a robot to kick the ball toward a target.

R009: Pure stateless skill function.  No py_trees imports, no class state,
no blackboard access.  Same inputs always produce the same output.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

from TeamControl.bt.contracts.motion_target import MotionTarget

if TYPE_CHECKING:
    from TeamControl.bt.contracts.snapshot import Snapshot

# Maximum linear speed in m/s used for proportional velocity scaling.
_MAX_SPEED: float = 2.0


def _get_robot(snapshot: Snapshot, robot_id: int):
    """Return the RobotState for *robot_id* from *snapshot*, or raise ValueError."""
    for r in snapshot.own_robots:
        if r.robot_id == robot_id:
            return r
    raise ValueError(f"Robot {robot_id} not found in snapshot")


def _proportional_velocity(
    robot_pos: tuple[float, float],
    target_pos: tuple[float, float],
    max_speed: float = _MAX_SPEED,
) -> tuple[float, float]:
    """Compute a proportional velocity vector toward *target_pos*.

    Returns (0.0, 0.0) when the robot is already at the target.
    """
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return (0.0, 0.0)
    speed = min(dist, max_speed)
    return (dx / dist * speed, dy / dist * speed)


def kick_at(
    snapshot: Snapshot,
    robot_id: int,
    target_pos: tuple[float, float],
) -> MotionTarget:
    """Position robot *robot_id* to kick the ball toward *target_pos*.

    The robot drives toward the kick target at maximum speed and orients
    itself to face the target so it is ready to kick.

    Args:
        snapshot: Read-only world state for the current tick.
        robot_id: Identifier of the robot that will kick.
        target_pos: Desired (x, y) target for the kicked ball (world coords, m).

    Returns:
        A :class:`MotionTarget` with ``arrival_mode="fast"`` and
        ``target_orientation`` pointing at *target_pos*.

    Raises:
        ValueError: If *robot_id* is not present in *snapshot*.
    """
    robot = _get_robot(snapshot, robot_id)

    # Drive toward the ball at full speed — proportional slows to a crawl when
    # close, producing a weak kick. Constant speed ensures a solid contact.
    dx_b = snapshot.ball_position[0] - robot.position[0]
    dy_b = snapshot.ball_position[1] - robot.position[1]
    dist_b = math.hypot(dx_b, dy_b)
    if dist_b < 1e-9:
        velocity = (0.0, 0.0)
    else:
        velocity = (dx_b / dist_b * _MAX_SPEED, dy_b / dist_b * _MAX_SPEED)

    # Orientation: face ball→target so the kick travels the right direction.
    dx = target_pos[0] - snapshot.ball_position[0]
    dy = target_pos[1] - snapshot.ball_position[1]
    orientation = math.atan2(dy, dx)

    return MotionTarget(
        target_velocity=velocity,
        target_orientation=orientation,
        arrival_mode="fast",
    )
