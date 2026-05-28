"""move_to skill — navigate a robot to a target position.

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

    The robot moves directly toward the target at a speed proportional to
    the distance, clamped to *max_speed*.  Returns (0.0, 0.0) when the
    robot is already at the target.
    """
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return (0.0, 0.0)
    speed = min(dist, max_speed)
    return (dx / dist * speed, dy / dist * speed)


def move_to(
    snapshot: Snapshot,
    robot_id: int,
    target_pos: tuple[float, float],
    target_orientation: float | None = None,
    max_speed: float | None = None,
) -> MotionTarget:
    """Navigate robot *robot_id* to *target_pos*.

    Args:
        snapshot: Read-only world state for the current tick.
        robot_id: Identifier of the robot to move.
        target_pos: Desired (x, y) position in world coordinates (m).
        target_orientation: Desired heading in radians on arrival.
            If ``None``, heading defaults to 0.0 (unconstrained).

    Returns:
        A :class:`MotionTarget` with ``arrival_mode="precision"``.

    Raises:
        ValueError: If *robot_id* is not present in *snapshot*.
    """
    robot = _get_robot(snapshot, robot_id)
    velocity = _proportional_velocity(
        robot.position, target_pos, max_speed=max_speed if max_speed is not None else _MAX_SPEED
    )
    # Keep current orientation when none specified — avoids spinning to face 0.0.
    orientation = target_orientation if target_orientation is not None else robot.orientation
    return MotionTarget(
        target_velocity=velocity,
        target_orientation=float(orientation),
        arrival_mode="precision",
    )
