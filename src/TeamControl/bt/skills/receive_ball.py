"""receive_ball skill — position a robot to intercept the incoming ball.

R009: Pure stateless skill function.  No py_trees imports, no class state,
no blackboard access.  Same inputs always produce the same output.

v1 behaviour: stationary receive — the robot holds position (zero velocity)
and waits for the ball to arrive.  The behaviour tree is responsible for
positioning the robot before this skill is invoked.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from TeamControl.bt.contracts.motion_target import MotionTarget

if TYPE_CHECKING:
    from TeamControl.bt.contracts.snapshot import Snapshot


def _get_robot(snapshot: Snapshot, robot_id: int):
    """Return the RobotState for *robot_id* from *snapshot*, or raise ValueError."""
    for r in snapshot.own_robots:
        if r.robot_id == robot_id:
            return r
    raise ValueError(f"Robot {robot_id} not found in snapshot")


def receive_ball(
    snapshot: Snapshot,
    robot_id: int,
) -> MotionTarget:
    """Move robot *robot_id* to intercept and receive the ball.

    v1 implementation: the robot holds its current position (zero velocity)
    and keeps its current orientation, waiting for the ball.  Higher-level
    tree nodes are responsible for navigating the robot to the intercept
    point before this skill takes over.

    Args:
        snapshot: Read-only world state for the current tick.
        robot_id: Identifier of the robot that will receive the ball.

    Returns:
        A :class:`MotionTarget` with zero velocity and ``arrival_mode="precision"``.

    Raises:
        ValueError: If *robot_id* is not present in *snapshot*.
    """
    robot = _get_robot(snapshot, robot_id)
    return MotionTarget(
        target_velocity=(0.0, 0.0),
        target_orientation=float(robot.orientation),
        arrival_mode="precision",
    )
