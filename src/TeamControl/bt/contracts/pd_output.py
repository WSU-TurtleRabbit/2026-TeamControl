"""PDOutput — contract between the PD Controller (Task 2) and its consumers."""
from __future__ import annotations

import dataclasses


@dataclasses.dataclass(frozen=True)
class PDOutput:
    """Velocity command produced by the PD Controller for one robot tick.

    All velocities are in the robot's local frame (forward = +x).
    """
    vx: float                    # forward velocity m/s
    vy: float                    # lateral velocity m/s
    w: float                     # angular velocity rad/s
    robot_reached_target: bool   # True when within position + heading tolerance
    distance_error: float        # distance to target in metres
    heading_error: float         # heading error in radians
