"""MotionTarget — the output contract of every skill function.

R009: Each skill function returns a MotionTarget describing the desired
robot motion for the current tick. The motion execution layer reads this
and drives the robot accordingly.

MotionTarget is frozen (immutable) so it can be safely passed between
the skill layer and motion execution without defensive copying.
"""
from __future__ import annotations

import dataclasses


@dataclasses.dataclass(frozen=True)
class MotionTarget:
    """Desired robot motion for a single tick, produced by a skill function.

    Attributes:
        target_velocity: Desired (vx, vy) velocity in m/s in the robot's
            local frame.
        target_orientation: Desired heading in radians (world frame).
        arrival_mode: How to approach the target.  One of:
            ``"precision"`` — slow, accurate stop at target;
            ``"normal"``    — balanced speed and accuracy;
            ``"fast"``      — maximum speed, lower positional accuracy.
    """

    target_velocity: tuple[float, float]
    target_orientation: float
    arrival_mode: str
