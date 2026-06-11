"""Actuator Controller — Task 3 of the Skill Intent Executor pipeline.

Receives an ActuatorInput (mode + robot/ball state + PD output) and returns
an ActuatorOutput (kick: bool, dribble: bool).

Rules:
  - NONE        → kick=False, dribble=False always
  - DRIBBLE     → dribble=True when ball is within DRIBBLE_ZONE and angle error
                  is within DRIBBLE_ANGLE; kick=False always
  - KICK        → kick=True when ball is within KICK_ZONE and angle error is
                  within KICK_ANGLE; dribble=False always
  - DRIBBLE_KICK → dribble engages first (wider zone); once inside KICK_ZONE
                   and angle satisfied, dribble turns off and kick fires

Does NOT compute velocity commands (that is the PD Controller's job).
Does NOT create RobotCommand objects (that is the Command Composer's job).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto

from TeamControl.bt.contracts.geometry import Point2D, Pose2D, Velocity2D
from TeamControl.bt.contracts.pd_output import PDOutput


# ---------------------------------------------------------------------------
# Mode enum
# ---------------------------------------------------------------------------

class ActuatorMode(Enum):
    NONE = auto()
    DRIBBLE = auto()
    KICK = auto()
    DRIBBLE_KICK = auto()


# ---------------------------------------------------------------------------
# Zone thresholds (spec values)
# ---------------------------------------------------------------------------

DRIBBLE_ZONE_MM: float = 150.0   # mm — engage dribbler when ball is closer than this
DRIBBLE_ANGLE_RAD: float = 0.01  # rad — angle tolerance for dribbler engagement
KICK_ZONE_MM: float = 80.0       # mm — fire kick when ball is closer than this
KICK_ANGLE_RAD: float = 0.015    # rad — angle tolerance for kick engagement

# Convert mm thresholds to metres once (the rest of the codebase uses metres)
_DRIBBLE_ZONE_M: float = DRIBBLE_ZONE_MM / 1000.0
_KICK_ZONE_M: float = KICK_ZONE_MM / 1000.0


# ---------------------------------------------------------------------------
# I/O dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ActuatorInput:
    """Everything the Actuator Controller needs for one tick."""
    robot_id: int
    is_yellow: bool
    actuator_mode: ActuatorMode
    current_pose: Pose2D            # robot position + heading (world frame)
    ball_pos: Point2D | None        # None if ball is not visible
    ball_vel: Velocity2D | None     # None if ball velocity is not available
    pd_output: PDOutput             # forwarded from the PD Controller (read-only)


@dataclass(frozen=True)
class ActuatorOutput:
    """Actuator decisions for the Command Composer."""
    kick: bool
    dribble: bool


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _dist_to_ball(pose: Pose2D, ball: Point2D) -> float:
    """Euclidean distance in metres from robot centre to ball."""
    return math.hypot(pose.x - ball.x, pose.y - ball.y)


def _angle_to_ball(pose: Pose2D, ball: Point2D) -> float:
    """Absolute angle error in radians between the robot heading and the ball.

    Returns the smallest positive difference so it can be compared against a
    tolerance threshold regardless of sign.
    """
    world_angle = math.atan2(ball.y - pose.y, ball.x - pose.x)
    error = world_angle - pose.theta
    # Normalise to (-π, π]
    error = (error + math.pi) % (2 * math.pi) - math.pi
    return abs(error)


def _in_dribble_zone(pose: Pose2D, ball: Point2D) -> bool:
    return (
        _dist_to_ball(pose, ball) <= _DRIBBLE_ZONE_M
        and _angle_to_ball(pose, ball) <= DRIBBLE_ANGLE_RAD
    )


def _in_kick_zone(pose: Pose2D, ball: Point2D) -> bool:
    return (
        _dist_to_ball(pose, ball) <= _KICK_ZONE_M
        and _angle_to_ball(pose, ball) <= KICK_ANGLE_RAD
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def run_actuator(inp: ActuatorInput) -> ActuatorOutput:
    """Compute actuator outputs for the current tick.

    Args:
        inp: ActuatorInput for this tick.

    Returns:
        ActuatorOutput with kick and dribble booleans.
    """
    mode = inp.actuator_mode

    if mode is ActuatorMode.NONE:
        return ActuatorOutput(kick=False, dribble=False)

    # Can't act without ball position
    if inp.ball_pos is None:
        return ActuatorOutput(kick=False, dribble=False)

    pose = inp.current_pose
    ball = inp.ball_pos

    if mode is ActuatorMode.DRIBBLE:
        return ActuatorOutput(kick=False, dribble=_in_dribble_zone(pose, ball))

    if mode is ActuatorMode.KICK:
        return ActuatorOutput(kick=_in_kick_zone(pose, ball), dribble=False)

    if mode is ActuatorMode.DRIBBLE_KICK:
        if _in_kick_zone(pose, ball):
            # Ball is close enough to kick — stop dribbling, fire kick
            return ActuatorOutput(kick=True, dribble=False)
        # Ball is approaching — dribble to guide it into kick zone
        return ActuatorOutput(kick=False, dribble=_in_dribble_zone(pose, ball))

    return ActuatorOutput(kick=False, dribble=False)
