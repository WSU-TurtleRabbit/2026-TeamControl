"""
Pure stateless skill functions: Intent + Snapshot -> MotionTarget.

Each function takes the robot pose and relevant world-state inputs and returns
a MotionTarget describing the velocities and actuator flags for one tick.
No side effects except KickState (passed in explicitly by the caller).

Usage:
    mt = move_to(robot_pos, target)
    mt = kick_at(ks, robot_pos, ball_pos, aim_pos, now)
    mt = receive_ball(robot_pos, target, ball_pos)
    mt = dribble_backwards(robot_pos, target)

    cmd_mgr.update_command(vx=mt.vx, vy=mt.vy, w=mt.w,
                           kick=mt.kick, dribble=mt.dribble)
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from TeamControl.robot.path_planner import (
    move_toward, move_toward_relative, turn_toward, move_and_face,
)
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.constants import (
    CRUISE_SPEED, DRIBBLE_SPEED, BALL_NEAR, KICK_RANGE,
)

Pose = Tuple[float, float, float]   # (x, y, orientation_rad)
Vec2 = Tuple[float, float]          # (x, y)

_CAPTURE_DIST = 130  # mm — ball touching dribbler; mirrors kick_engine.CONTACT_DIST


@dataclass
class MotionTarget:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    kick: int = 0
    dribble: int = 0
    done: bool = False  # True when the skill has achieved its goal this tick


# ─────────────────────────────────────────────────────────────────────────────


def move_to(
    robot_pos: Pose,
    target_pos: Vec2,
    max_speed: float = CRUISE_SPEED,
    face_target: bool = True,
    stop_radius: float = 40.0,
) -> MotionTarget:
    """
    Command the robot to move to target_pos.

    face_target=True rotates the robot to face the direction of travel.
    Returns done=True once the robot is within stop_radius of the target.
    """
    rel = world2robot(robot_pos, target_pos)
    dist = math.hypot(rel[0], rel[1])
    if dist < stop_radius:
        return MotionTarget(done=True)

    vx, vy = move_toward_relative(rel, max_speed, stop_radius=stop_radius)
    w = turn_toward(rel) if face_target else 0.0
    return MotionTarget(vx=vx, vy=vy, w=w) 


def kick_at(
    ks: KickState,
    robot_pos: Pose,
    ball_pos: Optional[Vec2],
    aim_pos: Vec2,
    now: float,
) -> MotionTarget:
    """
    Approach the ball and kick it toward aim_pos.

    Delegates entirely to kick_tick(); KickState carries burst/cooldown tracking
    across ticks and must be owned by the caller (one per robot).
    Returns done=True immediately after the kick burst finishes.
    """
    result = kick_tick(ks, robot_pos, ball_pos, aim_pos, now)
    return MotionTarget(
        vx=result.vx, vy=result.vy, w=result.w,
        kick=result.kick, dribble=result.dribble,
        done=result.burst_done,
    )


def receive_ball(
    robot_pos: Pose,
    target_pos: Vec2,
    ball_pos: Optional[Vec2],
    activate_dist: float = BALL_NEAR,
    stop_radius: float = 40.0,
) -> MotionTarget:
    """
    Wait at target_pos, continuously watching for the ball.

    - Moves to target_pos while facing the ball (or the target if ball unknown).
    - Turns to track the ball once at the target.
    - Activates dribbler when ball enters activate_dist.
    - Returns done=True when ball reaches the dribbler (~_CAPTURE_DIST).
    """
    rel_target = world2robot(robot_pos, target_pos)
    at_target = math.hypot(rel_target[0], rel_target[1]) < stop_radius

    rel_ball: Optional[Tuple[float, float]] = None
    d_ball: Optional[float] = None
    if ball_pos is not None:
        rel_ball = world2robot(robot_pos, ball_pos)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])

    dribble = 1 if (d_ball is not None and d_ball < activate_dist) else 0
    done = d_ball is not None and d_ball < _CAPTURE_DIST

    if at_target:
        w = turn_toward(rel_ball) if rel_ball is not None else 0.0
        return MotionTarget(w=w, dribble=dribble, done=done)

    # Move toward target while facing the ball (or target if ball unknown)
    face = ball_pos if ball_pos is not None else target_pos
    vx, vy, w = move_and_face(
        robot_pos, target_pos, face,
        max_linear_speed=CRUISE_SPEED,
        stop_radius=stop_radius,
    )
    return MotionTarget(vx=vx, vy=vy, w=w, dribble=dribble, done=done)


def dribble_backwards(
    robot_pos: Pose,
    target_pos: Vec2,
    max_speed: float = DRIBBLE_SPEED,
    stop_radius: float = 40.0,
) -> MotionTarget:
    """
    Drive backward to target_pos with the dribbler on (ball held at the front).

    Keeps the current heading (w=0) so the dribbler continues to face forward
    while the robot body reverses toward the target.  The caller must ensure the
    robot already has the ball before invoking this skill.
    """
    rel = world2robot(robot_pos, target_pos)
    dist = math.hypot(rel[0], rel[1])
    if dist < stop_radius:
        return MotionTarget(dribble=1, done=True)

    vx, vy = move_toward_relative(rel, max_speed, stop_radius=stop_radius)
    return MotionTarget(vx=vx, vy=vy, w=0.0, dribble=1)
