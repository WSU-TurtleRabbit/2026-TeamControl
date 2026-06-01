"""
Behaviour-tree nodes that wrap the pure skill functions in robot/skills.py.

Each node reads the current snapshot from the blackboard, calls the matching
skill function, and writes the resulting (vx, vy, w, kick, dribble) back so
that SendRobotCommand can dispatch them.

Blackboard contract (shared with common_trees.py):
    READ  : robot_pos (x, y, o), ball_pos (x, y) or None
    WRITE : vx, vy, w, kick, dribble

Node lifecycle:
    RUNNING — skill is still working toward its goal
    SUCCESS — skill reported done=True
    FAILURE — required blackboard key is missing

KickAt is the only stateful node: it owns a KickState per instance and resets
it in initialise() each time the node is entered from a parent Sequence.
"""

import time

import py_trees

from TeamControl.robot.skills import (
    MotionTarget, move_to, kick_at, receive_ball, dribble_backwards,
)
from TeamControl.robot.kick_engine import KickState
from TeamControl.robot.constants import CRUISE_SPEED, DRIBBLE_SPEED, BALL_NEAR


def _write_motion(bb: py_trees.blackboard.Client, mt: MotionTarget) -> None:
    bb.vx = mt.vx
    bb.vy = mt.vy
    bb.w = mt.w
    bb.kick = mt.kick
    bb.dribble = mt.dribble


def _motion_keys(bb: py_trees.blackboard.Client) -> None:
    for key in ("vx", "vy", "w", "kick", "dribble"):
        bb.register_key(key=key, access=py_trees.common.Access.WRITE)


# ─────────────────────────────────────────────────────────────────────────────


class MoveTo(py_trees.behaviour.Behaviour):
    """
    Drive the robot to a fixed world-frame target position.

    Returns SUCCESS once inside stop_radius; RUNNING while en-route.
    """

    def __init__(
        self,
        name: str,
        target_pos,
        max_speed: float = CRUISE_SPEED,
        face_target: bool = True,
        stop_radius: float = 40.0,
    ):
        super().__init__(name)
        self.target_pos = target_pos
        self.max_speed = max_speed
        self.face_target = face_target
        self.stop_radius = stop_radius

    def setup(self, logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        _motion_keys(self.bb)

    def update(self) -> py_trees.common.Status:
        robot_pos = self.bb.robot_pos
        if robot_pos is None:
            return py_trees.common.Status.FAILURE

        mt = move_to(robot_pos, self.target_pos, self.max_speed,
                     face_target=self.face_target, stop_radius=self.stop_radius)
        _write_motion(self.bb, mt)
        return py_trees.common.Status.SUCCESS if mt.done else py_trees.common.Status.RUNNING


class KickAt(py_trees.behaviour.Behaviour):
    """
    Approach the ball and kick it toward a fixed aim position.

    Owns a KickState that is reset each time this node is re-entered from a
    parent Sequence (via initialise()).  Returns SUCCESS after the kick burst
    completes; RUNNING while approaching or aligning.
    """

    def __init__(self, name: str, aim_pos):
        super().__init__(name)
        self.aim_pos = aim_pos
        self._ks = KickState()

    def setup(self, logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.bb.register_key(key="ball_pos", access=py_trees.common.Access.READ)
        _motion_keys(self.bb)

    def initialise(self):
        self._ks.reset()

    def update(self) -> py_trees.common.Status:
        robot_pos = self.bb.robot_pos
        if robot_pos is None:
            return py_trees.common.Status.FAILURE

        ball_pos = self.bb.ball_pos if self.bb.exists("ball_pos") else None
        mt = kick_at(self._ks, robot_pos, ball_pos, self.aim_pos, time.monotonic())
        _write_motion(self.bb, mt)
        return py_trees.common.Status.SUCCESS if mt.done else py_trees.common.Status.RUNNING


class ReceiveBall(py_trees.behaviour.Behaviour):
    """
    Move to target_pos and wait for the ball, activating the dribbler early.

    Dribbler turns on once the ball enters activate_dist.
    Returns SUCCESS when the ball reaches the dribbler (captured);
    RUNNING while moving to position or waiting.
    """

    def __init__(
        self,
        name: str,
        target_pos,
        activate_dist: float = BALL_NEAR,
        stop_radius: float = 40.0,
    ):
        super().__init__(name)
        self.target_pos = target_pos
        self.activate_dist = activate_dist
        self.stop_radius = stop_radius

    def setup(self, logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.bb.register_key(key="ball_pos", access=py_trees.common.Access.READ)
        _motion_keys(self.bb)

    def update(self) -> py_trees.common.Status:
        robot_pos = self.bb.robot_pos
        if robot_pos is None:
            return py_trees.common.Status.FAILURE

        ball_pos = self.bb.ball_pos if self.bb.exists("ball_pos") else None
        mt = receive_ball(robot_pos, self.target_pos, ball_pos,
                          self.activate_dist, self.stop_radius)
        _write_motion(self.bb, mt)
        return py_trees.common.Status.SUCCESS if mt.done else py_trees.common.Status.RUNNING


class DribbleBackwards(py_trees.behaviour.Behaviour):
    """
    Reverse to target_pos while keeping the dribbler on (ball at the front).

    The robot's heading does not change — the caller must ensure target_pos is
    roughly behind the robot before entering this node.
    Returns SUCCESS when at target; RUNNING while reversing.
    """

    def __init__(
        self,
        name: str,
        target_pos,
        max_speed: float = DRIBBLE_SPEED,
        stop_radius: float = 40.0,
    ):
        super().__init__(name)
        self.target_pos = target_pos
        self.max_speed = max_speed
        self.stop_radius = stop_radius

    def setup(self, logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        _motion_keys(self.bb)

    def update(self) -> py_trees.common.Status:
        robot_pos = self.bb.robot_pos
        if robot_pos is None:
            return py_trees.common.Status.FAILURE

        mt = dribble_backwards(robot_pos, self.target_pos,
                               self.max_speed, self.stop_radius)
        _write_motion(self.bb, mt)
        return py_trees.common.Status.SUCCESS if mt.done else py_trees.common.Status.RUNNING
