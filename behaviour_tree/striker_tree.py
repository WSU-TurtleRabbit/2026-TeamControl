"""Striker behaviour tree — enhanced with ball velocity tracking, intercept prediction,
one-touch shooting, and pressure awareness.

  GetWorldPositionUpdate -> GetRobotIDPosition -> CalculateStrikerAction -> SendRobotCommand
"""

import math
import time
import py_trees
from TeamControl.robot.Movement import RobotMovement
from TeamControl.robot.path_planner import move_toward_relative, turn_toward
from TeamControl.world.transform_cords import world2robot

from TeamControl.robot.constants import MAX_W, TURN_GAIN

from .common_trees import GetWorldPositionUpdate, GetRobotIDPosition, SendRobotCommand

FIELD_LENGTH = 9000
GOAL_WIDTH = 1000
GOAL_HW = GOAL_WIDTH / 2

# Speeds — full speed, no throttle
MOVE_SPEED = 2.0
SPRINT_SPEED = 2.3
CHARGE_SPEED = 1.6
DRIBBLE_SPEED = 1.1
ONETOUCH_SPEED = 1.8

BEHIND_MM = 150
CLOSE_TO_BEHIND_MM = 250
KICK_MM = 250
ALIGN_RAD = 0.18
KICK_COOLDOWN_S = 5.0
STOP_RADIUS = 35
RAMP_DIST = 350
MIN_CHARGE_VX = 0.4
FORCE_KICK_TIME = 0.6    # if near ball this long without kicking, just fire

# Ball velocity
BALL_HISTORY_SIZE = 6
FRICTION = 0.4
ONETOUCH_MIN_SPEED = 250  # mm/s for one-touch


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _predict_ball_friction(bx, by, vx, vy, dt):
    """Predict ball with friction decay."""
    t = 0.0
    step = 0.02
    while t < dt:
        s = min(step, dt - t)
        bx += vx * s
        by += vy * s
        f = max(1.0 - FRICTION * s, 0.0)
        vx *= f
        vy *= f
        t += s
    return bx, by


class CalculateStrikerAction(py_trees.behaviour.Behaviour):
    """Elite striker: intercept prediction, one-touch shots, pressure-aware kicks."""

    def __init__(self):
        super().__init__("CalculateStrikerAction")
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.last_kick_time = 0.0
        self.ball_history = []
        self.last_ball_xy = None
        self.near_ball_since = 0.0   # when we first got close to ball

    def setup(self, **kwargs):
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.bb.register_key(key="ball_pos", access=py_trees.common.Access.READ)
        self.bb.register_key(key="isPositive", access=py_trees.common.Access.READ)
        self.bb.register_key(key="vx", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="vy", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="w", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="kick", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="dribble", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        robot_pos = self.bb.robot_pos
        ball_pos = self.bb.ball_pos
        is_positive = self.bb.isPositive

        if robot_pos is None or ball_pos is None:
            self.bb.vx = self.bb.vy = self.bb.w = 0.0
            self.bb.kick = self.bb.dribble = 0
            return py_trees.common.Status.FAILURE

        rpos = (float(robot_pos[0]), float(robot_pos[1]), float(robot_pos[2]))
        ball = (float(ball_pos[0]), float(ball_pos[1]))
        goal_x = -(FIELD_LENGTH / 2) if is_positive else (FIELD_LENGTH / 2)

        # ── Ball velocity tracking ────────────────────────────
        now = time.time()
        if (self.last_ball_xy is None or
                ball[0] != self.last_ball_xy[0] or ball[1] != self.last_ball_xy[1]):
            self.ball_history.append((now, ball[0], ball[1]))
            if len(self.ball_history) > BALL_HISTORY_SIZE:
                self.ball_history.pop(0)
            self.last_ball_xy = ball

        bvx, bvy, ball_speed = 0.0, 0.0, 0.0
        if len(self.ball_history) >= 2:
            dt = self.ball_history[-1][0] - self.ball_history[0][0]
            if dt > 0.02:
                bvx = (self.ball_history[-1][1] - self.ball_history[0][1]) / dt
                bvy = (self.ball_history[-1][2] - self.ball_history[0][2]) / dt
                ball_speed = math.hypot(bvx, bvy)

        # ── Intercept prediction ──────────────────────────────
        if ball_speed > 200:
            # Find optimal intercept using friction model
            best_pt = ball
            best_t = math.hypot(rpos[0] - ball[0], rpos[1] - ball[1]) / (SPRINT_SPEED * 1000)
            for i in range(1, 10):
                pdt = i * 0.1
                px, py = _predict_ball_friction(ball[0], ball[1], bvx, bvy, pdt)
                d_rob = math.hypot(rpos[0] - px, rpos[1] - py)
                t_rob = d_rob / (SPRINT_SPEED * 1000)
                if t_rob <= pdt and t_rob < best_t:
                    best_pt = (px, py)
                    best_t = t_rob
            intercept = best_pt
        else:
            intercept = ball

        # ── Smart aim: pick best goal target ──────────────────
        aim_options = [
            (goal_x, -GOAL_HW * 0.40),
            (goal_x, -GOAL_HW * 0.15),
            (goal_x, 0),
            (goal_x, GOAL_HW * 0.15),
            (goal_x, GOAL_HW * 0.40),
        ]
        # Pick the one we're most aligned with for fastest shot
        best_aim_score = -999
        aim = (goal_x, 0.0)
        for ax, ay in aim_options:
            rel_a = world2robot(rpos, (ax, ay))
            ang = abs(math.atan2(rel_a[1], rel_a[0]))
            score = -ang  # prefer smallest angle
            score += abs(ay) / GOAL_HW * 0.3  # slight corner preference
            if score > best_aim_score:
                best_aim_score = score
                aim = (ax, ay)

        try:
            behind = RobotMovement.behind_ball_point(intercept, aim, BEHIND_MM)
        except ValueError:
            behind = intercept

        rel_ball = world2robot(rpos, ball)
        rel_intercept = world2robot(rpos, intercept)
        rel_behind = world2robot(rpos, behind)
        rel_aim = world2robot(rpos, aim)

        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        d_behind = math.hypot(rel_behind[0], rel_behind[1])
        angle_to_aim = math.atan2(rel_aim[1], rel_aim[0])

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ── One-touch: ball coming toward us ──────────────────
        ball_toward_me = (rel_ball[0] > 0 and
                          ball_speed > ONETOUCH_MIN_SPEED and
                          d_ball < 600)

        if (ball_toward_me and abs(angle_to_aim) < 0.8
                and (now - self.last_kick_time) > KICK_COOLDOWN_S):
            # Intercept and redirect
            dribble = 1
            vx, vy = move_toward_relative(
                rel_ball, ONETOUCH_SPEED,
                stop_radius=20, ramp_dist=250,
            )
            w = turn_toward(rel_aim, epsilon=0.05, max_w=MAX_W)

            if (d_ball < KICK_MM * 1.3 and abs(angle_to_aim) < ALIGN_RAD * 1.3):
                kick = 1
                dribble = 0
                self.last_kick_time = now
                vx = CHARGE_SPEED
                vy = 0.0

        # ── Normal approach/charge/kick ───────────────────────
        elif d_behind > CLOSE_TO_BEHIND_MM:
            # Approach: go to behind-ball (use intercept for moving ball)
            target_rel = rel_behind if ball_speed < 300 else world2robot(rpos, behind)
            vx, vy = move_toward_relative(
                target_rel, MOVE_SPEED if d_ball > 800 else CHARGE_SPEED,
                stop_radius=STOP_RADIUS, ramp_dist=RAMP_DIST,
            )
            # Pre-orient toward aim while approaching
            if d_ball < 600:
                w = turn_toward(rel_aim, epsilon=0.06, max_w=MAX_W)
            else:
                w = turn_toward(rel_ball, epsilon=0.08, max_w=MAX_W)
        else:
            # Charge: drive to ball, face goal, dribble
            dribble = 1
            vx, vy = move_toward_relative(
                rel_ball, CHARGE_SPEED,
                stop_radius=15, ramp_dist=220,
            )
            # Always push forward while charging — prevents halting
            if vx < MIN_CHARGE_VX and d_ball > 15:
                vx = MIN_CHARGE_VX
            w = turn_toward(rel_aim, epsilon=0.05, max_w=MAX_W)

            # Track how long we've been near the ball
            if d_ball < KICK_MM * 1.5:
                if self.near_ball_since == 0.0:
                    self.near_ball_since = now
            else:
                self.near_ball_since = 0.0

            near_ball_duration = (now - self.near_ball_since) if self.near_ball_since > 0 else 0.0
            force_kick = near_ball_duration > FORCE_KICK_TIME

            if (now - self.last_kick_time) > KICK_COOLDOWN_S and (
                (d_ball < KICK_MM and rel_ball[0] > -40 and abs(angle_to_aim) < ALIGN_RAD)
                or force_kick
            ):
                kick = 1
                dribble = 0
                self.last_kick_time = now
                self.near_ball_since = 0.0
                vx = CHARGE_SPEED
                vy = 0.0

        self.bb.vx = vx
        self.bb.vy = vy
        self.bb.w = _clamp(w, -MAX_W, MAX_W)
        self.bb.kick = kick
        self.bb.dribble = dribble
        return py_trees.common.Status.SUCCESS


class StrikerRunningSeq(py_trees.composites.Sequence):
    def __init__(self, wm, dispatch_q, robot_id, isYellow=True, isPositive=None):
        name = f"StrikerRunningSeq (RobotID:{robot_id})"
        super().__init__(name, memory=True)
        self.wm = wm
        self.dispatch_q = dispatch_q
        self.robot_id = robot_id
        self.isYellow = isYellow
        self.isPositive = isPositive if isPositive is not None else isYellow
        self.bb = py_trees.blackboard.Client(name=name)
        self.add_children([
            GetWorldPositionUpdate(wm),
            GetRobotIDPosition(robot_id),
            CalculateStrikerAction(),
            SendRobotCommand(dispatch_q),
        ])

    def setup(self, **kwargs):
        self.bb.register_key(key="robot_id", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isYellow", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isPositive", access=py_trees.common.Access.WRITE)
        self.bb.robot_id = self.robot_id
        self.bb.isYellow = self.isYellow
        self.bb.isPositive = self.isPositive
        super().setup(**kwargs)
