"""
Simple striker — approach, align, kick using shared kick engine.

Behaviors checked each tick:
  1. WAIT  — ball inside penalty box -> hold outside, face ball
  2. KICK  — ball reachable -> kick engine handles approach + align + burst
"""

import time
import math

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.ball_nav import (
    clamp, move_toward, wall_brake, rotation_compensate,
)
from TeamControl.robot.navigator import _compute_avoidance
from TeamControl.robot.kick_engine import KickState, kick_tick
from TeamControl.cache import TickCache
from TeamControl.robot.constants import (
    FIELD_LENGTH, HALF_LEN, HALF_WID,
    GOAL_WIDTH, GOAL_HW, GOAL_DEPTH,
    PENALTY_DEPTH, PENALTY_HW,
    KICK_RANGE, BALL_NEAR, BEHIND_DIST, AVOID_RADIUS,
    CRUISE_SPEED, CHARGE_SPEED, DRIBBLE_SPEED,
    MAX_W, TURN_GAIN,
    KICK_COOLDOWN, LOOP_RATE, FRAME_INTERVAL,
    DEFENSE_DEPTH, POSSESS_DIST
)

# -- Tuning ---------------------------------------------------------------
APPROACH_SPD   = CRUISE_SPEED
WAIT_SPD       = CRUISE_SPEED * 0.6

BALL_MEMORY_TIME = 0.5    # seconds to trust last-known ball when occluded


def _in_penalty_box(px, py, goal_x):
    return abs(px - goal_x) < PENALTY_DEPTH and abs(py) < PENALTY_HW


def _pick_aim(cache, ball, goal_x, is_yellow):
    """Pick aim point in goal mouth, away from opponent goalie."""
    aim_inward = 1 if goal_x > 0 else -1
    aim_x = goal_x + aim_inward * (GOAL_DEPTH * 0.5)

    gk_y = None
    for _oid, pos in cache.robots.iter_team(not is_yellow):
        if abs(pos[0] - goal_x) < DEFENSE_DEPTH:
            gk_y = pos[1]
            break

    if gk_y is not None:
        aim_y = -GOAL_HW * 0.6 if gk_y > 0 else GOAL_HW * 0.6
    else:
        aim_y = 0.0

    return (aim_x, aim_y)


def _get_opponent(cache, is_yellow):
    """Return first visible opponent as (x, y, orientation), or None."""
    for _oid, pos in cache.robots.iter_team(not is_yellow):
        return pos
    return None


def _opponent_has_ball(opp, ball):
    """Check if opponent is possessing the ball (close + ball in front)."""
    if opp is None or ball is None:
        return False
    d = math.hypot(opp[0] - ball[0], opp[1] - ball[1])
    if d > POSSESS_DIST:
        return False
    # Check if ball is roughly in front of opponent
    rel = world2robot(opp, ball)
    return rel[0] > -30  # ball not behind them


# -- Main loop ------------------------------------------------------------

def run_striker(is_running, dispatch_q, wm, robot_id=0, is_yellow=True):
    cache = TickCache(wm)
    last_d_ball = float('inf')
    prev_obs_pos = {}

    ks = KickState()

    while is_running.is_set():
        now = time.time()

        # -- Refresh all cached categories from the frame --------------
        if not cache.refresh(now):
            time.sleep(LOOP_RATE)
            continue
        frame = cache.frame

        rpos = cache.robots.get_position(is_yellow, robot_id)
        if rpos is None:
            time.sleep(LOOP_RATE)
            continue

        # -- Resolve ball (use memory if occluded) ---------------------
        ball_visible = cache.ball.visible
        if ball_visible:
            ball = cache.ball.position
        else:
            ball = cache.ball.last_known(now, BALL_MEMORY_TIME)
            if ball is None or last_d_ball >= BALL_NEAR:
                time.sleep(LOOP_RATE)
                continue

        # -- Ball velocity (cached, recomputed on new frame) ----------
        bvx, bvy, bspeed = cache.ball.velocity

        # -- Team / side info (cached) --------------------------------
        goal_x = cache.team.goal_x(is_yellow)
        our_goal_x = cache.team.their_goal_x(is_yellow)

        # -- Robot-local vectors ----------------------------------------
        rel_ball = world2robot(rpos, ball)
        d_ball = math.hypot(rel_ball[0], rel_ball[1])
        ang_ball = math.atan2(rel_ball[1], rel_ball[0])
        if ball_visible:
            last_d_ball = d_ball

        # -- Opponent info ----------------------------------------------
        opp = _get_opponent(cache, is_yellow)
        opp_has_ball = _opponent_has_ball(opp, ball)
        opp_dist_to_ball = math.hypot(opp[0] - ball[0], opp[1] - ball[1]) \
            if opp is not None else float('inf')

        aim = _pick_aim(cache, ball, goal_x, is_yellow)

        vx, vy, w = 0.0, 0.0, 0.0
        kick, dribble = 0, 0

        # ==============================================================
        #  KICK ENGINE — handles approach, align, contact, burst
        # ==============================================================
        kr = kick_tick(ks, rpos, ball, aim, now, rel_ball, d_ball)
        vx, vy, w = kr.vx, kr.vy, kr.w
        kick, dribble = kr.kick, kr.dribble

        # -- Obstacle avoidance (reduced when dribbling/kicking) --------
        avoid_vx, avoid_vy, _closest, prev_obs_pos = _compute_avoidance(
            rpos, frame, is_yellow, robot_id, rel_ball, prev_obs_pos)
        if dribble == 1 or kick == 1:
            avoid_vx *= 0.3
            avoid_vy *= 0.3
        vx += avoid_vx
        vy += avoid_vy

        # -- Face ball when idle ----------------------------------------
        if kick == 0 and dribble == 0:
            if abs(ang_ball) < 0.04:
                w = 0.0
            else:
                w = clamp(ang_ball * TURN_GAIN, -MAX_W, MAX_W)

        # -- Wall braking -----------------------------------------------
        vx, vy = wall_brake(rpos[0], rpos[1], vx, vy)

        # -- Rotation compensation --------------------------------------
        vx, vy = rotation_compensate(vx, vy, w)

        cmd = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                           kick=kick, dribble=dribble,
                           isYellow=is_yellow)
        dispatch_q.put((cmd, 0.15))
        time.sleep(LOOP_RATE)
