"""
Central constants for all robot behaviour and field geometry.

Edit values HERE — every robot file imports from this single source.
Tunable angular-velocity parameters are loaded from tuning.json at the
project root.  The UI Tuning tab writes that file; restarting the model
in the same app instance picks up the new values automatically.
"""

import json
import os

_TUNING_PATH = os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, "tuning.json")
_TUNING_PATH = os.path.normpath(_TUNING_PATH)

def _load_tuning():
    defaults = {
        "max_w_raw": 0.5,
        "w_clamp_pct": 0.60,
        "turn_gain": 0.8,
        "face_ball_gain": 0.8,
        "path_planner_gain": 0.8,
        "path_planner_min_impulse": 0.15,
        "angular_slow_speed": 0.25,
        "angular_normal_speed": 0.5,
        "angular_fast_speed": 0.6,
        "turn_kp": 1.0,
        "turn_kd": 0.1,
        "linear_kp": 0.002,
        "linear_kd": 0.0005,
        "angle_epsilon": 0.1,
    }
    try:
        with open(_TUNING_PATH, "r") as f:
            data = json.load(f)
        for k in defaults:
            if k in data:
                defaults[k] = float(data[k])
    except (FileNotFoundError, json.JSONDecodeError, ValueError):
        pass
    return defaults

_t = _load_tuning()

# ═════════════════════════════════════════════════════════════════
#  FIELD GEOMETRY (mm) — SSL small field 5000 × 3000
# ═════════════════════════════════════════════════════════════════

FIELD_LENGTH      = 4500
FIELD_WIDTH       = 2230
HALF_LEN          = FIELD_LENGTH / 2
HALF_WID          = FIELD_WIDTH / 2
GOAL_WIDTH        = 1000
GOAL_HW           = GOAL_WIDTH / 2
GOAL_DEPTH        = 180

PENALTY_DEPTH     = 500
PENALTY_WIDTH     = 1000
PENALTY_HW        = PENALTY_WIDTH / 2
CENTER_RADIUS     = 500
FIELD_MARGIN      = 300

DEFENSE_DEPTH     = 1200
DEFENSE_HALF_WIDTH = 1200

# ═════════════════════════════════════════════════════════════════
#  ROBOT PHYSICAL LIMITS
# ═════════════════════════════════════════════════════════════════

ROBOT_RADIUS      = 90       # mm

MAX_SPEED         = 1.0      # m/s — absolute hardware speed limit

_MAX_W_RAW        = _t["max_w_raw"]
W_CLAMP_PCT       = _t["w_clamp_pct"]
MAX_W             = _MAX_W_RAW * W_CLAMP_PCT
TURN_GAIN         = _t["turn_gain"]

# Path-planner defaults (read by path_planner.py)
PP_GAIN           = _t["path_planner_gain"]
PP_MIN_IMPULSE    = _t["path_planner_min_impulse"]

# Field speeds as fraction of MAX_SPEED (clamped automatically)
SPRINT_SPEED      = 0.73 * MAX_SPEED   # max repositioning
CRUISE_SPEED      = 0.60 * MAX_SPEED   # medium approach
CHARGE_SPEED      = 0.47 * MAX_SPEED   # close-range drive
DRIBBLE_SPEED     = 0.33 * MAX_SPEED   # precise ball control
ONETOUCH_SPEED    = 0.53 * MAX_SPEED   # one-touch redirect

# Goalie-specific speeds (fraction of MAX_SPEED)
SAVE_SPEED        = 0.83 * MAX_SPEED   # shot-save sprint
POSITION_SPEED    = 0.53 * MAX_SPEED   # angle narrowing
CLEAR_SPEED       = 0.47 * MAX_SPEED   # dead-ball clearance
RETREAT_SPEED     = 0.67 * MAX_SPEED   # return to goal
DISTRIBUTE_SPEED  = 0.40 * MAX_SPEED   # dribble to pass

# ═════════════════════════════════════════════════════════════════
#  DISTANCES (mm)
# ═════════════════════════════════════════════════════════════════

KICK_RANGE        = 190      # trigger kick distance
KICK_DIST         = 190      # alias used by goalie
BALL_NEAR         = 450      # "close to ball" threshold
BEHIND_DIST       = 280      # lineup distance behind ball
AVOID_RADIUS      = 500      # swing-around radius
MAX_ADVANCE       = PENALTY_DEPTH - 50  # goalie must stay inside penalty box

PRESSURE_DIST     = 500      # mm — opponent "under pressure" radius
PASS_CLEAR        = 400      # mm — pass lane clearance

# ═════════════════════════════════════════════════════════════════
#  ANGULAR
# ═════════════════════════════════════════════════════════════════

FACE_BALL_GAIN    = _t["face_ball_gain"]
ONETOUCH_ANGLE    = 0.8      # max angle offset for one-touch redirect

# PD controller gains (Movement.py)
TURN_KP           = _t["turn_kp"]     # rad -> rad/s
TURN_KD           = _t["turn_kd"]     # rad/s -> rad/s
LINEAR_KP         = _t["linear_kp"]   # mm -> m/s
LINEAR_KD         = _t["linear_kd"]   # mm/s -> m/s
ANGLE_EPSILON     = _t["angle_epsilon"]  # deadband below which ω = 0

# Threshold zones for go_to_target (mm)
KICKER_ZONE        = 70       # below this, speed is 0
DRIBBLE_ZONE       = 400      # below this, speed is capped to DRIBBLE_SPEED_FRAC
DRIBBLE_SPEED_FRAC = 0.2      # fraction of max_speed inside dribble zone

# ═════════════════════════════════════════════════════════════════
#  THRESHOLDS
# ═════════════════════════════════════════════════════════════════

SHOT_SPEED        = 500      # mm/s — incoming shot detection
CLEAR_BALL_SPEED  = 450      # mm/s — clearable ball speed
CLEAR_BALL_DIST   = 1100     # mm — go clear if this close
DANGER_ZONE       = HALF_LEN # mm — ball in our half
ONETOUCH_MIN_SPEED = 300     # mm/s — min ball speed for one-touch
BALL_MOVING_THRESH = 150     # mm/s — ball considered moving

# ═════════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═════════════════════════════════════════════════════════════════

FRICTION          = 0.4      # friction deceleration factor per second
BALL_HISTORY_SIZE = 7        # frames of ball position history
INTERCEPT_MAX_T   = 1.0      # max seconds to predict ahead
INTERCEPT_STEPS   = 12       # number of prediction steps

# ═════════════════════════════════════════════════════════════════
#  TIMING
# ═════════════════════════════════════════════════════════════════

LOOP_RATE         = 0.016    # ~60 Hz main loop sleep
FRAME_INTERVAL    = 0.04     # ~25 Hz frame fetch interval
KICK_COOLDOWN     = 5.0      # seconds between kicks (hardware limit)
