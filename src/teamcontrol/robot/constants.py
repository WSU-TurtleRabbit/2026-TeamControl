"""
Central constants for robot behaviour and field geometry.

Everything tunable lives here. Every robot module imports from
this file so you only have to change things in one place.

Angular/speed tuning parameters can be overridden via tuning.json
in the project root. The UI tuning tab writes to that file.

SSL Division B field dimensions used (9000x6000mm).
If we're still on the small field, change FIELD_LENGTH and FIELD_WIDTH.
"""

import json
import os

# ── load tuning overrides ───────────────────────────────────────

_TUNING_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "tuning.json")
)

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


# ═══════════════════════════════════════════════════════════════
#  FIELD GEOMETRY (mm)
# ═══════════════════════════════════════════════════════════════

# using division B small field for now — change if we move up
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


# ═══════════════════════════════════════════════════════════════
#  ROBOT PHYSICAL LIMITS
# ═══════════════════════════════════════════════════════════════

ROBOT_RADIUS      = 90       # mm

MAX_SPEED         = 1.0      # m/s — hardware limit
_MAX_W_RAW        = _t["max_w_raw"]
W_CLAMP_PCT       = _t["w_clamp_pct"]
MAX_W             = _MAX_W_RAW * W_CLAMP_PCT
TURN_GAIN         = _t["turn_gain"]

PP_GAIN           = _t["path_planner_gain"]
PP_MIN_IMPULSE    = _t["path_planner_min_impulse"]

# speeds as fraction of MAX_SPEED
SPRINT_SPEED      = 0.73 * MAX_SPEED
CRUISE_SPEED      = 0.60 * MAX_SPEED
CHARGE_SPEED      = 0.47 * MAX_SPEED
DRIBBLE_SPEED     = 0.33 * MAX_SPEED
ONETOUCH_SPEED    = 0.53 * MAX_SPEED

# goalie-specific
SAVE_SPEED        = 0.83 * MAX_SPEED
POSITION_SPEED    = 0.53 * MAX_SPEED
CLEAR_SPEED       = 0.47 * MAX_SPEED
RETREAT_SPEED     = 0.67 * MAX_SPEED
DISTRIBUTE_SPEED  = 0.40 * MAX_SPEED


# ═══════════════════════════════════════════════════════════════
#  DISTANCES (mm)
# ═══════════════════════════════════════════════════════════════

KICK_RANGE        = 190
BALL_NEAR         = 450
BEHIND_DIST       = 280
AVOID_RADIUS      = 500
MAX_ADVANCE       = PENALTY_DEPTH - 50

PRESSURE_DIST     = 500
PASS_CLEAR        = 400


# ═══════════════════════════════════════════════════════════════
#  ANGULAR
# ═══════════════════════════════════════════════════════════════

FACE_BALL_GAIN    = _t["face_ball_gain"]
ONETOUCH_ANGLE    = 0.8


# ═══════════════════════════════════════════════════════════════
#  THRESHOLDS
# ═══════════════════════════════════════════════════════════════

SHOT_SPEED        = 500      # mm/s — incoming shot
CLEAR_BALL_SPEED  = 450      # mm/s
CLEAR_BALL_DIST   = 1100     # mm
DANGER_ZONE       = HALF_LEN
ONETOUCH_MIN_SPEED = 300     # mm/s
BALL_MOVING_THRESH = 150     # mm/s


# ═══════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═══════════════════════════════════════════════════════════════

FRICTION          = 0.4
BALL_HISTORY_SIZE = 7
INTERCEPT_MAX_T   = 1.0      # seconds
INTERCEPT_STEPS   = 12


# ═══════════════════════════════════════════════════════════════
#  TIMING
# ═══════════════════════════════════════════════════════════════

LOOP_RATE         = 0.016    # ~60 Hz
FRAME_INTERVAL    = 0.04     # ~25 Hz
KICK_COOLDOWN     = 5.0      # seconds
