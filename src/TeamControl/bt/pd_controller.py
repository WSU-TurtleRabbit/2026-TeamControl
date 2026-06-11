"""PD Controller — Task 2.

Converts current pose + target pose + PDMode into velocity commands.
Internally owns previous-error state per robot.

Does NOT know about: dribbler, kicker, SkillIntent, RobotCommand, Dispatcher.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from TeamControl.bt.contracts.geometry import Pose2D
from TeamControl.bt.contracts.pd_output import PDOutput
from TeamControl.bt.contracts.skill_intent import PDMode
from TeamControl.robot.pd_controller import PDController as _RawPD

# ---------------------------------------------------------------------------
# Gains
# ---------------------------------------------------------------------------

_KP_LIN: float = 2.5     # proportional gain for linear position error
_KD_LIN: float = 0.05    # derivative gain for linear position error
_KP_ANG: float = 4.0     # proportional gain for heading error
_KD_ANG: float = 0.05    # derivative gain for heading error

_MAX_SPEED: float = 2.0  # m/s
_MAX_W: float = 6.0      # rad/s

# ---------------------------------------------------------------------------
# Mode tolerances
# ---------------------------------------------------------------------------

_TOLERANCES: dict[PDMode, tuple[float, float]] = {
    PDMode.ROUGH:   (0.120, 0.40),   # (pos_m, theta_rad)
    PDMode.PRECISE: (0.050, 0.15),
}


# ---------------------------------------------------------------------------
# PDInput
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PDInput:
    robot_id: int
    current_pose: Pose2D
    target_pose: Pose2D
    pd_mode: PDMode
    dt_s: float = 0.01          # seconds since last tick; defaults to 100 Hz period
    calibration: object | None = None  # reserved for per-robot gain overrides


# ---------------------------------------------------------------------------
# PDController class
# ---------------------------------------------------------------------------

class PDController:
    """Stateful PD controller — one instance shared across all robots.

    Internal previous-error dicts are keyed by robot_id so state is kept
    per robot without the executor needing to manage it.
    """

    def __init__(self) -> None:
        self._linear: dict[int, _RawPD] = {}
        self._angular: dict[int, _RawPD] = {}

    def _get_controllers(self, robot_id: int) -> tuple[_RawPD, _RawPD]:
        if robot_id not in self._linear:
            self._linear[robot_id] = _RawPD(kp=_KP_LIN, kd=_KD_LIN, out_limit=_MAX_SPEED)
            self._angular[robot_id] = _RawPD(kp=_KP_ANG, kd=_KD_ANG, out_limit=_MAX_W)
        return self._linear[robot_id], self._angular[robot_id]

    def reset(self, robot_id: int) -> None:
        """Clear history for one robot — call when skill intent changes abruptly."""
        if robot_id in self._linear:
            self._linear[robot_id].reset()
            self._angular[robot_id].reset()

    def run(self, inp: PDInput) -> PDOutput:
        """Compute velocity command for one robot tick."""
        mode = inp.pd_mode
        cur = inp.current_pose
        tgt = inp.target_pose
        rid = inp.robot_id
        now = None  # let _RawPD use time.monotonic() internally

        # NONE — stop
        if mode is PDMode.NONE:
            self.reset(rid)
            return PDOutput(vx=0.0, vy=0.0, w=0.0,
                            robot_reached_target=True,
                            distance_error=0.0, heading_error=0.0)

        lin_pd, ang_pd = self._get_controllers(rid)

        # Position error (world frame, metres)
        dx = tgt.x - cur.x
        dy = tgt.y - cur.y
        distance_error = math.hypot(dx, dy)

        # Heading error — normalised to (−π, π]
        heading_error = (tgt.theta - cur.theta + math.pi) % (2 * math.pi) - math.pi

        # TURN_ONLY — rotate in place
        if mode is PDMode.TURN_ONLY:
            w = float(ang_pd.update(heading_error, now))
            pos_tol, theta_tol = _TOLERANCES.get(PDMode.PRECISE, (0.05, 0.15))
            reached = abs(heading_error) <= theta_tol
            return PDOutput(vx=0.0, vy=0.0, w=w,
                            robot_reached_target=reached,
                            distance_error=distance_error,
                            heading_error=heading_error)

        # ROUGH / PRECISE — full positional control
        # Linear PD output is in world frame
        if distance_error > 1e-4:
            raw_vx, raw_vy = lin_pd.update((dx, dy), now)
        else:
            lin_pd.reset()
            raw_vx, raw_vy = 0.0, 0.0

        # Rotate world-frame velocity into robot-local frame
        cos_o = math.cos(cur.theta)
        sin_o = math.sin(cur.theta)
        vx =  raw_vx * cos_o + raw_vy * sin_o
        vy = -raw_vx * sin_o + raw_vy * cos_o

        # Angular PD
        w = float(ang_pd.update(heading_error, now))

        # Reached-target check
        pos_tol, theta_tol = _TOLERANCES.get(mode, (0.05, 0.15))
        reached = distance_error <= pos_tol and abs(heading_error) <= theta_tol

        return PDOutput(
            vx=float(vx),
            vy=float(vy),
            w=float(w),
            robot_reached_target=reached,
            distance_error=distance_error,
            heading_error=heading_error,
        )


# ---------------------------------------------------------------------------
# Module-level singleton + convenience function
# ---------------------------------------------------------------------------

_controller = PDController()


def run_pd(inp: PDInput) -> PDOutput:
    """Stateless-style entry point used by the Skill Intent Executor."""
    return _controller.run(inp)
