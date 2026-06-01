"""Snapshot — read-only world state fed into the decision pipeline each tick.

R001: Snapshot is the sole input to behaviour trees. It is frozen (immutable)
and carries no behaviour or mutation methods. All world state lives here;
the blackboard holds decision state only.
"""
from __future__ import annotations

import dataclasses
from enum import Enum
from typing import Sequence


class GamePhase(str, Enum):
    # Halted — robots must not move
    HALTED = "HALTED"
    HALF_TIME = "HALF_TIME"
    # Stopped — all robots keep 500 mm from ball
    STOPPED = "STOPPED"
    # Set-piece states (our team has the privilege)
    PREPARE_KICKOFF = "PREPARE_KICKOFF"   # move to kickoff positions
    KICKOFF = "KICKOFF"                   # attacker kicks off from centre
    OPP_KICKOFF = "OPP_KICKOFF"          # opponent kickoff — all robots to own half
    FREE_KICK = "FREE_KICK"               # attacker takes free kick
    OPP_FREE_KICK = "OPP_FREE_KICK"       # opponent free kick — keep 0.5m from ball
    BALL_PLACEMENT = "BALL_PLACEMENT"     # we place the ball
    PREPARE_PENALTY = "PREPARE_PENALTY"   # pre-kick: position robots before we shoot
    PREPARE_PENALTY_OPP = "PREPARE_PENALTY_OPP"  # pre-kick: position robots before opponent shoots
    PENALTY_SHOOT = "PENALTY_SHOOT"       # we shoot a penalty
    PENALTY_DEFEND = "PENALTY_DEFEND"     # we defend a penalty
    # Normal play
    RUNNING = "RUNNING"


@dataclasses.dataclass(frozen=True)
class RobotState:
    """Position and orientation of a single robot at a given tick."""
    robot_id: int
    position: tuple[float, float]
    orientation: float  # radians


@dataclasses.dataclass(frozen=True)
class RefereeState:
    """Referee-reported game state at a given tick."""
    game_phase: GamePhase
    score: tuple[int, int]  # (own, opponent)
    # Target position for BALL_PLACEMENT — None in all other states.
    # Coordinates are in the same unit as robot/ball positions (metres).
    ball_placement_pos: tuple[float, float] | None = None


@dataclasses.dataclass(frozen=True)
class Snapshot:
    """Immutable world state snapshot — the sole read-only input to the BT pipeline.

    One Snapshot is produced per tick by the network ingestion process and
    passed (read-only) through: Coordinator → Behaviour trees → Skill functions.

    No mutation is possible after construction (frozen dataclass).
    No world state belongs on the blackboard; if a tree node needs world data,
    it reads from the Snapshot passed to it.
    """
    ball_position: tuple[float, float]
    ball_velocity: tuple[float, float]
    own_robots: tuple[RobotState, ...]
    opponent_robots: tuple[RobotState, ...]
    referee_state: RefereeState

    def __init__(
        self,
        ball_position: tuple[float, float],
        ball_velocity: tuple[float, float],
        own_robots: Sequence[RobotState],
        opponent_robots: Sequence[RobotState],
        referee_state: RefereeState,
    ) -> None:
        # Use object.__setattr__ because the dataclass is frozen.
        object.__setattr__(self, "ball_position", tuple(ball_position))
        object.__setattr__(self, "ball_velocity", tuple(ball_velocity))
        object.__setattr__(self, "own_robots", tuple(own_robots))
        object.__setattr__(self, "opponent_robots", tuple(opponent_robots))
        object.__setattr__(self, "referee_state", referee_state)
