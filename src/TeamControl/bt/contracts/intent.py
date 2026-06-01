"""Intent — typed output objects produced by behaviour trees.

R002: Behaviour trees output Intent objects, never raw motor commands.
All Intent dataclasses are frozen (immutable) and carry only semantic goal
information — no velocity, kick, or dribbler fields.
"""
from __future__ import annotations

import dataclasses
from enum import Enum


class IntentType(Enum):
    """Discriminant enum for each Intent variant."""

    MOVE = 1
    KICK = 2
    RECEIVE = 3
    PASS = 4
    DRIBBLE = 5
    ORIENT = 6


@dataclasses.dataclass(frozen=True)
class IntentMove:
    """Move to a target position, optionally facing a given orientation."""

    target_pos: tuple[float, float]
    target_orientation: float | None
    # Optional speed cap in m/s. None means use the skill's default (2.0 m/s).
    # Set to 1.4 during STOPPED to comply with the SSL < 1.5 m/s rule.
    max_speed: float | None = None


@dataclasses.dataclass(frozen=True)
class IntentKick:
    """Kick the ball toward a target position."""

    target_pos: tuple[float, float]


@dataclasses.dataclass(frozen=True)
class IntentPass:
    """Pass the ball to a specific allied robot."""

    target_robot_id: int
    target_pos: tuple[float, float]


@dataclasses.dataclass(frozen=True)
class IntentDribble:
    """Dribble the ball toward a target position."""

    target_pos: tuple[float, float]


@dataclasses.dataclass(frozen=True)
class IntentReceive:
    """Signal readiness to receive a pass — no positional arguments."""


@dataclasses.dataclass(frozen=True)
class IntentOrient:
    """Rotate in place to face a target orientation."""

    target_orientation: float


# Union type covering all intent variants.
Intent = IntentMove | IntentKick | IntentPass | IntentDribble | IntentReceive | IntentOrient
