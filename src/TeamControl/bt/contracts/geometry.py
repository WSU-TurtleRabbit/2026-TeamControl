"""Shared geometric primitives for the Skill Intent pipeline."""
from __future__ import annotations

import dataclasses


@dataclasses.dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclasses.dataclass(frozen=True)
class Velocity2D:
    vx: float
    vy: float


@dataclasses.dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float  # heading in radians, world frame
