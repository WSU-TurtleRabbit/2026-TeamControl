"""Contracts for the Skill Intent Executor pipeline.

Defines the enums and state objects shared between the Behaviour Tree,
the Skill Intent Executor, and its sub-modules (PD Controller, Actuator,
Command Composer).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

from TeamControl.bt.actuator import ActuatorMode
from TeamControl.bt.contracts.geometry import Pose2D


class SkillIntent(Enum):
    STOP = auto()
    MOVE_TO_POINT = auto()
    GO_TO_BALL = auto()
    GO_TO_BALL_AND_KICK = auto()
    FACE_TARGET = auto()
    DRIBBLE_TO_POINT = auto()
    TEST_PD = auto()
    TEST_ACTUATOR = auto()
    CUSTOM = auto()


class PDMode(Enum):
    NONE = auto()
    ROUGH = auto()
    PRECISE = auto()
    TURN_ONLY = auto()
    PASS_THROUGH = auto()


@dataclass
class SkillExecutionState:
    """Mutable execution state for one robot — updated each tick by the executor."""
    robot_id: int
    is_yellow: bool
    skill_intent: SkillIntent
    pd_mode: PDMode
    actuator_mode: ActuatorMode
    original_target_pose: Pose2D
    active_target_pose: Pose2D
    waypoints: list[Pose2D] = field(default_factory=list)
    current_waypoint_index: int = 0
    clearance_mm: float = 0.0
    is_planning: bool = False
    is_rerouting: bool = False
    is_moving: bool = False
    is_dribbling: bool = False
    is_kicking: bool = False
    robot_reached_target: bool = False


@dataclass
class ExecutionContext:
    """Optional overrides — only applied for TEST_PD, TEST_ACTUATOR, CUSTOM intents."""
    pd_mode_override: PDMode | None = None
    actuator_mode_override: ActuatorMode | None = None
