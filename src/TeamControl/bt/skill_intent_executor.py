"""Skill Intent Executor — Task 1.

Converts a SkillIntent from the Behaviour Tree into a RobotCommand by
coordinating the PD Controller, Actuator Controller, and Command Composer.

This module owns:
  - SkillIntent → PDMode + ActuatorMode mapping
  - Waypoint priority logic
  - Calling order of sub-modules

This module does NOT own:
  - PD derivative math (pd_controller.py)
  - Actuator zone rules (actuator.py)
  - UDP / radio sending (dispatcher)
"""
from __future__ import annotations

from TeamControl.bt.actuator import ActuatorInput, ActuatorMode, run_actuator
from TeamControl.bt.command_composer import CommandComposerInput, compose_command
from TeamControl.bt.contracts.geometry import Point2D, Pose2D, Velocity2D
from TeamControl.bt.contracts.skill_intent import (
    ExecutionContext,
    PDMode,
    SkillExecutionState,
    SkillIntent,
)
from TeamControl.bt.pd_controller import PDInput, run_pd
from TeamControl.network.robot_command import RobotCommand

# ---------------------------------------------------------------------------
# Skill mapping table
# ---------------------------------------------------------------------------

SKILL_CONFIG: dict[SkillIntent, dict] = {
    SkillIntent.STOP: {
        "pd_mode": PDMode.NONE,
        "actuator_mode": ActuatorMode.NONE,
        "clearance_mm": 0,
    },
    SkillIntent.MOVE_TO_POINT: {
        "pd_mode": PDMode.PRECISE,
        "actuator_mode": ActuatorMode.NONE,
        "clearance_mm": 200,
    },
    SkillIntent.GO_TO_BALL: {
        "pd_mode": PDMode.PRECISE,
        "actuator_mode": ActuatorMode.DRIBBLE,
        "clearance_mm": 200,
    },
    SkillIntent.GO_TO_BALL_AND_KICK: {
        "pd_mode": PDMode.PRECISE,
        "actuator_mode": ActuatorMode.DRIBBLE_KICK,
        "clearance_mm": 200,
    },
    SkillIntent.FACE_TARGET: {
        "pd_mode": PDMode.TURN_ONLY,
        "actuator_mode": ActuatorMode.NONE,
        "clearance_mm": 0,
    },
    SkillIntent.DRIBBLE_TO_POINT: {
        "pd_mode": PDMode.PRECISE,
        "actuator_mode": ActuatorMode.DRIBBLE,
        "clearance_mm": 200,
    },
}

_TEST_INTENTS = {SkillIntent.TEST_PD, SkillIntent.TEST_ACTUATOR, SkillIntent.CUSTOM}


# ---------------------------------------------------------------------------
# Mode resolution
# ---------------------------------------------------------------------------

def _resolve_modes(
    skill_intent: SkillIntent,
    context: ExecutionContext | None,
) -> tuple[PDMode, ActuatorMode]:
    """Return (pd_mode, actuator_mode) for this intent, applying overrides if allowed."""
    cfg = SKILL_CONFIG.get(skill_intent, {"pd_mode": PDMode.NONE, "actuator_mode": ActuatorMode.NONE})
    default_pd = cfg["pd_mode"]
    default_act = cfg["actuator_mode"]

    if skill_intent in _TEST_INTENTS and context is not None:
        pd_mode = context.pd_mode_override or default_pd
        actuator_mode = context.actuator_mode_override or default_act
    else:
        pd_mode = default_pd
        actuator_mode = default_act

    return pd_mode, actuator_mode


# ---------------------------------------------------------------------------
# Waypoint priority
# ---------------------------------------------------------------------------

def _active_target(state: SkillExecutionState) -> Pose2D:
    """Return the current waypoint if one is available, otherwise original target."""
    if state.waypoints and state.current_waypoint_index < len(state.waypoints):
        return state.waypoints[state.current_waypoint_index]
    return state.original_target_pose


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def execute(
    skill_intent: SkillIntent,
    robot_id: int,
    is_yellow: bool,
    current_pose: Pose2D,
    target_pose: Pose2D,
    ball_pos: Point2D | None,
    ball_vel: Velocity2D | None,
    state: SkillExecutionState | None = None,
    context: ExecutionContext | None = None,
) -> tuple[RobotCommand, SkillExecutionState]:
    """Run one tick of the Skill Intent Executor for a single robot.

    Args:
        skill_intent:  Intent received from the Behaviour Tree this tick.
        robot_id:      Robot being controlled.
        is_yellow:     Team colour.
        current_pose:  Robot's current position and heading.
        target_pose:   Goal position/heading set by the BT.
        ball_pos:      Ball position in metres, or None if not visible.
        ball_vel:      Ball velocity, or None if not available.
        state:         Execution state from the previous tick, or None on first tick.
        context:       Optional overrides for TEST_PD / TEST_ACTUATOR / CUSTOM intents.

    Returns:
        (RobotCommand, updated SkillExecutionState)
    """
    # 1. Resolve PDMode and ActuatorMode from the mapping table.
    pd_mode, actuator_mode = _resolve_modes(skill_intent, context)

    cfg = SKILL_CONFIG.get(skill_intent, {"clearance_mm": 0})
    clearance_mm = cfg.get("clearance_mm", 0)

    # 2. Build or update execution state.
    if state is None or state.skill_intent != skill_intent or state.robot_id != robot_id:
        state = SkillExecutionState(
            robot_id=robot_id,
            is_yellow=is_yellow,
            skill_intent=skill_intent,
            pd_mode=pd_mode,
            actuator_mode=actuator_mode,
            original_target_pose=target_pose,
            active_target_pose=target_pose,
            clearance_mm=float(clearance_mm),
        )
    else:
        state.skill_intent = skill_intent
        state.pd_mode = pd_mode
        state.actuator_mode = actuator_mode
        state.original_target_pose = target_pose
        state.clearance_mm = float(clearance_mm)

    # 3. Ask planner if rerouting is needed (stub — planner not yet implemented).
    #    When the planner is ready, call it here and populate state.waypoints.

    # 4. Resolve active target: waypoint takes priority over original target.
    active_target = _active_target(state)
    state.active_target_pose = active_target

    # 5. Call PD Controller.
    pd_output = run_pd(PDInput(
        robot_id=robot_id,
        pd_mode=pd_mode,
        current_pose=current_pose,
        target_pose=active_target,
    ))

    # 6. Call Actuator Controller.
    actuator_output = run_actuator(ActuatorInput(
        robot_id=robot_id,
        is_yellow=is_yellow,
        actuator_mode=actuator_mode,
        current_pose=current_pose,
        ball_pos=ball_pos,
        ball_vel=ball_vel,
        pd_output=pd_output,
    ))

    # 7. Update state flags from PD and Actuator outputs.
    state.is_moving = pd_mode not in (PDMode.NONE,)
    state.is_dribbling = actuator_output.dribble
    state.is_kicking = actuator_output.kick
    state.robot_reached_target = pd_output.robot_reached_target

    # 8. Call Command Composer.
    command = compose_command(CommandComposerInput(
        robot_id=robot_id,
        is_yellow=is_yellow,
        pd_output=pd_output,
        actuator_output=actuator_output,
    ))

    return command, state
