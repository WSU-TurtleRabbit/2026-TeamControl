"""Adapter layer — bridges 2026 TeamControl runtime with the TurtleRabbitBT
behaviour tree pipeline.

The TurtleRabbitBT pipeline is:

    WorldModel  ──(this module)──> Snapshot
                                       │
                                Coordinator.tick()
                                       │
                                 list[Intent]
                                       │
                          ──(this module)──> RobotCommand → dispatcher_q

Two responsibilities live here so the BT core stays free of any 2026-specific
types:

* ``build_snapshot_from_world_model`` — read the latest vision frame from the
  shared ``WorldModel`` and produce a frozen ``Snapshot``.
* ``intent_to_robot_command`` — resolve an ``Intent`` into a ``RobotCommand``
  using the existing stateless skill functions.

Intent kinds not yet wired to physical kicker/dribbler hardware fall through
to a zero-velocity command with a TODO marker, so the surrounding pipeline
keeps running while higher fidelity is added.
"""
from __future__ import annotations

import math
from typing import Iterable

from TeamControl.bt.contracts.intent import (
    Intent,
    IntentDribble,
    IntentKick,
    IntentMove,
    IntentOrient,
    IntentPass,
    IntentReceive,
)
from TeamControl.bt.contracts.motion_target import MotionTarget
from TeamControl.bt.contracts.snapshot import (
    GamePhase,
    RefereeState,
    RobotState,
    Snapshot,
)
from TeamControl.bt.skills.kick_at import kick_at
from TeamControl.bt.skills.move_to import move_to
from TeamControl.network.robot_command import RobotCommand
from TeamControl.SSL.game_controller.common import GameState

# Map every GC-produced GameState into the BT's GamePhase.
# The GC FSM already resolves ours-vs-theirs before storing GameState,
# so no extra colour check is needed here.
_PHASE_MAP = {
    GameState.HALTED:          GamePhase.HALTED,
    GameState.HALF_TIME:       GamePhase.HALF_TIME,
    GameState.STOPPED:         GamePhase.STOPPED,
    GameState.PREPARE_KICKOFF: GamePhase.PREPARE_KICKOFF,
    GameState.OPP_KICKOFF:    GamePhase.OPP_KICKOFF,
    GameState.KICKOFF:         GamePhase.KICKOFF,
    GameState.FREE_KICK:       GamePhase.FREE_KICK,
    GameState.OPP_FREE_KICK:   GamePhase.OPP_FREE_KICK,
    GameState.BALL_PLACEMENT:  GamePhase.BALL_PLACEMENT,
    GameState.PREPARE_PENALTY:     GamePhase.PREPARE_PENALTY,
    GameState.PREPARE_PENALTY_OPP: GamePhase.PREPARE_PENALTY_OPP,
    GameState.PENALTY_SHOOT:       GamePhase.PENALTY_SHOOT,
    GameState.PENALTY_DEFEND:      GamePhase.PENALTY_DEFEND,
    GameState.RUNNING:         GamePhase.RUNNING,
}


def _phase_from_state(state) -> GamePhase:
    if state is None:
        return GamePhase.RUNNING  # sandbox / no-GC default
    return _PHASE_MAP.get(state, GamePhase.RUNNING)


def _ball_pos_vel(frame) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return (ball_position, ball_velocity) tuples from the latest frame.

    Velocity is not currently estimated by ``Frame``/``Ball``; emit (0,0) and
    leave the proper estimator hookup as a follow-up.
    """
    ball = frame.ball if frame is not None else None
    if ball is None:
        return (0.0, 0.0), (0.0, 0.0)
    # Vision protocol sends positions in mm; BT uses metres.
    return (float(ball.x) / 1000.0, float(ball.y) / 1000.0), (0.0, 0.0)


def _team_to_states(team) -> tuple[RobotState, ...]:
    out: list[RobotState] = []
    for robot in team:  # iteration only yields active Robot instances
        out.append(
            RobotState(
                robot_id=int(robot.id),
                # Vision protocol sends positions in mm; BT uses metres.
                position=(float(robot.x) / 1000.0, float(robot.y) / 1000.0),
                orientation=float(robot.o),
            )
        )
    return tuple(out)


def build_snapshot_from_world_model(wm) -> Snapshot | None:
    """Build a frozen ``Snapshot`` from the latest data in ``WorldModel``.

    Returns ``None`` when no vision frame has been received yet — callers
    should skip the tick in that case.
    """
    frame = wm.get_latest_frame()
    if frame is None:
        return None

    ball_pos, ball_vel = _ball_pos_vel(frame)

    us_yellow = wm.us_yellow()
    own_team = frame.robots_yellow if us_yellow else frame.robots_blue
    opp_team = frame.robots_blue if us_yellow else frame.robots_yellow

    raw_placement = wm.get_ball_placement_pos()
    placement_pos = (float(raw_placement[0]), float(raw_placement[1])) if raw_placement else None

    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=ball_vel,
        own_robots=_team_to_states(own_team),
        opponent_robots=_team_to_states(opp_team),
        referee_state=RefereeState(
            game_phase=_phase_from_state(wm.get_game_state()),
            score=(0, 0),  # TODO: read from wm.ref_data once exposed
            ball_placement_pos=placement_pos,
        ),
    )


def _robot_id_of(intent: Intent, fallback: int) -> int:
    # Most intents do not carry a robot id; the caller already knows which
    # blackboard produced them. The Coordinator could be extended to carry
    # robot_id alongside the intent — for now the runner supplies it.
    return fallback


def intent_to_motion_target(
    intent: Intent, robot_id: int, snapshot: Snapshot
) -> MotionTarget | None:
    """Resolve an ``Intent`` to a ``MotionTarget`` via the skill layer.

    Returns ``None`` when the robot is absent from the snapshot or when the
    intent kind has no skill mapping yet.
    """
    try:
        if isinstance(intent, IntentMove):
            return move_to(
                snapshot,
                robot_id,
                intent.target_pos,
                intent.target_orientation,
                intent.max_speed,
            )
        if isinstance(intent, IntentKick):
            return kick_at(snapshot, robot_id, intent.target_pos)
        if isinstance(intent, IntentDribble):
            # No dribble skill yet — drive toward the target like move_to.
            return move_to(snapshot, robot_id, intent.target_pos, None)
        if isinstance(intent, IntentPass):
            return kick_at(snapshot, robot_id, intent.target_pos)
        if isinstance(intent, IntentOrient):
            return MotionTarget(
                target_velocity=(0.0, 0.0),
                target_orientation=float(intent.target_orientation),
                arrival_mode="precision",
            )
        if isinstance(intent, IntentReceive):
            return MotionTarget(
                target_velocity=(0.0, 0.0),
                target_orientation=0.0,
                arrival_mode="normal",
            )
    except ValueError:
        # Robot absent from snapshot — skip the tick for it.
        return None
    return None


def _angular_velocity_to_target(
    current_orientation: float, target_orientation: float, gain: float = 4.0
) -> float:
    """Wrap angle error to [-pi, pi] and apply a proportional gain."""
    err = (target_orientation - current_orientation + math.pi) % (2 * math.pi) - math.pi
    return float(max(-6.0, min(6.0, err * gain)))


def intent_to_robot_command(
    intent: Intent,
    robot_id: int,
    snapshot: Snapshot,
    is_yellow: bool,
) -> RobotCommand | None:
    """End-to-end: Intent → MotionTarget → RobotCommand wire packet."""
    target = intent_to_motion_target(intent, robot_id, snapshot)
    if target is None:
        return None

    # Pull the current orientation so we can compute an angular velocity.
    robot = next(
        (r for r in snapshot.own_robots if r.robot_id == robot_id), None
    )
    current_o = robot.orientation if robot is not None else 0.0
    w = _angular_velocity_to_target(current_o, target.target_orientation)

    # Rotate world-frame velocity into robot-local frame.
    # grSim expects veltangent/velnormal in the robot's own coordinate frame.
    vx_w, vy_w = target.target_velocity
    cos_o = math.cos(current_o)
    sin_o = math.sin(current_o)
    vx = vx_w * cos_o + vy_w * sin_o
    vy = -vx_w * sin_o + vy_w * cos_o

    kick = 1 if isinstance(intent, (IntentKick, IntentPass)) else 0
    dribble = 1 if isinstance(intent, IntentDribble) else 0

    return RobotCommand(
        robot_id=robot_id,
        vx=float(vx),
        vy=float(vy),
        w=float(w),
        kick=kick,
        dribble=dribble,
        isYellow=bool(is_yellow),
    )


def dispatch_coordinator_output(
    coordinator,
    robot_ids: Iterable[int],
    snapshot: Snapshot,
    is_yellow: bool,
    dispatcher_q,
    run_time: float = 1.0,
) -> int:
    """Walk each robot's blackboard after a ``Coordinator.tick`` and emit a
    ``RobotCommand`` per non-empty intent.

    Reading the intent off the per-robot blackboard is more robust than
    aligning the ``Coordinator.tick`` list with ``robot_ids``: skipped robots
    (absent from snapshot, or which produced no intent) leave gaps in the
    returned list that are easy to misalign.
    """
    sent = 0
    for rid in robot_ids:
        bb = coordinator.blackboards.get(rid)
        if bb is None or bb.current_intent is None:
            continue
        cmd = intent_to_robot_command(bb.current_intent, rid, snapshot, is_yellow)
        if cmd is None:
            continue
        if not dispatcher_q.full():
            dispatcher_q.put([cmd, run_time])
            sent += 1
    return sent
