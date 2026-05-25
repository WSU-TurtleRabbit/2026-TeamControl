"""V2 behaviour-tree process runner.

Mirrors ``behaviour_tree/run_bt_process.py`` but drives the TurtleRabbitBT
Coordinator instead of the legacy ``MainTree``. Spawn this from
``SSL/grSim/sandbox.py`` (or any other harness) using
``multiprocessing.Process``.

Pipeline each tick:

    WorldModel  →  build_snapshot_from_world_model
                →  Coordinator.tick(snapshot, robot_ids)
                →  dispatch_coordinator_output → dispatcher_q
"""
from __future__ import annotations

import time
from multiprocessing import Event, Queue

from TeamControl.bt.adapter import (
    build_snapshot_from_world_model,
    dispatch_coordinator_output,
)
from TeamControl.bt.contracts.blackboard import RoleType
from TeamControl.bt.contracts.snapshot import GamePhase
from TeamControl.bt.coordinator import Coordinator
from TeamControl.bt.trees.attacker import AttackerTree
from TeamControl.bt.trees.defender import DefenderTree
from TeamControl.bt.trees.goalie import GoalieTree
from TeamControl.bt.trees.supporter import SupporterTree
from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.model import WorldModel

_HALT_PHASES = (GamePhase.HALTED, GamePhase.HALF_TIME)


def _send_stop_commands(
    robot_ids: list[int], is_yellow: bool, dispatcher_q: Queue
) -> None:
    """Send zero-velocity commands for every robot to override stale commands."""
    for rid in robot_ids:
        cmd = RobotCommand(robot_id=rid, vx=0.0, vy=0.0, w=0.0, isYellow=is_yellow)
        if not dispatcher_q.full():
            dispatcher_q.put([cmd, 0.1])

# Robot ids 0..5 — matches Coordinator.ROLE_ASSIGNMENT.
DEFAULT_ROBOT_IDS: list[int] = [0, 1, 2, 3, 4, 5]

# Target tick period in seconds (100 Hz).
TICK_PERIOD: float = 0.01


def _build_coordinator(us_positive: bool) -> Coordinator:
    return Coordinator(
        trees={
            RoleType.GOALIE: GoalieTree(),
            RoleType.DEFENDER: DefenderTree(),
            RoleType.SUPPORTER: SupporterTree(),
            RoleType.ATTACKER: AttackerTree(),
        },
        us_positive=us_positive,
    )


def run_bt_v2_process(
    is_running: Event,
    wm: WorldModel,
    dispatcher_q: Queue,
    robot_ids: list[int] | None = None,
) -> None:
    """Tick the v2 (TurtleRabbitBT) coordinator in a child process.

    Args:
        is_running: shared Event — clear to stop the loop.
        wm: shared WorldModel proxy.
        dispatcher_q: queue consumed by the dispatcher; items are
            ``[RobotCommand, run_time_seconds]``.
        robot_ids: which robot ids to tick this process. Defaults to 0..5.
    """
    if robot_ids is None:
        robot_ids = DEFAULT_ROBOT_IDS

    is_yellow = bool(wm.us_yellow())
    coordinator = _build_coordinator(us_positive=bool(wm.us_positive()))
    print(f"[BT] started — yellow={is_yellow}, robot_ids={robot_ids}")

    last_phase = None
    tick_count = 0

    try:
        while is_running.is_set():
            snapshot = build_snapshot_from_world_model(wm)
            if snapshot is None:
                time.sleep(TICK_PERIOD)
                continue

            phase = snapshot.referee_state.game_phase

            # Print whenever the game phase changes
            if phase != last_phase:
                print(f"[BT] phase changed: {last_phase} → {phase}", flush=True)
                last_phase = phase

            coordinator.tick(snapshot, robot_ids)

            # Print intents every 100 ticks (~1 sec)
            tick_count += 1
            if tick_count % 100 == 0:
                print(f"[BT] tick={tick_count} phase={phase.value}", flush=True)
                for rid in robot_ids:
                    bb = coordinator.blackboards.get(rid)
                    if bb and bb.current_intent:
                        print(f"  robot {rid} ({bb.current_role.value}): {bb.current_intent}", flush=True)

            if phase in _HALT_PHASES:
                _send_stop_commands(robot_ids, is_yellow, dispatcher_q)
            else:
                dispatch_coordinator_output(
                    coordinator,
                    robot_ids,
                    snapshot,
                    is_yellow,
                    dispatcher_q,
                )
            time.sleep(TICK_PERIOD)
    except KeyboardInterrupt:
        print("[BT] KeyboardInterrupt — exiting", flush=True)
