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
from TeamControl.bt.coordinator import Coordinator
from TeamControl.bt.trees.attacker import AttackerTree
from TeamControl.bt.trees.defender import DefenderTree
from TeamControl.bt.trees.goalie import GoalieTree
from TeamControl.bt.trees.supporter import SupporterTree
from TeamControl.world.model import WorldModel

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

    while is_running.is_set():
        snapshot = build_snapshot_from_world_model(wm)
        if snapshot is None:
            time.sleep(TICK_PERIOD)
            continue

        coordinator.tick(snapshot, robot_ids)
        dispatch_coordinator_output(
            coordinator,
            robot_ids,
            snapshot,
            is_yellow,
            dispatcher_q,
        )
        time.sleep(TICK_PERIOD)
