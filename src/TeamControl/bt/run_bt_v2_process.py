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


def _build_coordinator(role_assignment: dict[int, RoleType] | None = None) -> Coordinator:
    return Coordinator(
        trees={
            RoleType.GOALIE: GoalieTree(),
            RoleType.DEFENDER: DefenderTree(),
            RoleType.SUPPORTER: SupporterTree(),
            RoleType.ATTACKER: AttackerTree(),
        },
        role_assignment=role_assignment,
    )


def run_bt_v2_process(
    is_running: Event,
    wm: WorldModel,
    dispatcher_q: Queue,
    is_yellow: bool | None = None,
    robot_ids: list[int] | None = None,
    role_assignment: dict[int, RoleType] | None = None,
    tick_period: float = TICK_PERIOD,
) -> None:
    """Tick the v2 (TurtleRabbitBT) coordinator in a child process.

    Args:
        is_running: shared Event — clear to stop the loop.
        wm: shared WorldModel proxy.
        dispatcher_q: queue consumed by the dispatcher; items are
            ``[RobotCommand, run_time_seconds]``.
        is_yellow: team perspective for this BT instance. ``None`` falls
            back to ``wm.us_yellow()`` (single-team mode). For 6v6 spawn
            two processes and pass ``True`` and ``False`` explicitly.
        robot_ids: which robot ids to tick. Defaults to 0..5.
        role_assignment: per-robot role override; defaults to the
            module-level ``ROLE_ASSIGNMENT`` in ``coordinator.py``.
        tick_period: seconds to sleep between ticks.
    """
    if robot_ids is None:
        robot_ids = DEFAULT_ROBOT_IDS

    coordinator = _build_coordinator(role_assignment)
    if is_yellow is None:
        is_yellow = bool(wm.us_yellow())
    is_yellow = bool(is_yellow)

    while is_running.is_set():
        snapshot = build_snapshot_from_world_model(wm, is_yellow=is_yellow)
        if snapshot is None:
            time.sleep(tick_period)
            continue

        coordinator.tick(snapshot, robot_ids)
        dispatch_coordinator_output(
            coordinator,
            robot_ids,
            snapshot,
            is_yellow,
            dispatcher_q,
        )
        time.sleep(tick_period)
