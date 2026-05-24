"""Coordinator — role assignment and per-robot tree dispatch.

R004: The Coordinator is the entry point for the decision pipeline each tick.
It assigns a fixed role to each robot by robot ID, selects the matching
py_trees tree, injects the per-robot blackboard, ticks the tree, and
collects the resulting Intent.

Pipeline:
    Snapshot → Coordinator.tick() → list[Intent]

The Coordinator output is exclusively a list of Intent variants, one per
robot present in the snapshot. Raw motor commands are never produced here.

Tree dispatch protocol
----------------------
The Coordinator supports two tree interfaces:

1. **Tree wrapper** (preferred) — any object that exposes both
   ``set_snapshot(snapshot)`` and ``tick(blackboard)``.  This is the
   standard interface implemented by ``AttackerTree``, ``DefenderTree``,
   ``SupporterTree``, and ``GoalieTree``.

2. **Raw py_trees node** (legacy/testing) — any object with
   ``tick_once()``.  The Coordinator injects the blackboard via
   ``_blackboard_ref[0]`` before calling ``tick_once()``.

The wrapper protocol is detected first via ``hasattr`` checks.
"""
from __future__ import annotations

from typing import Any

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import Intent
from TeamControl.bt.contracts.snapshot import Snapshot

# Fixed role assignment by robot list index.
# index 0 → GOALIE, 1-2 → DEFENDER, 3-4 → SUPPORTER, 5 → ATTACKER
ROLE_ASSIGNMENT: dict[int, RoleType] = {
    0: RoleType.GOALIE,
    1: RoleType.DEFENDER,
    2: RoleType.DEFENDER,
    3: RoleType.SUPPORTER,
    4: RoleType.SUPPORTER,
    5: RoleType.ATTACKER,
}


class Coordinator:
    """Assigns roles and dispatches role trees each tick.

    Parameters
    ----------
    trees:
        Mapping of RoleType → tree object.  Each tree is instantiated once
        (by the caller) and reused across ticks.  Tree objects should
        implement the wrapper protocol: ``set_snapshot(snapshot)`` and
        ``tick(blackboard)``.  Raw py_trees nodes are also accepted for
        backwards compatibility (see module docstring).
    """

    def __init__(
        self,
        trees: dict[RoleType, Any],
        role_assignment: dict[int, RoleType] | None = None,
    ) -> None:
        self.trees = trees
        # Per-robot blackboards — created lazily on first tick for each robot.
        self.blackboards: dict[int, RobotBlackboard] = {}
        # Per-instance role map; defaults to the module-level ROLE_ASSIGNMENT
        # so existing single-team callers keep working.
        self.role_assignment: dict[int, RoleType] = (
            dict(role_assignment) if role_assignment is not None else dict(ROLE_ASSIGNMENT)
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tick(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """Tick all robots present in the snapshot.

        Parameters
        ----------
        snapshot:
            Read-only world state for this tick.
        robot_ids:
            Robot IDs to process this tick. Role is assigned by robot_id
            (see ROLE_ASSIGNMENT); list order does not affect assignment.

        Returns
        -------
        list[Intent]
            One Intent per robot that was present in the snapshot.
            Robots absent from the snapshot are silently skipped.
        """
        snapshot_ids: set[int] = {r.robot_id for r in snapshot.own_robots}
        intents: list[Intent] = []

        for robot_id in robot_ids:
            if robot_id not in snapshot_ids:
                # Robot absent from snapshot — skip gracefully.
                continue

            role = self.role_assignment.get(robot_id, RoleType.SUPPORTER)

            # Create or update the per-robot blackboard.
            if robot_id not in self.blackboards:
                self.blackboards[robot_id] = RobotBlackboard(
                    robot_id=robot_id,
                    current_role=role,
                )
            bb = self.blackboards[robot_id]

            # Shift intent history before the new tick.
            bb.last_intent = bb.current_intent
            bb.current_intent = None
            bb.current_role = role

            # Dispatch the tree using whichever protocol it exposes.
            tree = self.trees[role]
            if hasattr(tree, "set_snapshot") and hasattr(tree, "tick"):
                # Tree wrapper protocol (AttackerTree, DefenderTree, etc.)
                tree.set_snapshot(snapshot)  # type: ignore[union-attr]
                tree.tick(bb)               # type: ignore[union-attr]
            else:
                # Raw py_trees node — inject blackboard ref then tick.
                if hasattr(tree, "_blackboard_ref"):
                    tree._blackboard_ref[0] = bb  # type: ignore[attr-defined]
                tree.tick_once()            # type: ignore[union-attr]

            # Collect the intent produced this tick.
            if bb.current_intent is not None:
                intents.append(bb.current_intent)  # type: ignore[arg-type]

        return intents
