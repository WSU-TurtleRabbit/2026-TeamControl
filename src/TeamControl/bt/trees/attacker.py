"""Attacker behaviour tree — R005.

Topology (from docs/attacking_node.png):

    AttackingSequenceNode (Sequence)
    ├── GoToBallSequence (Sequence)
    │   └── IsBallInRangeOrMove  (combined condition+action)
    │         SUCCESS → ball is in range, proceed
    │         FAILURE → writes IntentMove(ball_pos) and fails root
    └── PassPlaySelector (Selector)
        ├── PassOrPlaySequence (Sequence)
        │   ├── IsSupporterAvailable (Condition)
        │   └── PassToSupporter      (Action — writes IntentPass)
        ├── HoldPossession           (Action — writes IntentDribble)
        └── ShootAtGoal              (Action — writes IntentKick)

Design notes
------------
- Snapshot is injected via ``set_snapshot()`` before each ``tick()``.
  All condition and action nodes access world state through
  ``self._tree._snapshot`` (read-only Snapshot reference).
- Blackboard is injected via ``tick(blackboard)`` using the standard
  ``_blackboard_ref`` protocol (one-element list). Nodes write
  ``_blackboard_ref[0].current_intent`` to produce their output.
- No raw motor commands are produced anywhere in this module.
"""
from __future__ import annotations

import math
import py_trees

from TeamControl.bt.contracts.blackboard import RobotBlackboard
from TeamControl.bt.contracts.intent import IntentDribble, IntentKick, IntentMove, IntentPass
from TeamControl.bt.contracts.snapshot import Snapshot

# -----------------------------------------------------------------------
# Tuneable constants
# -----------------------------------------------------------------------

BALL_IN_RANGE_THRESHOLD: float = 0.8   # metres
SUPPORTER_ROLE_IDS: tuple[int, ...] = (3, 4)  # robot IDs with SUPPORTER role
GOAL_POSITION: tuple[float, float] = (4.5, 0.0)   # opponent goal centre


# -----------------------------------------------------------------------
# Condition / action nodes
# -----------------------------------------------------------------------

class IsBallInRangeOrMove(py_trees.behaviour.Behaviour):
    """Check if ball is within range; write IntentMove and fail if not.

    SUCCESS → attacker is close enough to the ball; proceed to PassPlaySelector.
    FAILURE → attacker is too far; writes IntentMove(ball_position) so the
              caller knows to move, then returns FAILURE to halt the root Sequence.
    """

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("IsBallInRange")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE

        robot = _find_robot(snap, bb.robot_id)
        if robot is None:
            return py_trees.common.Status.FAILURE

        dist = math.hypot(
            snap.ball_position[0] - robot.position[0],
            snap.ball_position[1] - robot.position[1],
        )
        if dist <= BALL_IN_RANGE_THRESHOLD:
            return py_trees.common.Status.SUCCESS

        # Ball out of range — write move intent and signal failure so the
        # root Sequence stops (PassPlaySelector is not reached).
        bb.current_intent = IntentMove(
            target_pos=snap.ball_position,
            target_orientation=None,
        )
        return py_trees.common.Status.FAILURE


class IsSupporterAvailable(py_trees.behaviour.Behaviour):
    """Succeed when at least one supporter robot is in the snapshot."""

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("IsSupporterAvailable")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        if snap is None:
            return py_trees.common.Status.FAILURE
        for robot in snap.own_robots:
            if robot.robot_id in SUPPORTER_ROLE_IDS:
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PassToSupporter(py_trees.behaviour.Behaviour):
    """Write IntentPass targeting the first available supporter."""

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("PassToSupporter")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE
        for robot in snap.own_robots:
            if robot.robot_id in SUPPORTER_ROLE_IDS:
                bb.current_intent = IntentPass(
                    target_robot_id=robot.robot_id,
                    target_pos=robot.position,
                )
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class HoldPossession(py_trees.behaviour.Behaviour):
    """Write IntentDribble toward goal when no pass is available."""

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("HoldPossession")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        bb = self._tree._blackboard_ref[0]
        if bb is None:
            return py_trees.common.Status.FAILURE
        bb.current_intent = IntentDribble(target_pos=GOAL_POSITION)
        return py_trees.common.Status.SUCCESS


class ShootAtGoal(py_trees.behaviour.Behaviour):
    """Write IntentKick toward goal as the last-resort fallback."""

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("ShootAtGoal")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        bb = self._tree._blackboard_ref[0]
        if bb is None:
            return py_trees.common.Status.FAILURE
        bb.current_intent = IntentKick(target_pos=GOAL_POSITION)
        return py_trees.common.Status.SUCCESS


# -----------------------------------------------------------------------
# AttackerTree
# -----------------------------------------------------------------------

class AttackerTree:
    """Wrapper around the Attacker py_trees topology.

    Usage::

        tree = AttackerTree()
        tree.set_snapshot(snapshot)   # inject world state
        tree.tick(blackboard)          # run tree; writes Intent to blackboard
        intent = blackboard.current_intent
    """

    def __init__(self) -> None:
        self._snapshot: Snapshot | None = None
        # Shared mutable ref — nodes read the current blackboard without
        # being reconstructed each tick.
        self._blackboard_ref: list = [None]
        self.root = self._build_tree()

    # ------------------------------------------------------------------

    def set_snapshot(self, snapshot: Snapshot) -> None:
        """Inject the current world-state snapshot before ticking."""
        self._snapshot = snapshot

    def tick(self, blackboard: RobotBlackboard) -> None:
        """Tick the tree with the given per-robot blackboard.

        After this call, ``blackboard.current_intent`` contains the Intent
        produced for this tick.
        """
        self._blackboard_ref[0] = blackboard
        self.root.tick_once()

    # ------------------------------------------------------------------

    def _build_tree(self) -> py_trees.composites.Sequence:
        # GoToBallSequence — Sequence with one combined condition+action child.
        # If ball is in range: child returns SUCCESS → Sequence succeeds → proceed.
        # If ball out of range: child writes IntentMove and returns FAILURE →
        # Sequence fails → root Sequence stops (intent already written).
        go_to_ball = py_trees.composites.Sequence(
            name="GoToBallSequence", memory=False
        )
        go_to_ball.add_child(IsBallInRangeOrMove(self))

        # PassOrPlaySequence — check for supporter first, then pass.
        # TEMPORARILY DISABLED: the pass branch caused an oscillation around
        # the BALL_IN_RANGE_THRESHOLD ring (attacker would enter range, get
        # told to drive at a supporter, exit range, get told to chase ball,
        # repeat — never touching the ball). Re-enable once we have a real
        # "has ball" predicate to gate possession-vs-pursuit.
        # pass_or_play = py_trees.composites.Sequence(
        #      name="PassOrPlaySequence", memory=False
        # )
        # pass_or_play.add_children([
        #     IsSupporterAvailable(self),
        #     PassToSupporter(self),
        # ])

        # PassPlaySelector — pass branch disabled; falls straight through to
        # HoldPossession (dribble toward goal). ShootAtGoal kept as a final
        # fallback even though HoldPossession always succeeds.
        pass_play_selector = py_trees.composites.Selector(
            name="PassPlaySelector", memory=False
        )
        pass_play_selector.add_children([
            # pass_or_play,
            HoldPossession(self),
            ShootAtGoal(self),
        ])

        # Root: AttackingSequenceNode
        root = py_trees.composites.Sequence(
            name="AttackingSequenceNode", memory=False
        )
        root.add_children([go_to_ball, pass_play_selector])
        return root


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------

def _find_robot(snap: Snapshot, robot_id: int):
    for r in snap.own_robots:
        if r.robot_id == robot_id:
            return r
    return None
