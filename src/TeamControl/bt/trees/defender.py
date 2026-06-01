"""Defender behaviour tree — R006.

Topology (from docs/defending_node.png):

    DefendingSequenceNode (Sequence)
    ├── LookAtBall         → stashes angle_to_ball on the tree (no intent)
    ├── DefendZoneFallback (Selector — OR logic)
    │   ├── InDefendingZone  → IntentMove(hold pos, facing ball) when x<0
    │   └── GoToDefendZone   → IntentMove(defend_zone, facing ball) otherwise
    └── ChallengeSequence (Sequence)
        ├── IsCloseEnough    (Condition — reads Snapshot)
        └── ClearBall        → writes IntentKick(clear_direction)

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
from TeamControl.bt.contracts.intent import IntentKick, IntentMove
from TeamControl.bt.contracts.snapshot import Snapshot

# -----------------------------------------------------------------------
# Tuneable constants
# -----------------------------------------------------------------------

DEFEND_ZONE_POSITION: tuple[float, float] = (-3.0, 0.0)   # own half centre
CLOSE_ENOUGH_THRESHOLD: float = 0.6                         # metres to ball to challenge
CLEAR_DIRECTION: tuple[float, float] = (4.5, 0.0)           # kick toward opponent's half
DEFENDER_ROLE_ID: int = 1                                    # defender robot ID


# -----------------------------------------------------------------------
# Condition / action nodes
# -----------------------------------------------------------------------

class LookAtBall(py_trees.behaviour.Behaviour):
    """Compute the angle from defender to ball and stash it on the tree.

    Does NOT write an intent — earlier versions wrote IntentOrient here, which
    clobbered every downstream movement intent (the adapter translates
    IntentOrient as zero linear velocity). Subsequent movement nodes pick up
    this angle via ``self._tree.look_angle`` and apply it as the target
    orientation on their IntentMove.
    """

    def __init__(self, tree_ref: DefenderTree) -> None:
        super().__init__("LookAtBall")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE

        robot = _find_robot(snap, bb.robot_id)
        if robot is None:
            return py_trees.common.Status.FAILURE

        self._tree.look_angle = math.atan2(
            snap.ball_position[1] - robot.position[1],
            snap.ball_position[0] - robot.position[0],
        )
        return py_trees.common.Status.SUCCESS


class InDefendingZone(py_trees.behaviour.Behaviour):
    """Hold current position facing the ball when defender is in its own half.

    If x < 0 (in zone): write IntentMove(target=current_pos, orientation=look_angle)
    so the defender stops in place but stays oriented toward the ball, and
    return SUCCESS to short-circuit the selector.
    If x >= 0 (out of zone): return FAILURE so GoToDefendZone runs next.
    """

    def __init__(self, tree_ref: DefenderTree) -> None:
        super().__init__("InDefendingZone")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE

        robot = _find_robot(snap, bb.robot_id)
        if robot is None:
            return py_trees.common.Status.FAILURE

        # "In zone" means we're on our own half. Under main's convention,
        # us_positive=True ⇒ own half is +x; us_positive=False ⇒ own half is -x.
        in_zone = (
            robot.position[0] > 0.0
            if self._tree.us_positive
            else robot.position[0] < 0.0
        )
        if in_zone:
            bb.current_intent = IntentMove(
                target_pos=robot.position,
                target_orientation=self._tree.look_angle,
            )
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class GoToDefendZone(py_trees.behaviour.Behaviour):
    """Write IntentMove toward the defend zone and return SUCCESS.

    Returning SUCCESS allows the parent Selector (DefendZoneFallback) to
    succeed, which lets the root Sequence proceed to ChallengeSequence.
    """

    def __init__(self, tree_ref: DefenderTree) -> None:
        super().__init__("GoToDefendZone")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        bb = self._tree._blackboard_ref[0]
        if bb is None:
            return py_trees.common.Status.FAILURE

        bb.current_intent = IntentMove(
            target_pos=self._tree.defend_zone_position,
            target_orientation=self._tree.look_angle,
        )
        return py_trees.common.Status.SUCCESS


class IsCloseEnough(py_trees.behaviour.Behaviour):
    """Succeed when the distance from robot to ball is within CLOSE_ENOUGH_THRESHOLD."""

    def __init__(self, tree_ref: DefenderTree) -> None:
        super().__init__("IsCloseEnough")
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
        if dist <= CLOSE_ENOUGH_THRESHOLD:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class ClearBall(py_trees.behaviour.Behaviour):
    """Write IntentKick toward CLEAR_DIRECTION and return SUCCESS."""

    def __init__(self, tree_ref: DefenderTree) -> None:
        super().__init__("ClearBall")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        bb = self._tree._blackboard_ref[0]
        if bb is None:
            return py_trees.common.Status.FAILURE

        bb.current_intent = IntentKick(target_pos=self._tree.clear_direction)
        return py_trees.common.Status.SUCCESS


# -----------------------------------------------------------------------
# DefenderTree
# -----------------------------------------------------------------------

class DefenderTree:
    """Wrapper around the Defender py_trees topology.

    Usage::

        tree = DefenderTree()
        tree.set_snapshot(snapshot)   # inject world state
        tree.tick(blackboard)          # run tree; writes Intent to blackboard
        intent = blackboard.current_intent
    """

    def __init__(self, us_positive: bool = True) -> None:
        self._snapshot: Snapshot | None = None
        # Shared mutable ref — nodes read the current blackboard without
        # being reconstructed each tick.
        self._blackboard_ref: list = [None]
        # Updated each tick by LookAtBall; consumed by movement nodes as the
        # target_orientation for their IntentMove. 0.0 is a safe initial value
        # in case a movement node runs before LookAtBall (shouldn't happen
        # given the sequence order, but defensive).
        self.look_angle: float = 0.0
        # Mirror side-dependent constants onto the half we're actually
        # defending. Module constants assume us_positive=True (own goal at
        # negative x). When we attack the negative half instead, negate x so
        # the defender parks in OUR half and clears toward the OPPONENT goal.
        self.us_positive = us_positive
        # Convention: us_positive=True means we are on +x; own half is +x,
        # opp goal is at -x. The module constants are authored for the
        # us_positive=False case, so negate x when us_positive=True.
        self.defend_zone_position: tuple[float, float] = (
            (-DEFEND_ZONE_POSITION[0], DEFEND_ZONE_POSITION[1]) if us_positive
            else DEFEND_ZONE_POSITION
        )
        self.clear_direction: tuple[float, float] = (
            (-CLEAR_DIRECTION[0], CLEAR_DIRECTION[1]) if us_positive
            else CLEAR_DIRECTION
        )
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
        # DefendZoneFallback — Selector: succeed if in zone, else move to zone.
        defend_zone_fallback = py_trees.composites.Selector(
            name="DefendZoneFallback", memory=False
        )
        defend_zone_fallback.add_children([
            InDefendingZone(self),
            GoToDefendZone(self),
        ])

        # ChallengeSequence — Sequence: only clear ball if close enough.
        challenge_sequence = py_trees.composites.Sequence(
            name="ChallengeSequence", memory=False
        )
        challenge_sequence.add_children([
            IsCloseEnough(self),
            ClearBall(self),
        ])

        # Root: DefendingSequenceNode
        root = py_trees.composites.Sequence(
            name="DefendingSequenceNode", memory=False
        )
        root.add_children([
            LookAtBall(self),
            defend_zone_fallback,
            challenge_sequence,
        ])
        return root


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------

def _find_robot(snap: Snapshot, robot_id: int):
    for r in snap.own_robots:
        if r.robot_id == robot_id:
            return r
    return None
