"""Attacker behaviour tree — R005.

Topology (current):

    AttackingSelector (Selector, memory=False)
    ├── ShootSequence   (HasBallControl → HasClearShot → ShootAtGoal)
    ├── DribbleSequence (HasBallControl → HoldPossession[IntentDribble])
    └── ChaseBall       (IntentMove(ball_position))

The tree asks two predicates each tick:
    Q1: HasBallControl — is the ball within POSSESSION_DIST AND in front
                         of the kicker (heading aligned within
                         POSSESSION_HEADING_TOL)?
    Q2: HasClearShot   — is the ball→goal line free of opponents?
And acts on the answers:
    no possession             → ChaseBall
    possession + clear shot   → ShootAtGoal (IntentKick to goal)
    possession + blocked shot → HoldPossession (dribble toward goal)

The original pass branch (PassOrPlaySequence) and the (now-unused)
IsBallInRangeOrMove node are kept in the source as commented references
for future re-enablement once a real "passing" play is added.

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

BALL_IN_RANGE_THRESHOLD: float = 0.8   # metres — legacy chase threshold (unused by new tree)
SUPPORTER_ROLE_IDS: tuple[int, ...] = (3, 4)  # robot IDs with SUPPORTER role
GOAL_POSITION: tuple[float, float] = (4.5, 0.0)   # opponent goal centre

# Distance at which we consider the ball to be in the dribbler (we "have" it).
# SSL robot diameter ~0.18 m, ball ~0.043 m → centre-to-centre ~0.11 m when
# touching. 0.15 m gives a small margin so the predicate doesn't flicker.
POSSESSION_DIST: float = 0.15

# Maximum angular error between the robot's heading and the direction to the
# ball before we say we "have" the ball. The kicker plate is on the front of
# the robot — if the ball is behind us (or to the side), a kick command fires
# into empty space. ~17° tolerance lets us claim possession when the ball is
# roughly in front but not perfectly centred.
POSSESSION_HEADING_TOL: float = 0.3   # radians (~17 degrees)

# Half-width of the shooting corridor: an opponent within this perpendicular
# distance of the ball→goal line segment is considered to be blocking the shot.
# Robot radius is ~0.09 m, so 0.20 m means an opponent within ~one body length
# of the line counts as a block.
SHOT_CORRIDOR_RADIUS: float = 0.20


# -----------------------------------------------------------------------
# Condition / action nodes
# -----------------------------------------------------------------------

class HasBallControl(py_trees.behaviour.Behaviour):
    """Succeed when the ball is close to the robot AND in front of the kicker.

    Two checks:
      1. ``dist(robot, ball) <= POSSESSION_DIST`` — ball is within dribbler range.
      2. ``|angle(robot→ball) - robot.orientation| <= POSSESSION_HEADING_TOL`` —
         ball is in front of the kicker plate. Without this check, the tree
         would issue IntentKick while the ball is behind/beside the robot and
         the kick command would punch air with the ball untouched.
    """

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("HasBallControl")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE

        robot = _find_robot(snap, bb.robot_id)
        if robot is None:
            return py_trees.common.Status.FAILURE

        dx = snap.ball_position[0] - robot.position[0]
        dy = snap.ball_position[1] - robot.position[1]
        dist = math.hypot(dx, dy)
        if dist > POSSESSION_DIST:
            return py_trees.common.Status.FAILURE

        # Heading: ball must be in front of the kicker.
        angle_to_ball = math.atan2(dy, dx)
        err = (angle_to_ball - robot.orientation + math.pi) % (2 * math.pi) - math.pi
        if abs(err) > POSSESSION_HEADING_TOL:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS


class HasClearShot(py_trees.behaviour.Behaviour):
    """Succeed when no opponent obstructs the line from the ball to the goal.

    For each opponent, compute the perpendicular distance from their position
    to the ball→goal line segment. If every opponent is farther than
    ``SHOT_CORRIDOR_RADIUS`` from the segment, the shot is considered clear.
    Empty opponent list → trivially clear.
    """

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("HasClearShot")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        if snap is None:
            return py_trees.common.Status.FAILURE

        ball = snap.ball_position
        goal = self._tree.goal_position
        for opp in snap.opponent_robots:
            if _point_to_segment_dist(opp.position, ball, goal) <= SHOT_CORRIDOR_RADIUS:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class ChaseBall(py_trees.behaviour.Behaviour):
    """Write IntentMove(target=ball_position, orientation=toward_ball). SUCCESS.

    Fallback when we don't have ball control. Crucially, this sets
    ``target_orientation`` to the robot→ball bearing so the robot rotates to
    face the ball as it closes in. Without this, the move_to skill defaults
    target_orientation to 0.0 — the robot stays pointed east, holonomically
    drifts toward the ball with the ball arriving behind/beside the kicker,
    and HasBallControl never succeeds because its heading check fails.
    """

    def __init__(self, tree_ref: AttackerTree) -> None:
        super().__init__("ChaseBall")
        self._tree = tree_ref

    def update(self) -> py_trees.common.Status:
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE
        robot = _find_robot(snap, bb.robot_id)
        if robot is None:
            return py_trees.common.Status.FAILURE
        angle_to_ball = math.atan2(
            snap.ball_position[1] - robot.position[1],
            snap.ball_position[0] - robot.position[0],
        )
        bb.current_intent = IntentMove(
            target_pos=snap.ball_position,
            target_orientation=angle_to_ball,
        )
        bb.intent_source = "ChaseBall"
        return py_trees.common.Status.SUCCESS


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
        bb.intent_source = "IsBallInRangeOrMove"
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
                bb.intent_source = "PassToSupporter"
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
        bb.current_intent = IntentDribble(target_pos=self._tree.goal_position)
        bb.intent_source = "HoldPossession"
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
        bb.current_intent = IntentKick(target_pos=self._tree.goal_position)
        bb.intent_source = "ShootAtGoal"
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

    def __init__(self, us_positive: bool = True) -> None:
        self._snapshot: Snapshot | None = None
        # Shared mutable ref — nodes read the current blackboard without
        # being reconstructed each tick.
        self._blackboard_ref: list = [None]
        # Convention: us_positive=True means we are on +x, so the opponent
        # goal is at -x. GOAL_POSITION = (4.5, 0) is the un-mirrored "opp
        # goal" used when us_positive=False; negate x when us_positive=True.
        self.us_positive = us_positive
        self.goal_position: tuple[float, float] = (
            (-GOAL_POSITION[0], GOAL_POSITION[1]) if us_positive
            else GOAL_POSITION
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

    def _build_tree(self) -> py_trees.composites.Selector:
        # New topology — asks two questions and acts on the answers:
        #   Q1: do I have ball control?
        #   Q2: if yes, is the shot clear?
        #
        # AttackingSelector (Selector)
        # ├── ShootSequence  : HasBallControl → HasClearShot → ShootAtGoal
        # ├── DribbleSequence: HasBallControl → HoldPossession
        # └── ChaseBall      : IntentMove(ball) — no possession yet
        #
        # Selector with memory=False re-evaluates every tick. As soon as we
        # have possession AND a clean shot, ShootSequence wins. If we lose
        # the ball, we fall straight to ChaseBall the next tick.

        # ShootSequence — fires only with possession and clear line to goal.
        shoot_seq = py_trees.composites.Sequence(
            name="ShootSequence", memory=False
        )
        shoot_seq.add_children([
            HasBallControl(self),
            HasClearShot(self),
            ShootAtGoal(self),
        ])

        # DribbleSequence — possession but no clear shot: carry toward goal.
        dribble_seq = py_trees.composites.Sequence(
            name="DribbleSequence", memory=False
        )
        dribble_seq.add_children([
            HasBallControl(self),
            HoldPossession(self),
        ])

        # PassOrPlaySequence — TEMPORARILY DISABLED (was causing an
        # oscillation around BALL_IN_RANGE_THRESHOLD). Re-enable when ready
        # to use real pass logic gated by HasBallControl.
        # pass_or_play = py_trees.composites.Sequence(
        #      name="PassOrPlaySequence", memory=False
        # )
        # pass_or_play.add_children([
        #     IsSupporterAvailable(self),
        #     PassToSupporter(self),
        # ])

        # Root: AttackingSelector
        root = py_trees.composites.Selector(
            name="AttackingSelector", memory=False
        )
        root.add_children([
            shoot_seq,
            # pass_or_play,
            dribble_seq,
            ChaseBall(self),
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


def _point_to_segment_dist(
    point: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
) -> float:
    """Shortest distance from *point* to the line segment A→B."""
    ax, ay = seg_a
    bx, by = seg_b
    px, py = point
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab_len_sq))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return math.hypot(px - closest_x, py - closest_y)
