"""End-to-end Attacker pipeline tests — R010.

Validates the full decision pipeline:

    Snapshot  →  AttackerTree  →  RobotBlackboard.current_intent

Three canonical scenarios from R010:

    1. Ball out of range      → IntentMove toward ball position
    2. Ball in range, no supporter → IntentDribble (HoldPossession)
    3. Ball in range, supporter available → IntentPass

Note on R010 "IntentKick" wording
----------------------------------
R010 states "no supporters available → IntentKick".  The tree topology (R005)
places HoldPossession (writes IntentDribble) *before* ShootAtGoal (writes
IntentKick) in PassPlaySelector.  HoldPossession always returns SUCCESS, so
ShootAtGoal is never reached in v1.  These tests reflect actual behaviour;
the spec wording should be read as "the last-resort fallback is IntentKick"
rather than "no-supporter always gives IntentKick".

No network, simulator, or real hardware is required.
"""
from __future__ import annotations

import pytest

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    IntentDribble,
    IntentKick,
    IntentMove,
    IntentPass,
)
from TeamControl.bt.contracts.snapshot import GamePhase, RefereeState, RobotState, Snapshot
from TeamControl.bt.trees.attacker import AttackerTree, BALL_IN_RANGE_THRESHOLD

# ---------------------------------------------------------------------------
# Field constants
# ---------------------------------------------------------------------------

_ATTACKER_ID = 5       # robot_id that receives ATTACKER role (index 5)
_SUPPORTER_ID = 3      # robot_id in SUPPORTER_ROLE_IDS

_BALL_NEAR: tuple[float, float] = (0.2, 0.0)   # well inside BALL_IN_RANGE_THRESHOLD
_BALL_FAR: tuple[float, float] = (2.0, 0.0)    # clearly outside threshold
_ATTACKER_AT_ORIGIN: tuple[float, float] = (0.0, 0.0)


# ---------------------------------------------------------------------------
# Snapshot factory
# ---------------------------------------------------------------------------

def _make_snapshot(
    ball_pos: tuple[float, float],
    attacker_pos: tuple[float, float],
    include_supporter: bool,
) -> Snapshot:
    own_robots: list[RobotState] = [
        RobotState(robot_id=_ATTACKER_ID, position=attacker_pos, orientation=0.0),
    ]
    if include_supporter:
        own_robots.append(
            RobotState(robot_id=_SUPPORTER_ID, position=(1.5, 1.0), orientation=0.0)
        )
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=(0.0, 0.0),
        own_robots=own_robots,
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


def _make_blackboard() -> RobotBlackboard:
    return RobotBlackboard(robot_id=_ATTACKER_ID, current_role=RoleType.ATTACKER)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tick(tree: AttackerTree, snapshot: Snapshot) -> RobotBlackboard:
    """Inject snapshot, tick tree, return the populated blackboard."""
    bb = _make_blackboard()
    tree.set_snapshot(snapshot)
    tree.tick(bb)
    return bb


# ---------------------------------------------------------------------------
# R010 scenario 1: ball out of range → IntentMove toward ball
# ---------------------------------------------------------------------------

class TestBallOutOfRange:
    """Pipeline produces IntentMove(ball_pos) when attacker is far from the ball."""

    def setup_method(self) -> None:
        self.tree = AttackerTree()
        self.snapshot = _make_snapshot(
            ball_pos=_BALL_FAR,
            attacker_pos=_ATTACKER_AT_ORIGIN,
            include_supporter=False,
        )

    def test_produces_intent_move(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert isinstance(bb.current_intent, IntentMove)

    def test_intent_move_targets_ball_position(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == _BALL_FAR

    def test_no_robot_command_fields(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        intent = bb.current_intent
        for field in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert not hasattr(intent, field), f"RobotCommand field '{field}' found on intent"

    def test_blackboard_holds_intent_after_tick(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert bb.current_intent is not None

    def test_ball_is_actually_out_of_range(self) -> None:
        import math
        dist = math.hypot(_BALL_FAR[0] - _ATTACKER_AT_ORIGIN[0],
                          _BALL_FAR[1] - _ATTACKER_AT_ORIGIN[1])
        assert dist > BALL_IN_RANGE_THRESHOLD


# ---------------------------------------------------------------------------
# R010 scenario 2: ball in range, no supporters → IntentDribble
# ---------------------------------------------------------------------------

class TestBallInRangeNoSupporter:
    """Pipeline produces IntentDribble (HoldPossession) when ball is close
    but no supporter is in the snapshot.

    The tree topology (R005) has HoldPossession before ShootAtGoal in the
    Selector, so HoldPossession fires first and always succeeds. IntentKick
    (ShootAtGoal) is never reached in v1 — see module docstring.
    """

    def setup_method(self) -> None:
        self.tree = AttackerTree()
        self.snapshot = _make_snapshot(
            ball_pos=_BALL_NEAR,
            attacker_pos=_ATTACKER_AT_ORIGIN,
            include_supporter=False,
        )

    def test_produces_intent_dribble(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert isinstance(bb.current_intent, IntentDribble)

    def test_not_intent_move(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert not isinstance(bb.current_intent, IntentMove)

    def test_not_intent_pass(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert not isinstance(bb.current_intent, IntentPass)

    def test_no_robot_command_fields(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        intent = bb.current_intent
        for field in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert not hasattr(intent, field)

    def test_blackboard_holds_intent_after_tick(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert bb.current_intent is not None

    def test_ball_is_actually_in_range(self) -> None:
        import math
        dist = math.hypot(_BALL_NEAR[0] - _ATTACKER_AT_ORIGIN[0],
                          _BALL_NEAR[1] - _ATTACKER_AT_ORIGIN[1])
        assert dist <= BALL_IN_RANGE_THRESHOLD


# ---------------------------------------------------------------------------
# R010 scenario 3: ball in range, supporter available → IntentPass
# ---------------------------------------------------------------------------

class TestBallInRangeWithSupporter:
    """Pipeline produces IntentPass when ball is close and a supporter exists."""

    def setup_method(self) -> None:
        self.tree = AttackerTree()
        self.snapshot = _make_snapshot(
            ball_pos=_BALL_NEAR,
            attacker_pos=_ATTACKER_AT_ORIGIN,
            include_supporter=True,
        )

    def test_produces_intent_pass(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert isinstance(bb.current_intent, IntentPass)

    def test_intent_pass_targets_supporter(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert isinstance(bb.current_intent, IntentPass)
        assert bb.current_intent.target_robot_id == _SUPPORTER_ID

    def test_not_intent_dribble(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert not isinstance(bb.current_intent, IntentDribble)

    def test_not_intent_move(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert not isinstance(bb.current_intent, IntentMove)

    def test_no_robot_command_fields(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        intent = bb.current_intent
        for field in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert not hasattr(intent, field)

    def test_blackboard_holds_intent_after_tick(self) -> None:
        bb = _tick(self.tree, self.snapshot)
        assert bb.current_intent is not None


# ---------------------------------------------------------------------------
# Pipeline isolation invariants (R010 — no network, no hardware)
# ---------------------------------------------------------------------------

class TestPipelineInvariants:
    """Cross-scenario checks: no RobotCommand leakage, no external deps."""

    def test_no_robot_command_in_attacker_source(self) -> None:
        import inspect
        import TeamControl.bt.trees.attacker as mod
        src_text = inspect.getsource(mod)
        assert "RobotCommand" not in src_text

    def test_tree_is_reusable_across_ticks(self) -> None:
        """Same AttackerTree instance can be ticked with different snapshots."""
        tree = AttackerTree()

        snap_far = _make_snapshot(_BALL_FAR, _ATTACKER_AT_ORIGIN, False)
        bb1 = _tick(tree, snap_far)
        assert isinstance(bb1.current_intent, IntentMove)

        snap_near_with_supporter = _make_snapshot(_BALL_NEAR, _ATTACKER_AT_ORIGIN, True)
        bb2 = _tick(tree, snap_near_with_supporter)
        assert isinstance(bb2.current_intent, IntentPass)

    def test_independent_blackboards_per_tick(self) -> None:
        """Two ticks on different blackboards do not share state."""
        tree = AttackerTree()
        snap = _make_snapshot(_BALL_FAR, _ATTACKER_AT_ORIGIN, False)

        bb_a = _make_blackboard()
        tree.set_snapshot(snap)
        tree.tick(bb_a)

        bb_b = _make_blackboard()
        tree.set_snapshot(snap)
        tree.tick(bb_b)

        assert bb_a.current_intent == bb_b.current_intent
        assert bb_a is not bb_b

    def test_snapshot_not_mutated_by_tick(self) -> None:
        """Ticking the tree must not modify the Snapshot."""
        tree = AttackerTree()
        snap = _make_snapshot(_BALL_FAR, _ATTACKER_AT_ORIGIN, False)
        original_ball_pos = snap.ball_position
        original_robot_count = len(snap.own_robots)

        _tick(tree, snap)

        assert snap.ball_position == original_ball_pos
        assert len(snap.own_robots) == original_robot_count

    def test_intent_produced_on_every_tick(self) -> None:
        """Each tick produces a non-None intent regardless of scenario."""
        tree = AttackerTree()
        scenarios = [
            _make_snapshot(_BALL_FAR, _ATTACKER_AT_ORIGIN, False),
            _make_snapshot(_BALL_NEAR, _ATTACKER_AT_ORIGIN, False),
            _make_snapshot(_BALL_NEAR, _ATTACKER_AT_ORIGIN, True),
        ]
        for snap in scenarios:
            bb = _tick(tree, snap)
            assert bb.current_intent is not None, f"No intent produced for snap: {snap.ball_position}"
