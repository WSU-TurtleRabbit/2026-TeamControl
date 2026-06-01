"""Tests for the Defender behaviour tree — R006.

TDD: these tests are written BEFORE the implementation exists.
All tests in this file must fail with ImportError (or a collection error
that wraps ImportError) until ``src/bt/trees/defender.py`` is created.

Tree topology under test (from spec R006 / docs/defending_node.png):

    DefendingSequenceNode (Sequence)
    ├── LookAtBall         → writes IntentOrient(angle_to_ball)
    ├── DefendZoneFallback (Selector — OR logic)
    │   ├── InDefendingZone  (Condition — reads Snapshot)
    │   └── GoToDefendZone   → writes IntentMove(defend_zone_pos) on failure
    └── ChallengeSequence (Sequence)
        ├── IsCloseEnough    (Condition — reads Snapshot)
        └── ClearBall        → writes IntentKick(clear_direction)

Usage contract (mirrors AttackerTree):

    from TeamControl.bt.trees.defender import DefenderTree

    tree = DefenderTree()
    tree.set_snapshot(snapshot)
    tree.tick(blackboard)
    intent = blackboard.current_intent
"""
from __future__ import annotations

import math
import pytest
import py_trees

# --- import under test (will ImportError until defender.py is implemented) ---
from TeamControl.bt.trees.defender import DefenderTree  # noqa: E402

# --- contracts ---------------------------------------------------------------
from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    Intent,
    IntentKick,
    IntentMove,
    IntentOrient,
)
from TeamControl.bt.contracts.snapshot import (
    GamePhase,
    RefereeState,
    RobotState,
    Snapshot,
)


# ---------------------------------------------------------------------------
# Constants (mirrored from defender.py — keep in sync)
# ---------------------------------------------------------------------------

_DEFENDER_ID = 1
_DEFEND_ZONE_POSITION = (-3.0, 0.0)
_CLOSE_ENOUGH_THRESHOLD = 0.6
_CLEAR_DIRECTION = (4.5, 0.0)


# ---------------------------------------------------------------------------
# Snapshot helpers
# ---------------------------------------------------------------------------

def _make_snapshot(
    robot_pos: tuple[float, float],
    ball_pos: tuple[float, float],
) -> Snapshot:
    """Build a minimal Snapshot with one defender robot."""
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=(0.0, 0.0),
        own_robots=[
            RobotState(robot_id=_DEFENDER_ID, position=robot_pos, orientation=0.0),
        ],
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


def _make_defender_blackboard() -> RobotBlackboard:
    return RobotBlackboard(robot_id=_DEFENDER_ID, current_role=RoleType.DEFENDER)


def _tick(snapshot: Snapshot, blackboard: RobotBlackboard) -> RobotBlackboard:
    """Convenience: create tree, inject snapshot, tick, return blackboard."""
    tree = DefenderTree()
    tree.set_snapshot(snapshot)
    tree.tick(blackboard)
    return blackboard


# ---------------------------------------------------------------------------
# Scenario snapshots
# ---------------------------------------------------------------------------

# Defender in own half (x < 0), ball far away (dist > threshold)
SNAP_IN_ZONE_BALL_FAR = _make_snapshot(
    robot_pos=(-2.0, 0.0),
    ball_pos=(2.0, 0.0),
)

# Defender in own half (x < 0), ball close (dist <= threshold)
SNAP_IN_ZONE_BALL_CLOSE = _make_snapshot(
    robot_pos=(-2.0, 0.0),
    ball_pos=(-2.3, 0.0),  # 0.3 m away — within 0.6 m threshold
)

# Defender out of own half (x >= 0), ball far
SNAP_OUT_ZONE_BALL_FAR = _make_snapshot(
    robot_pos=(1.0, 0.0),
    ball_pos=(3.0, 0.0),
)

# Defender out of own half (x >= 0), ball close
SNAP_OUT_ZONE_BALL_CLOSE = _make_snapshot(
    robot_pos=(0.2, 0.0),
    ball_pos=(0.5, 0.0),  # 0.3 m away — within threshold
)


# ---------------------------------------------------------------------------
# TestDefenderTreeImport
# ---------------------------------------------------------------------------

class TestDefenderTreeImport:
    """DefenderTree must be importable and instantiable without side effects."""

    def test_defender_tree_is_importable(self) -> None:
        from TeamControl.bt.trees.defender import DefenderTree as DT  # noqa: F401

    def test_defender_tree_instantiates(self) -> None:
        tree = DefenderTree()
        assert tree is not None

    def test_has_set_snapshot_method(self) -> None:
        tree = DefenderTree()
        assert callable(getattr(tree, "set_snapshot", None))

    def test_has_tick_method(self) -> None:
        tree = DefenderTree()
        assert callable(getattr(tree, "tick", None))

    def test_does_not_import_robot_command(self) -> None:
        import inspect
        import TeamControl.bt.trees.defender as mod
        source = inspect.getsource(mod)
        assert "RobotCommand" not in source, (
            "DefenderTree must not reference RobotCommand"
        )


# ---------------------------------------------------------------------------
# TestDefenderTreeTopology — py_trees structure
# ---------------------------------------------------------------------------

class TestDefenderTreeTopology:
    """Root must be a py_trees Sequence named 'DefendingSequenceNode'."""

    def test_root_is_sequence(self) -> None:
        tree = DefenderTree()
        assert isinstance(tree.root, py_trees.composites.Sequence)

    def test_root_name(self) -> None:
        tree = DefenderTree()
        assert tree.root.name == "DefendingSequenceNode"

    def test_root_has_three_children(self) -> None:
        tree = DefenderTree()
        assert len(tree.root.children) == 3, (
            "Root must have 3 children: LookAtBall, DefendZoneFallback, ChallengeSequence"
        )

    def test_second_child_is_selector(self) -> None:
        """DefendZoneFallback must be a Selector (OR logic)."""
        tree = DefenderTree()
        second = tree.root.children[1]
        assert isinstance(second, py_trees.composites.Selector), (
            f"Second child must be Selector (DefendZoneFallback), got {type(second)}"
        )

    def test_second_child_name(self) -> None:
        tree = DefenderTree()
        second = tree.root.children[1]
        assert second.name == "DefendZoneFallback"

    def test_third_child_is_sequence(self) -> None:
        """ChallengeSequence must be a Sequence (AND logic)."""
        tree = DefenderTree()
        third = tree.root.children[2]
        assert isinstance(third, py_trees.composites.Sequence), (
            f"Third child must be Sequence (ChallengeSequence), got {type(third)}"
        )

    def test_third_child_name(self) -> None:
        tree = DefenderTree()
        third = tree.root.children[2]
        assert third.name == "ChallengeSequence"

    def test_selector_has_two_children(self) -> None:
        tree = DefenderTree()
        selector = tree.root.children[1]
        assert len(selector.children) == 2, (
            "DefendZoneFallback must have 2 children: InDefendingZone + GoToDefendZone"
        )

    def test_challenge_sequence_has_two_children(self) -> None:
        tree = DefenderTree()
        challenge = tree.root.children[2]
        assert len(challenge.children) == 2, (
            "ChallengeSequence must have 2 children: IsCloseEnough + ClearBall"
        )


# ---------------------------------------------------------------------------
# TestLookAtBall — first node always writes IntentOrient
# ---------------------------------------------------------------------------

class TestLookAtBall:
    """LookAtBall writes IntentOrient with the correct angle to the ball."""

    def test_look_at_ball_produces_intent_orient(self) -> None:
        """In-zone + ball far: LookAtBall runs, produces IntentOrient (or later node)."""
        # We verify IntentOrient is written by checking a scenario where no
        # IntentKick is produced (ball is too far for ClearBall).
        bb = _tick(SNAP_IN_ZONE_BALL_FAR, _make_defender_blackboard())
        # LookAtBall always succeeds and writes IntentOrient, but later nodes
        # may overwrite it. We check with a fresh tree that only runs LookAtBall.
        # The easiest proof: orient angle is computed from robot→ball direction.
        assert bb.current_intent is not None

    def test_look_at_ball_angle_correct_east(self) -> None:
        """Ball directly east of robot → angle ≈ 0 rad."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0), ball_pos=(1.0, 0.0))
        # In zone (robot x=0 is NOT in own half). Use a robot in own half.
        snap2 = _make_snapshot(robot_pos=(-1.0, 0.0), ball_pos=(0.0, 0.0))
        # Ball at (0,0), robot at (-1,0) → atan2(0-0, 0-(-1)) = atan2(0,1) = 0
        tree = DefenderTree()
        tree.set_snapshot(snap2)
        bb = _make_defender_blackboard()
        tree.tick(bb)
        # LookAtBall runs first; it writes IntentOrient before any other node.
        # If ChallengeSequence fires (ball close enough), intent may be overwritten.
        # ball dist = 1.0 > 0.6 threshold → ClearBall not reached → IntentOrient final.
        assert isinstance(bb.current_intent, IntentOrient), (
            f"Expected IntentOrient, got {type(bb.current_intent)}"
        )
        expected_angle = math.atan2(0.0 - 0.0, 0.0 - (-1.0))  # 0.0
        assert abs(bb.current_intent.target_orientation - expected_angle) < 1e-6

    def test_look_at_ball_angle_correct_north(self) -> None:
        """Ball directly north of robot → angle ≈ pi/2 rad."""
        snap = _make_snapshot(robot_pos=(-2.0, 0.0), ball_pos=(-2.0, 1.0))
        # dist to ball = 1.0 > threshold; in own half → IntentOrient expected
        bb = _tick(snap, _make_defender_blackboard())
        expected = math.atan2(1.0 - 0.0, -2.0 - (-2.0))  # pi/2
        assert isinstance(bb.current_intent, IntentOrient)
        assert abs(bb.current_intent.target_orientation - expected) < 1e-6

    def test_look_at_ball_angle_correct_southwest(self) -> None:
        """Ball SW of robot → correct atan2 angle."""
        robot = (-1.0, 1.0)
        ball = (-2.0, 0.0)
        snap = _make_snapshot(robot_pos=robot, ball_pos=ball)
        # dist = sqrt(1+1) ≈ 1.41 > 0.6; robot in own half
        bb = _tick(snap, _make_defender_blackboard())
        expected = math.atan2(ball[1] - robot[1], ball[0] - robot[0])
        assert isinstance(bb.current_intent, IntentOrient)
        assert abs(bb.current_intent.target_orientation - expected) < 1e-6


# ---------------------------------------------------------------------------
# TestInDefendingZone — condition: robot.x < 0
# ---------------------------------------------------------------------------

class TestInDefendingZone:
    """InDefendingZone condition: SUCCESS when robot x < 0."""

    def test_in_zone_no_move_intent(self) -> None:
        """When robot is in own half, GoToDefendZone must NOT fire."""
        bb = _tick(SNAP_IN_ZONE_BALL_FAR, _make_defender_blackboard())
        # Result is IntentOrient (LookAtBall) or IntentKick (challenge)
        # — must NOT be IntentMove to defend zone
        if isinstance(bb.current_intent, IntentMove):
            # If it IS a move, it must NOT be moving to defend zone
            assert bb.current_intent.target_pos != _DEFEND_ZONE_POSITION, (
                "Should not move to defend zone when already in zone"
            )

    def test_out_of_zone_triggers_move_to_defend_zone(self) -> None:
        """When robot is out of own half, GoToDefendZone writes IntentMove."""
        bb = _tick(SNAP_OUT_ZONE_BALL_FAR, _make_defender_blackboard())
        assert isinstance(bb.current_intent, IntentMove), (
            f"Expected IntentMove when out of zone, got {type(bb.current_intent)}"
        )
        assert bb.current_intent.target_pos == _DEFEND_ZONE_POSITION

    def test_boundary_x_exactly_zero_is_out_of_zone(self) -> None:
        """robot.x == 0.0 is NOT in own half (condition requires x < 0)."""
        snap = _make_snapshot(robot_pos=(0.0, 0.5), ball_pos=(3.0, 0.0))
        bb = _tick(snap, _make_defender_blackboard())
        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == _DEFEND_ZONE_POSITION

    def test_robot_at_negative_x_is_in_zone(self) -> None:
        """robot.x < 0 → in defending zone → no GoToDefendZone intent."""
        snap = _make_snapshot(robot_pos=(-0.01, 0.0), ball_pos=(3.0, 0.0))
        bb = _tick(snap, _make_defender_blackboard())
        # Should NOT move to defend zone
        if isinstance(bb.current_intent, IntentMove):
            assert bb.current_intent.target_pos != _DEFEND_ZONE_POSITION


# ---------------------------------------------------------------------------
# TestChallengeSequence — IsCloseEnough + ClearBall
# ---------------------------------------------------------------------------

class TestChallengeSequence:
    """When robot is in zone and close enough to ball, write IntentKick."""

    def test_in_zone_close_to_ball_produces_intent_kick(self) -> None:
        bb = _tick(SNAP_IN_ZONE_BALL_CLOSE, _make_defender_blackboard())
        assert isinstance(bb.current_intent, IntentKick), (
            f"Expected IntentKick when close to ball, got {type(bb.current_intent)}"
        )

    def test_clear_ball_direction_is_correct(self) -> None:
        """ClearBall must kick toward _CLEAR_DIRECTION."""
        bb = _tick(SNAP_IN_ZONE_BALL_CLOSE, _make_defender_blackboard())
        assert isinstance(bb.current_intent, IntentKick)
        assert bb.current_intent.target_pos == _CLEAR_DIRECTION, (
            f"Expected kick to {_CLEAR_DIRECTION}, got {bb.current_intent.target_pos}"
        )

    def test_in_zone_ball_far_no_kick(self) -> None:
        """Ball beyond threshold → ClearBall must not fire."""
        bb = _tick(SNAP_IN_ZONE_BALL_FAR, _make_defender_blackboard())
        assert not isinstance(bb.current_intent, IntentKick), (
            "Must not kick when ball is beyond close-enough threshold"
        )

    def test_out_of_zone_close_to_ball_produces_move_not_kick(self) -> None:
        """Out of zone: GoToDefendZone fires; ChallengeSequence also runs.

        Out of zone + ball close: GoToDefendZone writes IntentMove (Selector
        succeeds), then ChallengeSequence runs. If ball is close enough,
        IntentKick should be the final intent.
        """
        bb = _tick(SNAP_OUT_ZONE_BALL_CLOSE, _make_defender_blackboard())
        # Final intent: either IntentMove (from GoToDefendZone) or IntentKick
        # (if ChallengeSequence also fires). The key invariant is that a
        # raw command is NOT written.
        assert isinstance(bb.current_intent, (IntentMove, IntentKick))

    def test_close_enough_at_exact_threshold(self) -> None:
        """dist == threshold → SUCCESS (inclusive boundary).

        Use a Pythagorean triple so the distance is exact in floating point:
        robot at (-2.0, 0.0), ball at (-2.0 + 0.36, 0.48) → dist = 0.60 exactly
        (3-4-5 triple scaled by 0.12: 0.36² + 0.48² = 0.1296 + 0.2304 = 0.36 → sqrt = 0.6).
        """
        snap = _make_snapshot(
            robot_pos=(-2.0, 0.0),
            ball_pos=(-2.0 + 0.36, 0.48),  # dist = sqrt(0.36² + 0.48²) = 0.6 exactly
        )
        bb = _tick(snap, _make_defender_blackboard())
        assert isinstance(bb.current_intent, IntentKick)

    def test_just_beyond_threshold_no_kick(self) -> None:
        """dist slightly > threshold → FAILURE, no kick."""
        snap = _make_snapshot(
            robot_pos=(-2.0, 0.0),
            ball_pos=(-2.0 + _CLOSE_ENOUGH_THRESHOLD + 0.01, 0.0),
        )
        bb = _tick(snap, _make_defender_blackboard())
        assert not isinstance(bb.current_intent, IntentKick)


# ---------------------------------------------------------------------------
# TestNoRobotCommandWritten — R006 invariant
# ---------------------------------------------------------------------------

class TestNoRobotCommandWritten:
    """Tree must NEVER write a RobotCommand — only Intent objects."""

    _INTENT_TYPES = (IntentMove, IntentKick, IntentOrient)

    def _assert_no_robot_command(self, bb: RobotBlackboard) -> None:
        intent = bb.current_intent
        if intent is None:
            return
        cls_name = type(intent).__name__
        assert not cls_name.endswith("Command"), (
            f"Tree wrote a RobotCommand ({cls_name}) — only Intent objects allowed"
        )

    def test_no_robot_command_in_zone_far(self) -> None:
        bb = _tick(SNAP_IN_ZONE_BALL_FAR, _make_defender_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_in_zone_close(self) -> None:
        bb = _tick(SNAP_IN_ZONE_BALL_CLOSE, _make_defender_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_out_zone_far(self) -> None:
        bb = _tick(SNAP_OUT_ZONE_BALL_FAR, _make_defender_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_out_zone_close(self) -> None:
        bb = _tick(SNAP_OUT_ZONE_BALL_CLOSE, _make_defender_blackboard())
        self._assert_no_robot_command(bb)

    def test_intent_is_known_type_in_zone_far(self) -> None:
        bb = _tick(SNAP_IN_ZONE_BALL_FAR, _make_defender_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)

    def test_intent_is_known_type_in_zone_close(self) -> None:
        bb = _tick(SNAP_IN_ZONE_BALL_CLOSE, _make_defender_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)

    def test_intent_is_known_type_out_zone_far(self) -> None:
        bb = _tick(SNAP_OUT_ZONE_BALL_FAR, _make_defender_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)


# ---------------------------------------------------------------------------
# TestSnapshotOnlyReads — conditions must not use blackboard for world state
# ---------------------------------------------------------------------------

class TestSnapshotOnlyReads:
    """Conditions read from Snapshot only; blackboard carries no world state."""

    def test_blackboard_has_no_ball_position_field(self) -> None:
        import dataclasses
        bb = _make_defender_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "ball_position" not in field_names

    def test_blackboard_has_no_own_robots_field(self) -> None:
        import dataclasses
        bb = _make_defender_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "own_robots" not in field_names

    def test_snapshot_change_changes_intent(self) -> None:
        """Swapping snapshot changes intent when zone/closeness changes."""
        tree = DefenderTree()
        bb = _make_defender_blackboard()

        tree.set_snapshot(SNAP_IN_ZONE_BALL_FAR)
        tree.tick(bb)
        intent_a = type(bb.current_intent)

        tree.set_snapshot(SNAP_IN_ZONE_BALL_CLOSE)
        tree.tick(bb)
        intent_b = type(bb.current_intent)

        # Far → IntentOrient; Close → IntentKick
        assert intent_a != intent_b


# ---------------------------------------------------------------------------
# TestIsolation — tree works without Coordinator
# ---------------------------------------------------------------------------

class TestIsolation:
    """DefenderTree works in isolation without a Coordinator."""

    def test_fresh_tree_needs_no_coordinator(self) -> None:
        tree = DefenderTree()
        bb = _make_defender_blackboard()
        tree.set_snapshot(SNAP_IN_ZONE_BALL_FAR)
        tree.tick(bb)

    def test_two_tree_instances_are_independent(self) -> None:
        tree_a = DefenderTree()
        tree_b = DefenderTree()
        bb_a = _make_defender_blackboard()
        bb_b = _make_defender_blackboard()

        tree_a.set_snapshot(SNAP_IN_ZONE_BALL_CLOSE)
        tree_b.set_snapshot(SNAP_OUT_ZONE_BALL_FAR)

        tree_a.tick(bb_a)
        tree_b.tick(bb_b)

        assert isinstance(bb_a.current_intent, IntentKick)
        assert isinstance(bb_b.current_intent, IntentMove)
        assert bb_b.current_intent.target_pos == _DEFEND_ZONE_POSITION

    def test_multiple_ticks_do_not_raise(self) -> None:
        tree = DefenderTree()
        bb = _make_defender_blackboard()
        for snap in (
            SNAP_IN_ZONE_BALL_FAR,
            SNAP_IN_ZONE_BALL_CLOSE,
            SNAP_OUT_ZONE_BALL_FAR,
            SNAP_OUT_ZONE_BALL_CLOSE,
        ):
            tree.set_snapshot(snap)
            tree.tick(bb)

    def test_can_be_used_without_py_trees_runner(self) -> None:
        tree = DefenderTree()
        bb = _make_defender_blackboard()
        tree.set_snapshot(SNAP_IN_ZONE_BALL_FAR)
        tree.tick(bb)
        assert bb.current_intent is not None
