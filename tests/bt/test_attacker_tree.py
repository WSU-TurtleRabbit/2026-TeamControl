"""Tests for the Attacker behaviour tree — R005.

TDD: these tests are written BEFORE the implementation exists.
All tests in this file must fail with ImportError (or a collection error
that wraps ImportError) until ``src/bt/trees/attacker.py`` is created.

Tree topology under test (from spec R005 / docs/attacking_node.png):

    AttackingSequenceNode (Sequence)
    ├── GoToBallSequence (Sequence)
    │   ├── IsBallInRange (Condition — reads from Snapshot)
    │   └── [on IsBallInRange failure] → writes IntentMove(ball_position)
    └── PassPlaySelector (Selector)
        ├── PassOrPlaySequence (Sequence)
        │   ├── IsSupporterAvailable (Condition — reads from Snapshot)
        │   └── [on success] → writes IntentPass to blackboard
        ├── HoldPossession → writes IntentDribble(goal_direction)
        └── [fallback] → writes IntentKick(goal_position)

Usage contract expected by the tree (what T010 must implement):

    from TeamControl.bt.trees.attacker import AttackerTree

    tree = AttackerTree()
    tree.set_snapshot(snapshot)   # inject world state before tick
    tree.tick(blackboard)          # run the tree; writes intent to blackboard
    intent = blackboard.current_intent
"""
from __future__ import annotations

import math
import pytest
import py_trees

# --- import under test (will ImportError until T010 is implemented) ----------
from TeamControl.bt.trees.attacker import AttackerTree  # noqa: E402

# --- contracts ---------------------------------------------------------------
from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    Intent,
    IntentDribble,
    IntentKick,
    IntentMove,
    IntentPass,
)
from TeamControl.bt.contracts.snapshot import (
    GamePhase,
    RefereeState,
    RobotState,
    Snapshot,
)


# ---------------------------------------------------------------------------
# Snapshot helpers
# ---------------------------------------------------------------------------

_GOAL_POS: tuple[float, float] = (4.5, 0.0)   # standard far goal on x-axis
_BALL_FAR: tuple[float, float] = (3.0, 0.0)    # beyond in-range threshold
_BALL_NEAR: tuple[float, float] = (0.3, 0.0)   # within in-range threshold

# Attacker robot is always robot_id=5 (index 5, ATTACKER role per Coordinator)
_ATTACKER_ID = 5
_SUPPORTER_ID = 3


def make_snapshot(
    ball_pos: tuple[float, float],
    attacker_pos: tuple[float, float],
    ball_in_range_of_attacker: bool,
    supporter_available: bool,
) -> Snapshot:
    """Build a Snapshot for the given scenario.

    Args:
        ball_pos: Where the ball is this tick.
        attacker_pos: Where the attacker robot is.
        ball_in_range_of_attacker: If True, ball is close enough to dribble/pass.
            The tree implementation defines the exact threshold; the test just
            needs a canonical near vs. far position that straddles it.
        supporter_available: If True, include a supporter robot with a known ID.
    """
    own_robots: list[RobotState] = [
        RobotState(robot_id=_ATTACKER_ID, position=attacker_pos, orientation=0.0),
    ]
    if supporter_available:
        # Supporter is in a reasonable field position
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


def _make_attacker_blackboard() -> RobotBlackboard:
    """Return a fresh RobotBlackboard for the attacker robot."""
    return RobotBlackboard(robot_id=_ATTACKER_ID, current_role=RoleType.ATTACKER)


# ---------------------------------------------------------------------------
# Scenario snapshots (reused across test classes)
# ---------------------------------------------------------------------------

SNAPSHOT_BALL_FAR = make_snapshot(
    ball_pos=_BALL_FAR,
    attacker_pos=(0.0, 0.0),
    ball_in_range_of_attacker=False,
    supporter_available=False,
)

SNAPSHOT_BALL_NEAR_SUPPORTER = make_snapshot(
    ball_pos=_BALL_NEAR,
    attacker_pos=(0.0, 0.0),
    ball_in_range_of_attacker=True,
    supporter_available=True,
)

SNAPSHOT_BALL_NEAR_NO_SUPPORTER = make_snapshot(
    ball_pos=_BALL_NEAR,
    attacker_pos=(0.0, 0.0),
    ball_in_range_of_attacker=True,
    supporter_available=False,
)


def _tick(snapshot: Snapshot, blackboard: RobotBlackboard) -> RobotBlackboard:
    """Convenience: create a tree, inject snapshot, tick, return the blackboard."""
    tree = AttackerTree()
    tree.set_snapshot(snapshot)
    tree.tick(blackboard)
    return blackboard


# ---------------------------------------------------------------------------
# TestAttackerTreeImport — basic smoke checks
# ---------------------------------------------------------------------------

class TestAttackerTreeImport:
    """AttackerTree must be importable and instantiable without side effects."""

    def test_attacker_tree_is_importable(self) -> None:
        from TeamControl.bt.trees.attacker import AttackerTree as AT  # noqa: F401

    def test_attacker_tree_instantiates(self) -> None:
        tree = AttackerTree()
        assert tree is not None

    def test_has_set_snapshot_method(self) -> None:
        tree = AttackerTree()
        assert callable(getattr(tree, "set_snapshot", None))

    def test_has_tick_method(self) -> None:
        tree = AttackerTree()
        assert callable(getattr(tree, "tick", None))

    def test_does_not_import_robot_command(self) -> None:
        import inspect
        import TeamControl.bt.trees.attacker as mod
        source = inspect.getsource(mod)
        assert "RobotCommand" not in source, (
            "AttackerTree must not reference RobotCommand"
        )


# ---------------------------------------------------------------------------
# TestAttackerTreeTopology — py_trees structure
# ---------------------------------------------------------------------------

class TestAttackerTreeTopology:
    """Root node must be a py_trees Sequence named 'AttackingSequenceNode'."""

    def test_root_is_sequence(self) -> None:
        tree = AttackerTree()
        assert isinstance(tree.root, py_trees.composites.Sequence), (
            f"Expected py_trees.composites.Sequence, got {type(tree.root)}"
        )

    def test_root_name(self) -> None:
        tree = AttackerTree()
        assert tree.root.name == "AttackingSequenceNode"

    def test_root_has_two_children(self) -> None:
        tree = AttackerTree()
        assert len(tree.root.children) == 2, (
            "Root Sequence must have exactly 2 children: "
            "GoToBallSequence and PassPlaySelector"
        )

    def test_first_child_is_sequence(self) -> None:
        """GoToBallSequence must be a Sequence."""
        tree = AttackerTree()
        first = tree.root.children[0]
        assert isinstance(first, py_trees.composites.Sequence), (
            f"First child must be Sequence (GoToBallSequence), got {type(first)}"
        )

    def test_first_child_name(self) -> None:
        tree = AttackerTree()
        first = tree.root.children[0]
        assert first.name == "GoToBallSequence"

    def test_second_child_is_selector(self) -> None:
        """PassPlaySelector must be a Selector."""
        tree = AttackerTree()
        second = tree.root.children[1]
        assert isinstance(second, py_trees.composites.Selector), (
            f"Second child must be Selector (PassPlaySelector), got {type(second)}"
        )

    def test_second_child_name(self) -> None:
        tree = AttackerTree()
        second = tree.root.children[1]
        assert second.name == "PassPlaySelector"

    def test_selector_has_children(self) -> None:
        """PassPlaySelector must have at least two children (pass branch + fallback)."""
        tree = AttackerTree()
        selector = tree.root.children[1]
        assert len(selector.children) >= 2, (
            "PassPlaySelector must have at least 2 children"
        )


# ---------------------------------------------------------------------------
# TestAttackerTreeTickInterface
# ---------------------------------------------------------------------------

class TestAttackerTreeTickInterface:
    """set_snapshot + tick compose correctly; blackboard.current_intent is set."""

    def test_tick_sets_current_intent(self) -> None:
        bb = _make_attacker_blackboard()
        assert bb.current_intent is None
        bb = _tick(SNAPSHOT_BALL_FAR, bb)
        assert bb.current_intent is not None

    def test_tick_does_not_raise_on_any_scenario(self) -> None:
        for snap in (
            SNAPSHOT_BALL_FAR,
            SNAPSHOT_BALL_NEAR_SUPPORTER,
            SNAPSHOT_BALL_NEAR_NO_SUPPORTER,
        ):
            bb = _make_attacker_blackboard()
            _tick(snap, bb)  # must not raise

    def test_multiple_ticks_do_not_raise(self) -> None:
        tree = AttackerTree()
        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        bb = _make_attacker_blackboard()
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_BALL_NEAR_SUPPORTER)
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_BALL_NEAR_NO_SUPPORTER)
        tree.tick(bb)

    def test_snapshot_can_be_replaced_between_ticks(self) -> None:
        tree = AttackerTree()
        bb = _make_attacker_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        tree.tick(bb)
        intent_a = bb.current_intent

        tree.set_snapshot(SNAPSHOT_BALL_NEAR_SUPPORTER)
        tree.tick(bb)
        intent_b = bb.current_intent

        # Different snapshots should (likely) produce different intents
        # At minimum, both are non-None Intent objects
        assert intent_a is not None
        assert intent_b is not None

    def test_tick_without_coordinator(self) -> None:
        """Tree must be usable in isolation — no Coordinator required."""
        bb = _make_attacker_blackboard()
        tree = AttackerTree()
        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        # Should not raise even without a Coordinator
        tree.tick(bb)


# ---------------------------------------------------------------------------
# TestBallOutOfRange — ball FAR → IntentMove
# ---------------------------------------------------------------------------

class TestBallOutOfRange:
    """When ball is out of range, tree must write IntentMove(ball_position)."""

    def test_ball_out_of_range_produces_intent_move(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentMove), (
            f"Expected IntentMove when ball is out of range, "
            f"got {type(bb.current_intent)}"
        )

    def test_intent_move_target_is_ball_position(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == _BALL_FAR, (
            f"IntentMove.target_pos should be ball_position {_BALL_FAR}, "
            f"got {bb.current_intent.target_pos}"
        )

    def test_intent_move_not_intent_pass(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentPass)

    def test_intent_move_not_intent_kick(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentKick)

    def test_intent_move_not_intent_dribble(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentDribble)

    def test_ball_far_no_supporter_still_produces_move(self) -> None:
        """Supporter availability must not affect ball-out-of-range case."""
        snap = make_snapshot(
            ball_pos=_BALL_FAR,
            attacker_pos=(0.0, 0.0),
            ball_in_range_of_attacker=False,
            supporter_available=True,
        )
        bb = _tick(snap, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentMove)

    def test_intent_move_target_reflects_ball_position_variations(self) -> None:
        """Different ball positions produce IntentMove to the correct position."""
        positions = [(2.0, 1.0), (3.5, -1.0), (-1.0, 2.0)]
        for bpos in positions:
            # Place attacker at origin; ball far away
            snap = Snapshot(
                ball_position=bpos,
                ball_velocity=(0.0, 0.0),
                own_robots=[
                    RobotState(robot_id=_ATTACKER_ID, position=(0.0, 0.0), orientation=0.0),
                ],
                opponent_robots=[],
                referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
            )
            # Only use positions guaranteed to be far (>= 2.0 distance)
            dist = math.hypot(bpos[0], bpos[1])
            if dist < 1.0:
                continue  # skip positions too close to attacker
            bb = _tick(snap, _make_attacker_blackboard())
            if isinstance(bb.current_intent, IntentMove):
                assert bb.current_intent.target_pos == bpos


# ---------------------------------------------------------------------------
# TestBallInRangeSupporterAvailable — ball NEAR + supporter → IntentPass
# ---------------------------------------------------------------------------

class TestBallInRangeSupporterAvailable:
    """When ball is in range and a supporter is available, tree writes IntentPass."""

    def test_ball_near_supporter_produces_intent_pass(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentPass), (
            f"Expected IntentPass when ball in range and supporter available, "
            f"got {type(bb.current_intent)}"
        )

    def test_intent_pass_has_target_robot_id(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentPass)
        assert isinstance(bb.current_intent.target_robot_id, int)

    def test_intent_pass_target_robot_is_supporter(self) -> None:
        """Pass target should be the supporter robot ID."""
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentPass)
        assert bb.current_intent.target_robot_id == _SUPPORTER_ID

    def test_intent_pass_has_target_pos(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentPass)
        pos = bb.current_intent.target_pos
        assert isinstance(pos, tuple)
        assert len(pos) == 2

    def test_intent_pass_target_pos_is_supporter_position(self) -> None:
        """Pass target position should be the supporter's current position."""
        supporter_pos = (1.5, 1.0)  # as set in make_snapshot
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, IntentPass)
        assert bb.current_intent.target_pos == supporter_pos

    def test_intent_pass_not_intent_move(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentMove)

    def test_intent_pass_not_intent_kick(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentKick)

    def test_intent_pass_not_intent_dribble(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentDribble)


# ---------------------------------------------------------------------------
# TestBallInRangeNoSupporter — ball NEAR + no supporter → IntentDribble or IntentKick
# ---------------------------------------------------------------------------

class TestBallInRangeNoSupporter:
    """When ball is in range and no supporter, tree writes IntentDribble or IntentKick."""

    def test_ball_near_no_supporter_produces_dribble_or_kick(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, (IntentDribble, IntentKick)), (
            f"Expected IntentDribble or IntentKick when no supporter, "
            f"got {type(bb.current_intent)}"
        )

    def test_no_supporter_does_not_produce_intent_pass(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentPass)

    def test_no_supporter_does_not_produce_intent_move(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert not isinstance(bb.current_intent, IntentMove)

    def test_dribble_or_kick_has_target_pos(self) -> None:
        """Both IntentDribble and IntentKick carry a target_pos."""
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, (IntentDribble, IntentKick))
        pos = bb.current_intent.target_pos
        assert isinstance(pos, tuple)
        assert len(pos) == 2

    def test_dribble_or_kick_target_is_goal_direction(self) -> None:
        """Target position should be oriented toward the opponent goal."""
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, (IntentDribble, IntentKick))
        # Goal is at positive x side; target x should be positive
        tx, _ = bb.current_intent.target_pos
        assert tx > 0.0, (
            f"Dribble/Kick target x={tx} should point toward positive-x goal"
        )


# ---------------------------------------------------------------------------
# TestNoRobotCommandWritten — R005 invariant
# ---------------------------------------------------------------------------

class TestNoRobotCommandWritten:
    """Tree must NEVER write a RobotCommand — only Intent objects."""

    _INTENT_TYPES = (IntentMove, IntentKick, IntentPass, IntentDribble)

    def _assert_no_robot_command(self, bb: RobotBlackboard) -> None:
        intent = bb.current_intent
        if intent is None:
            return
        cls_name = type(intent).__name__
        assert not cls_name.endswith("Command"), (
            f"Tree wrote a RobotCommand ({cls_name}) to blackboard — "
            "only Intent objects are allowed"
        )

    def test_no_robot_command_ball_far(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_ball_near_supporter(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_ball_near_no_supporter(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        self._assert_no_robot_command(bb)

    def test_intent_is_known_type_ball_far(self) -> None:
        bb = _tick(SNAPSHOT_BALL_FAR, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)

    def test_intent_is_known_type_ball_near_supporter(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)

    def test_intent_is_known_type_ball_near_no_supporter(self) -> None:
        bb = _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, _make_attacker_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)


# ---------------------------------------------------------------------------
# TestConditionsReadFromSnapshot — R005: conditions use Snapshot, not blackboard
# ---------------------------------------------------------------------------

class TestConditionsReadFromSnapshot:
    """Conditions must read from Snapshot; blackboard must not carry world state."""

    def test_blackboard_has_no_ball_position_field(self) -> None:
        import dataclasses
        bb = _make_attacker_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "ball_position" not in field_names

    def test_blackboard_has_no_own_robots_field(self) -> None:
        import dataclasses
        bb = _make_attacker_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "own_robots" not in field_names

    def test_blackboard_has_no_snapshot_field(self) -> None:
        import dataclasses
        bb = _make_attacker_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "snapshot" not in field_names

    def test_snapshot_change_changes_intent_ball_range(self) -> None:
        """Swapping snapshot from far-ball to near-ball changes the intent type."""
        tree = AttackerTree()
        bb = _make_attacker_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        tree.tick(bb)
        far_intent = type(bb.current_intent)

        tree.set_snapshot(SNAPSHOT_BALL_NEAR_NO_SUPPORTER)
        tree.tick(bb)
        near_intent = type(bb.current_intent)

        assert far_intent != near_intent, (
            "Intent type must differ between ball-far and ball-near scenarios"
        )

    def test_snapshot_change_changes_intent_supporter(self) -> None:
        """Swapping snapshot supporter-available vs not changes intent type."""
        tree = AttackerTree()
        bb = _make_attacker_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_NEAR_SUPPORTER)
        tree.tick(bb)
        with_supporter = type(bb.current_intent)

        tree.set_snapshot(SNAPSHOT_BALL_NEAR_NO_SUPPORTER)
        tree.tick(bb)
        without_supporter = type(bb.current_intent)

        assert with_supporter != without_supporter, (
            "Intent type must differ when supporter is vs. is not available"
        )


# ---------------------------------------------------------------------------
# TestBlackboardUpdatedAfterTick — R005: current_intent updated after each tick
# ---------------------------------------------------------------------------

class TestBlackboardUpdatedAfterTick:
    """blackboard.current_intent must be set (not None) after every tick."""

    def test_intent_set_after_tick_ball_far(self) -> None:
        bb = _make_attacker_blackboard()
        assert bb.current_intent is None
        _tick(SNAPSHOT_BALL_FAR, bb)
        assert bb.current_intent is not None

    def test_intent_set_after_tick_ball_near_supporter(self) -> None:
        bb = _make_attacker_blackboard()
        _tick(SNAPSHOT_BALL_NEAR_SUPPORTER, bb)
        assert bb.current_intent is not None

    def test_intent_set_after_tick_ball_near_no_supporter(self) -> None:
        bb = _make_attacker_blackboard()
        _tick(SNAPSHOT_BALL_NEAR_NO_SUPPORTER, bb)
        assert bb.current_intent is not None

    def test_intent_replaced_on_second_tick(self) -> None:
        """Second tick must overwrite current_intent, not leave stale value."""
        tree = AttackerTree()
        bb = _make_attacker_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        tree.tick(bb)
        first_intent = bb.current_intent

        tree.set_snapshot(SNAPSHOT_BALL_NEAR_SUPPORTER)
        tree.tick(bb)
        second_intent = bb.current_intent

        assert second_intent is not None
        # Scenarios are different so intents must differ in type at minimum
        assert type(first_intent) != type(second_intent)

    def test_three_ticks_all_produce_intents(self) -> None:
        tree = AttackerTree()
        bb = _make_attacker_blackboard()

        for snap in (
            SNAPSHOT_BALL_FAR,
            SNAPSHOT_BALL_NEAR_SUPPORTER,
            SNAPSHOT_BALL_NEAR_NO_SUPPORTER,
        ):
            tree.set_snapshot(snap)
            tree.tick(bb)
            assert bb.current_intent is not None, (
                f"current_intent must be set after tick with snapshot {snap}"
            )


# ---------------------------------------------------------------------------
# TestIsolation — tree works without Coordinator (R005)
# ---------------------------------------------------------------------------

class TestIsolation:
    """Tree can be ticked in isolation with a mock snapshot; no Coordinator needed."""

    def test_fresh_tree_needs_no_coordinator(self) -> None:
        tree = AttackerTree()
        bb = _make_attacker_blackboard()
        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        tree.tick(bb)  # must not raise

    def test_two_tree_instances_are_independent(self) -> None:
        tree_a = AttackerTree()
        tree_b = AttackerTree()
        bb_a = _make_attacker_blackboard()
        bb_b = _make_attacker_blackboard()

        tree_a.set_snapshot(SNAPSHOT_BALL_FAR)
        tree_b.set_snapshot(SNAPSHOT_BALL_NEAR_SUPPORTER)

        tree_a.tick(bb_a)
        tree_b.tick(bb_b)

        # a → far ball → MOVE; b → near ball + supporter → PASS
        assert isinstance(bb_a.current_intent, IntentMove)
        assert isinstance(bb_b.current_intent, IntentPass)

    def test_tree_can_be_used_without_py_trees_runner(self) -> None:
        """AttackerTree.tick() must work without py_trees BehaviourTree/Runner."""
        tree = AttackerTree()
        bb = _make_attacker_blackboard()
        tree.set_snapshot(SNAPSHOT_BALL_FAR)
        # Directly calling tick() — not via py_trees.trees.BehaviourTree
        tree.tick(bb)
        assert bb.current_intent is not None
