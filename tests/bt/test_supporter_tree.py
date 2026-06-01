"""Tests for the Supporter behaviour tree — R007.

TDD: these tests are written BEFORE the implementation exists.
All tests in this file must fail with ImportError (or a collection error
that wraps ImportError) until ``src/bt/trees/supporter.py`` is created.

Tree topology under test (from spec R007 / docs/supporting_node.png):

    SupportingSelectorNode (Selector)
    ├── MoveToSpace        → writes IntentMove(open_space_pos)
    ├── ReceiveBallSequence (Sequence)
    │   ├── IsBallComing   (Condition — STUBBED, always returns FAILURE)
    │   └── ReceiveBall    → writes IntentReceive()
    └── BlockOpponent      → writes IntentMove(blocking_pos)

Usage contract expected by the tree:

    from TeamControl.bt.trees.supporter import SupporterTree

    tree = SupporterTree()
    tree.set_snapshot(snapshot)   # inject world state before tick
    tree.tick(blackboard)          # run the tree; writes intent to blackboard
    intent = blackboard.current_intent
"""
from __future__ import annotations

import pytest
import py_trees

# --- import under test (will ImportError until supporter.py is implemented) ---
from TeamControl.bt.trees.supporter import SupporterTree  # noqa: E402

# --- contracts ---------------------------------------------------------------
from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    Intent,
    IntentMove,
    IntentReceive,
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

_SUPPORTER_ID = 3


def make_snapshot(
    ball_pos: tuple[float, float] = (0.0, 0.0),
    own_robots: list[RobotState] | None = None,
) -> Snapshot:
    """Build a minimal Snapshot for supporter tests."""
    if own_robots is None:
        own_robots = [
            RobotState(robot_id=_SUPPORTER_ID, position=(0.0, 0.0), orientation=0.0),
        ]
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=(0.0, 0.0),
        own_robots=own_robots,
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


def _make_supporter_blackboard() -> RobotBlackboard:
    """Return a fresh RobotBlackboard for the supporter robot."""
    return RobotBlackboard(robot_id=_SUPPORTER_ID, current_role=RoleType.SUPPORTER)


SNAPSHOT_DEFAULT = make_snapshot()


def _tick(snapshot: Snapshot, blackboard: RobotBlackboard) -> RobotBlackboard:
    """Convenience: create a tree, inject snapshot, tick, return the blackboard."""
    tree = SupporterTree()
    tree.set_snapshot(snapshot)
    tree.tick(blackboard)
    return blackboard


# ---------------------------------------------------------------------------
# TestSupporterTreeImport — basic smoke checks
# ---------------------------------------------------------------------------

class TestSupporterTreeImport:
    """SupporterTree must be importable and instantiable without side effects."""

    def test_supporter_tree_is_importable(self) -> None:
        from TeamControl.bt.trees.supporter import SupporterTree as ST  # noqa: F401

    def test_supporter_tree_instantiates(self) -> None:
        tree = SupporterTree()
        assert tree is not None

    def test_has_set_snapshot_method(self) -> None:
        tree = SupporterTree()
        assert callable(getattr(tree, "set_snapshot", None))

    def test_has_tick_method(self) -> None:
        tree = SupporterTree()
        assert callable(getattr(tree, "tick", None))

    def test_does_not_import_robot_command(self) -> None:
        import inspect
        import TeamControl.bt.trees.supporter as mod
        source = inspect.getsource(mod)
        assert "RobotCommand" not in source, (
            "SupporterTree must not reference RobotCommand"
        )


# ---------------------------------------------------------------------------
# TestSupporterTreeTopology — py_trees structure
# ---------------------------------------------------------------------------

class TestSupporterTreeTopology:
    """Root node must be a py_trees Selector named 'SupportingSelectorNode'."""

    def test_root_is_selector(self) -> None:
        tree = SupporterTree()
        assert isinstance(tree.root, py_trees.composites.Selector), (
            f"Expected py_trees.composites.Selector, got {type(tree.root)}"
        )

    def test_root_name(self) -> None:
        tree = SupporterTree()
        assert tree.root.name == "SupportingSelectorNode"

    def test_root_has_three_children(self) -> None:
        tree = SupporterTree()
        assert len(tree.root.children) == 3, (
            "Root Selector must have exactly 3 children: "
            "MoveToSpace, ReceiveBallSequence, BlockOpponent"
        )

    def test_first_child_is_behaviour(self) -> None:
        """MoveToSpace must be a Behaviour leaf node."""
        tree = SupporterTree()
        first = tree.root.children[0]
        assert isinstance(first, py_trees.behaviour.Behaviour), (
            f"First child must be a Behaviour (MoveToSpace), got {type(first)}"
        )

    def test_first_child_name(self) -> None:
        tree = SupporterTree()
        first = tree.root.children[0]
        assert first.name == "MoveToSpace"

    def test_second_child_is_sequence(self) -> None:
        """ReceiveBallSequence must be a Sequence."""
        tree = SupporterTree()
        second = tree.root.children[1]
        assert isinstance(second, py_trees.composites.Sequence), (
            f"Second child must be Sequence (ReceiveBallSequence), got {type(second)}"
        )

    def test_second_child_name(self) -> None:
        tree = SupporterTree()
        second = tree.root.children[1]
        assert second.name == "ReceiveBallSequence"

    def test_receive_ball_sequence_has_two_children(self) -> None:
        tree = SupporterTree()
        seq = tree.root.children[1]
        assert len(seq.children) == 2, (
            "ReceiveBallSequence must have exactly 2 children: "
            "IsBallComing and ReceiveBall"
        )

    def test_is_ball_coming_is_first_child_of_sequence(self) -> None:
        tree = SupporterTree()
        seq = tree.root.children[1]
        first = seq.children[0]
        assert first.name == "IsBallComing"

    def test_receive_ball_is_second_child_of_sequence(self) -> None:
        tree = SupporterTree()
        seq = tree.root.children[1]
        second = seq.children[1]
        assert second.name == "ReceiveBall"

    def test_third_child_is_behaviour(self) -> None:
        """BlockOpponent must be a Behaviour leaf node."""
        tree = SupporterTree()
        third = tree.root.children[2]
        assert isinstance(third, py_trees.behaviour.Behaviour), (
            f"Third child must be a Behaviour (BlockOpponent), got {type(third)}"
        )

    def test_third_child_name(self) -> None:
        tree = SupporterTree()
        third = tree.root.children[2]
        assert third.name == "BlockOpponent"


# ---------------------------------------------------------------------------
# TestSupporterTreeTickInterface
# ---------------------------------------------------------------------------

class TestSupporterTreeTickInterface:
    """set_snapshot + tick compose correctly; blackboard.current_intent is set."""

    def test_tick_sets_current_intent(self) -> None:
        bb = _make_supporter_blackboard()
        assert bb.current_intent is None
        bb = _tick(SNAPSHOT_DEFAULT, bb)
        assert bb.current_intent is not None

    def test_tick_does_not_raise(self) -> None:
        bb = _make_supporter_blackboard()
        _tick(SNAPSHOT_DEFAULT, bb)  # must not raise

    def test_multiple_ticks_do_not_raise(self) -> None:
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        bb = _make_supporter_blackboard()
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)

    def test_snapshot_can_be_replaced_between_ticks(self) -> None:
        tree = SupporterTree()
        bb = _make_supporter_blackboard()

        snap_a = make_snapshot(ball_pos=(1.0, 0.0))
        snap_b = make_snapshot(ball_pos=(2.0, 0.0))

        tree.set_snapshot(snap_a)
        tree.tick(bb)
        intent_a = bb.current_intent

        tree.set_snapshot(snap_b)
        tree.tick(bb)
        intent_b = bb.current_intent

        assert intent_a is not None
        assert intent_b is not None

    def test_tick_without_coordinator(self) -> None:
        """Tree must be usable in isolation — no Coordinator required."""
        bb = _make_supporter_blackboard()
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)


# ---------------------------------------------------------------------------
# TestMoveToSpaceAction — v1: MoveToSpace always succeeds → IntentMove
# ---------------------------------------------------------------------------

class TestMoveToSpaceAction:
    """MoveToSpace must always succeed in v1, writing IntentMove(open_space_pos)."""

    def test_produces_intent_move(self) -> None:
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert isinstance(bb.current_intent, IntentMove), (
            f"Expected IntentMove from MoveToSpace, got {type(bb.current_intent)}"
        )

    def test_intent_move_has_target_pos(self) -> None:
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert isinstance(bb.current_intent, IntentMove)
        pos = bb.current_intent.target_pos
        assert isinstance(pos, tuple)
        assert len(pos) == 2

    def test_intent_move_target_is_open_space_position(self) -> None:
        """v1 open-space position is hardcoded to (1.0, 2.0)."""
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == (1.0, 2.0), (
            f"Expected open-space pos (1.0, 2.0), got {bb.current_intent.target_pos}"
        )

    def test_move_to_space_not_intent_receive(self) -> None:
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert not isinstance(bb.current_intent, IntentReceive)

    def test_move_to_space_result_is_consistent(self) -> None:
        """Multiple ticks on same snapshot produce same intent type."""
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        for _ in range(3):
            bb = _make_supporter_blackboard()
            tree.tick(bb)
            assert isinstance(bb.current_intent, IntentMove)


# ---------------------------------------------------------------------------
# TestIsBallComingStub — R007: IsBallComing always returns FAILURE in v1
# ---------------------------------------------------------------------------

class TestIsBallComingStub:
    """IsBallComing stub must always return FAILURE, blocking ReceiveBallSequence."""

    def test_is_ball_coming_always_fails(self) -> None:
        """ReceiveBallSequence never fires in v1 — IsBallComing blocks it."""
        import TeamControl.bt.trees.supporter as mod
        # Instantiate the tree to get access to node instances
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        bb = _make_supporter_blackboard()
        tree._blackboard_ref[0] = bb

        # Find IsBallComing child inside ReceiveBallSequence
        receive_seq = tree.root.children[1]
        is_ball_coming = receive_seq.children[0]

        # Tick it directly — must always return FAILURE
        result = is_ball_coming.update()
        assert result == py_trees.common.Status.FAILURE, (
            f"IsBallComing stub must return FAILURE, got {result}"
        )

    def test_is_ball_coming_has_todo_comment(self) -> None:
        """IsBallComing must have a TODO comment referencing DoBallTrajectory."""
        import inspect
        import TeamControl.bt.trees.supporter as mod
        source = inspect.getsource(mod)
        assert "TODO" in source and "DoBallTrajectory" in source, (
            "IsBallComing stub must contain '# TODO: implement DoBallTrajectory'"
        )

    def test_receive_ball_sequence_never_fires_in_v1(self) -> None:
        """Because IsBallComing always fails, ReceiveBall is never reached.

        So current_intent should never be IntentReceive in v1.
        """
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert not isinstance(bb.current_intent, IntentReceive), (
            "IntentReceive must not appear in v1 — IsBallComing stub blocks the sequence"
        )

    def test_receive_ball_sequence_never_fires_multiple_ticks(self) -> None:
        tree = SupporterTree()
        for _ in range(5):
            tree.set_snapshot(SNAPSHOT_DEFAULT)
            bb = _make_supporter_blackboard()
            tree.tick(bb)
            assert not isinstance(bb.current_intent, IntentReceive)


# ---------------------------------------------------------------------------
# TestBlockOpponentFallback — topology check only (never reached in v1)
# ---------------------------------------------------------------------------

class TestBlockOpponentNode:
    """BlockOpponent node exists in the tree but is never reached in v1.

    We verify its structure and that it would write IntentMove(blocking_pos)
    if invoked directly.
    """

    def test_block_opponent_exists_in_tree(self) -> None:
        tree = SupporterTree()
        third = tree.root.children[2]
        assert third.name == "BlockOpponent"

    def test_block_opponent_writes_intent_move_when_called_directly(self) -> None:
        """Direct invocation of BlockOpponent.update() writes IntentMove."""
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        bb = _make_supporter_blackboard()
        tree._blackboard_ref[0] = bb

        block_opponent = tree.root.children[2]
        result = block_opponent.update()

        assert result == py_trees.common.Status.SUCCESS
        assert isinstance(bb.current_intent, IntentMove), (
            f"BlockOpponent must write IntentMove, got {type(bb.current_intent)}"
        )

    def test_block_opponent_target_is_blocking_position(self) -> None:
        """v1 blocking position is hardcoded to (-1.0, 0.0)."""
        tree = SupporterTree()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        bb = _make_supporter_blackboard()
        tree._blackboard_ref[0] = bb

        block_opponent = tree.root.children[2]
        block_opponent.update()

        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == (-1.0, 0.0), (
            f"Expected blocking pos (-1.0, 0.0), got {bb.current_intent.target_pos}"
        )


# ---------------------------------------------------------------------------
# TestNoRobotCommandWritten — R007 invariant
# ---------------------------------------------------------------------------

class TestNoRobotCommandWritten:
    """Tree must NEVER write a RobotCommand — only Intent objects."""

    _INTENT_TYPES = (IntentMove, IntentReceive)

    def _assert_no_robot_command(self, bb: RobotBlackboard) -> None:
        intent = bb.current_intent
        if intent is None:
            return
        cls_name = type(intent).__name__
        assert not cls_name.endswith("Command"), (
            f"Tree wrote a RobotCommand ({cls_name}) to blackboard — "
            "only Intent objects are allowed"
        )

    def test_no_robot_command_default_snapshot(self) -> None:
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        self._assert_no_robot_command(bb)

    def test_intent_is_known_intent_type(self) -> None:
        bb = _tick(SNAPSHOT_DEFAULT, _make_supporter_blackboard())
        assert isinstance(bb.current_intent, self._INTENT_TYPES)

    def test_no_robot_command_string_in_source(self) -> None:
        import inspect
        import TeamControl.bt.trees.supporter as mod
        source = inspect.getsource(mod)
        assert "RobotCommand" not in source, (
            "SupporterTree source must not contain the string 'RobotCommand'"
        )


# ---------------------------------------------------------------------------
# TestConditionsReadFromSnapshot — R007: conditions use Snapshot only
# ---------------------------------------------------------------------------

class TestConditionsReadFromSnapshot:
    """Conditions must read from Snapshot; blackboard must not carry world state."""

    def test_blackboard_has_no_ball_position_field(self) -> None:
        import dataclasses
        bb = _make_supporter_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "ball_position" not in field_names

    def test_blackboard_has_no_own_robots_field(self) -> None:
        import dataclasses
        bb = _make_supporter_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "own_robots" not in field_names

    def test_blackboard_has_no_snapshot_field(self) -> None:
        import dataclasses
        bb = _make_supporter_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "snapshot" not in field_names


# ---------------------------------------------------------------------------
# TestBlackboardUpdatedAfterTick — R007: current_intent set after every tick
# ---------------------------------------------------------------------------

class TestBlackboardUpdatedAfterTick:
    """blackboard.current_intent must be set (not None) after every tick."""

    def test_intent_set_after_tick(self) -> None:
        bb = _make_supporter_blackboard()
        assert bb.current_intent is None
        _tick(SNAPSHOT_DEFAULT, bb)
        assert bb.current_intent is not None

    def test_intent_replaced_on_second_tick(self) -> None:
        tree = SupporterTree()
        bb = _make_supporter_blackboard()

        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)
        first_intent = bb.current_intent

        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)
        second_intent = bb.current_intent

        assert second_intent is not None

    def test_three_ticks_all_produce_intents(self) -> None:
        tree = SupporterTree()
        bb = _make_supporter_blackboard()

        for _ in range(3):
            tree.set_snapshot(SNAPSHOT_DEFAULT)
            tree.tick(bb)
            assert bb.current_intent is not None, (
                "current_intent must be set after each tick"
            )


# ---------------------------------------------------------------------------
# TestIsolation — tree works without Coordinator (R007)
# ---------------------------------------------------------------------------

class TestIsolation:
    """Tree can be ticked in isolation with a mock snapshot; no Coordinator needed."""

    def test_fresh_tree_needs_no_coordinator(self) -> None:
        tree = SupporterTree()
        bb = _make_supporter_blackboard()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)  # must not raise

    def test_two_tree_instances_are_independent(self) -> None:
        tree_a = SupporterTree()
        tree_b = SupporterTree()
        bb_a = _make_supporter_blackboard()
        bb_b = _make_supporter_blackboard()

        tree_a.set_snapshot(SNAPSHOT_DEFAULT)
        tree_b.set_snapshot(SNAPSHOT_DEFAULT)

        tree_a.tick(bb_a)
        tree_b.tick(bb_b)

        # Both produce IntentMove in v1
        assert isinstance(bb_a.current_intent, IntentMove)
        assert isinstance(bb_b.current_intent, IntentMove)

    def test_tree_can_be_used_without_py_trees_runner(self) -> None:
        """SupporterTree.tick() must work without py_trees BehaviourTree/Runner."""
        tree = SupporterTree()
        bb = _make_supporter_blackboard()
        tree.set_snapshot(SNAPSHOT_DEFAULT)
        tree.tick(bb)
        assert bb.current_intent is not None
