"""Tests for the Goalie behaviour tree — R008.

TDD: these tests are written BEFORE the implementation exists.
All tests in this file must fail with ImportError (or a collection error
that wraps ImportError) until ``src/bt/trees/goalie.py`` is created.

Tree topology under test (from spec R008 / docs/goalie_node.png):

    GoalieSequenceNode (Sequence)
    ├── LookAtBall          → writes IntentOrient(angle_to_ball)
    └── GoalieBallSequence (Sequence)
        ├── GetBallHistory  → stores ball position as single-frame history
        └── DoBallTrajectory → returns neutral goal pos (-4.0, 0.0) for v1
    └── GoToTarget          → writes IntentMove(target_pos=neutral_goal_pos)
        [IsBallComing stub: always FAILURE — goalie stays at neutral position]

Usage contract:

    from TeamControl.bt.trees.goalie import GoalieTree

    tree = GoalieTree()
    tree.set_snapshot(snapshot)   # inject world state before tick
    tree.tick(blackboard)          # run the tree; writes intent to blackboard
    intent = blackboard.current_intent
"""
from __future__ import annotations

import math
import pytest
import py_trees

# --- import under test (will ImportError until T014 is implemented) ----------
from TeamControl.bt.trees.goalie import GoalieTree  # noqa: E402

# --- contracts ---------------------------------------------------------------
from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    Intent,
    IntentMove,
    IntentOrient,
    IntentKick,
    IntentPass,
    IntentDribble,
)
from TeamControl.bt.contracts.snapshot import (
    GamePhase,
    RefereeState,
    RobotState,
    Snapshot,
)


# ---------------------------------------------------------------------------
# Constants (must match goalie.py)
# ---------------------------------------------------------------------------

NEUTRAL_GOAL_POSITION: tuple[float, float] = (-4.0, 0.0)
GOALIE_ROBOT_ID: int = 0


# ---------------------------------------------------------------------------
# Snapshot helpers
# ---------------------------------------------------------------------------

_BALL_RIGHT: tuple[float, float] = (2.0, 1.0)   # ball somewhere on field
_BALL_CENTRE: tuple[float, float] = (0.0, 0.0)  # ball at centre


def make_goalie_snapshot(
    ball_pos: tuple[float, float] = _BALL_CENTRE,
    goalie_pos: tuple[float, float] = (-3.5, 0.0),
    goalie_orientation: float = 0.0,
) -> Snapshot:
    """Build a Snapshot with a single goalie robot."""
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=(0.0, 0.0),
        own_robots=[
            RobotState(
                robot_id=GOALIE_ROBOT_ID,
                position=goalie_pos,
                orientation=goalie_orientation,
            )
        ],
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


def _make_goalie_blackboard() -> RobotBlackboard:
    """Return a fresh RobotBlackboard for the goalie robot."""
    return RobotBlackboard(robot_id=GOALIE_ROBOT_ID, current_role=RoleType.GOALIE)


def _tick(snapshot: Snapshot, blackboard: RobotBlackboard) -> RobotBlackboard:
    """Convenience: create a tree, inject snapshot, tick, return the blackboard."""
    tree = GoalieTree()
    tree.set_snapshot(snapshot)
    tree.tick(blackboard)
    return blackboard


# ---------------------------------------------------------------------------
# Scenario snapshots (reused across test classes)
# ---------------------------------------------------------------------------

SNAPSHOT_BALL_CENTRE = make_goalie_snapshot(
    ball_pos=_BALL_CENTRE,
    goalie_pos=(-3.5, 0.0),
)

SNAPSHOT_BALL_RIGHT = make_goalie_snapshot(
    ball_pos=_BALL_RIGHT,
    goalie_pos=(-3.5, 0.0),
)

SNAPSHOT_BALL_LEFT = make_goalie_snapshot(
    ball_pos=(-1.0, -2.0),
    goalie_pos=(-3.5, 0.0),
)


# ---------------------------------------------------------------------------
# TestGoalieTreeImport — basic smoke checks
# ---------------------------------------------------------------------------

class TestGoalieTreeImport:
    """GoalieTree must be importable and instantiable without side effects."""

    def test_goalie_tree_is_importable(self) -> None:
        from TeamControl.bt.trees.goalie import GoalieTree as GT  # noqa: F401

    def test_goalie_tree_instantiates(self) -> None:
        tree = GoalieTree()
        assert tree is not None

    def test_has_set_snapshot_method(self) -> None:
        tree = GoalieTree()
        assert callable(getattr(tree, "set_snapshot", None))

    def test_has_tick_method(self) -> None:
        tree = GoalieTree()
        assert callable(getattr(tree, "tick", None))

    def test_does_not_import_robot_command(self) -> None:
        import inspect
        import TeamControl.bt.trees.goalie as mod
        source = inspect.getsource(mod)
        assert "RobotCommand" not in source, (
            "GoalieTree must not reference RobotCommand anywhere in source"
        )


# ---------------------------------------------------------------------------
# TestGoalieTreeTopology — py_trees structure
# ---------------------------------------------------------------------------

class TestGoalieTreeTopology:
    """Root node must be a py_trees Sequence named 'GoalieSequenceNode'."""

    def test_root_is_sequence(self) -> None:
        tree = GoalieTree()
        assert isinstance(tree.root, py_trees.composites.Sequence), (
            f"Expected py_trees.composites.Sequence, got {type(tree.root)}"
        )

    def test_root_name(self) -> None:
        tree = GoalieTree()
        assert tree.root.name == "GoalieSequenceNode"

    def test_root_has_three_children(self) -> None:
        tree = GoalieTree()
        assert len(tree.root.children) == 3, (
            "Root Sequence must have exactly 3 children: "
            "LookAtBall, GoalieBallSequence, GoToTarget"
        )

    def test_first_child_name_is_look_at_ball(self) -> None:
        tree = GoalieTree()
        first = tree.root.children[0]
        assert first.name == "LookAtBall"

    def test_second_child_is_sequence(self) -> None:
        """GoalieBallSequence must be a Sequence."""
        tree = GoalieTree()
        second = tree.root.children[1]
        assert isinstance(second, py_trees.composites.Sequence), (
            f"Second child must be Sequence (GoalieBallSequence), got {type(second)}"
        )

    def test_second_child_name(self) -> None:
        tree = GoalieTree()
        second = tree.root.children[1]
        assert second.name == "GoalieBallSequence"

    def test_third_child_name_is_go_to_target(self) -> None:
        tree = GoalieTree()
        third = tree.root.children[2]
        assert third.name == "GoToTarget"

    def test_goalie_ball_sequence_has_two_children(self) -> None:
        """GoalieBallSequence must contain GetBallHistory and DoBallTrajectory."""
        tree = GoalieTree()
        ball_seq = tree.root.children[1]
        assert len(ball_seq.children) == 2, (
            "GoalieBallSequence must have exactly 2 children: "
            "GetBallHistory and DoBallTrajectory"
        )

    def test_get_ball_history_is_first_child_of_ball_sequence(self) -> None:
        tree = GoalieTree()
        ball_seq = tree.root.children[1]
        assert ball_seq.children[0].name == "GetBallHistory"

    def test_do_ball_trajectory_is_second_child_of_ball_sequence(self) -> None:
        tree = GoalieTree()
        ball_seq = tree.root.children[1]
        assert ball_seq.children[1].name == "DoBallTrajectory"


# ---------------------------------------------------------------------------
# TestGoalieTreeTickInterface
# ---------------------------------------------------------------------------

class TestGoalieTreeTickInterface:
    """set_snapshot + tick compose correctly; blackboard.current_intent is set."""

    def test_tick_sets_current_intent(self) -> None:
        bb = _make_goalie_blackboard()
        assert bb.current_intent is None
        bb = _tick(SNAPSHOT_BALL_CENTRE, bb)
        assert bb.current_intent is not None

    def test_tick_does_not_raise_on_any_scenario(self) -> None:
        for snap in (
            SNAPSHOT_BALL_CENTRE,
            SNAPSHOT_BALL_RIGHT,
            SNAPSHOT_BALL_LEFT,
        ):
            bb = _make_goalie_blackboard()
            _tick(snap, bb)  # must not raise

    def test_multiple_ticks_do_not_raise(self) -> None:
        tree = GoalieTree()
        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        bb = _make_goalie_blackboard()
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_BALL_RIGHT)
        tree.tick(bb)
        tree.set_snapshot(SNAPSHOT_BALL_LEFT)
        tree.tick(bb)

    def test_snapshot_can_be_replaced_between_ticks(self) -> None:
        tree = GoalieTree()
        bb = _make_goalie_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree.tick(bb)
        assert bb.current_intent is not None

        tree.set_snapshot(SNAPSHOT_BALL_RIGHT)
        tree.tick(bb)
        assert bb.current_intent is not None

    def test_tick_without_coordinator(self) -> None:
        """Tree must be usable in isolation — no Coordinator required."""
        bb = _make_goalie_blackboard()
        tree = GoalieTree()
        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree.tick(bb)


# ---------------------------------------------------------------------------
# TestLookAtBall — LookAtBall writes IntentOrient
# ---------------------------------------------------------------------------

class TestLookAtBall:
    """LookAtBall must write IntentOrient(target_orientation=angle_to_ball)."""

    def test_look_at_ball_produces_intent_orient(self) -> None:
        """After tick, blackboard holds an IntentOrient (LookAtBall fires first)."""
        # We verify that the tree *eventually* writes an IntentOrient.
        # Since the full sequence succeeds, GoToTarget overwrites with IntentMove.
        # We test LookAtBall by inspecting the node directly.
        import TeamControl.bt.trees.goalie as mod
        tree = GoalieTree()
        snap = SNAPSHOT_BALL_RIGHT
        tree.set_snapshot(snap)
        bb = _make_goalie_blackboard()
        tree._blackboard_ref[0] = bb

        look_node = tree.root.children[0]
        result = look_node.update()
        assert result == py_trees.common.Status.SUCCESS, (
            f"LookAtBall.update() must return SUCCESS, got {result}"
        )
        assert isinstance(bb.current_intent, IntentOrient), (
            f"LookAtBall must write IntentOrient, got {type(bb.current_intent)}"
        )

    def test_look_at_ball_angle_is_correct(self) -> None:
        """IntentOrient angle must equal atan2(ball_y - robot_y, ball_x - robot_x)."""
        goalie_pos = (-3.5, 0.0)
        ball_pos = (2.0, 1.0)
        expected_angle = math.atan2(
            ball_pos[1] - goalie_pos[1],
            ball_pos[0] - goalie_pos[0],
        )
        snap = make_goalie_snapshot(ball_pos=ball_pos, goalie_pos=goalie_pos)
        tree = GoalieTree()
        tree.set_snapshot(snap)
        bb = _make_goalie_blackboard()
        tree._blackboard_ref[0] = bb

        look_node = tree.root.children[0]
        look_node.update()
        assert isinstance(bb.current_intent, IntentOrient)
        assert abs(bb.current_intent.target_orientation - expected_angle) < 1e-9, (
            f"Expected angle {expected_angle}, got {bb.current_intent.target_orientation}"
        )

    def test_look_at_ball_angle_updates_with_ball_position(self) -> None:
        """Different ball positions produce different orient angles."""
        goalie_pos = (-3.5, 0.0)
        ball_a = (2.0, 0.0)
        ball_b = (0.0, 2.0)

        angles = []
        for ball_pos in (ball_a, ball_b):
            snap = make_goalie_snapshot(ball_pos=ball_pos, goalie_pos=goalie_pos)
            tree = GoalieTree()
            tree.set_snapshot(snap)
            bb = _make_goalie_blackboard()
            tree._blackboard_ref[0] = bb
            tree.root.children[0].update()
            assert isinstance(bb.current_intent, IntentOrient)
            angles.append(bb.current_intent.target_orientation)

        assert angles[0] != angles[1], (
            "Different ball positions must produce different orient angles"
        )


# ---------------------------------------------------------------------------
# TestGoToTarget — GoToTarget writes IntentMove to neutral goal pos
# ---------------------------------------------------------------------------

class TestGoToTarget:
    """After a full tick, blackboard must hold IntentMove(target_pos=NEUTRAL_GOAL_POSITION)."""

    def test_full_tick_produces_intent_move(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        assert isinstance(bb.current_intent, IntentMove), (
            f"Expected IntentMove after full goalie tick, got {type(bb.current_intent)}"
        )

    def test_intent_move_target_is_neutral_goal_position(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        assert isinstance(bb.current_intent, IntentMove)
        assert bb.current_intent.target_pos == NEUTRAL_GOAL_POSITION, (
            f"Expected target_pos={NEUTRAL_GOAL_POSITION}, "
            f"got {bb.current_intent.target_pos}"
        )

    def test_neutral_pos_consistent_across_ball_positions(self) -> None:
        """GoToTarget always writes the same neutral pos regardless of ball location."""
        for snap in (SNAPSHOT_BALL_CENTRE, SNAPSHOT_BALL_RIGHT, SNAPSHOT_BALL_LEFT):
            bb = _tick(snap, _make_goalie_blackboard())
            assert isinstance(bb.current_intent, IntentMove)
            assert bb.current_intent.target_pos == NEUTRAL_GOAL_POSITION, (
                f"Expected neutral pos {NEUTRAL_GOAL_POSITION}, "
                f"got {bb.current_intent.target_pos} for snapshot {snap}"
            )

    def test_intent_move_not_intent_kick(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        assert not isinstance(bb.current_intent, IntentKick)

    def test_intent_move_not_intent_pass(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        assert not isinstance(bb.current_intent, IntentPass)

    def test_intent_move_not_intent_dribble(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        assert not isinstance(bb.current_intent, IntentDribble)


# ---------------------------------------------------------------------------
# TestIsBallComingStub — IsBallComing always returns FAILURE
# ---------------------------------------------------------------------------

class TestIsBallComingStub:
    """IsBallComing is a stub that always returns FAILURE.

    Because IsBallComing always fails, GoToTarget writes neutral pos every tick.
    """

    def test_is_ball_coming_node_exists_with_correct_name(self) -> None:
        """GoalieTree must expose an IsBallComing node."""
        tree = GoalieTree()
        assert hasattr(tree, "is_ball_coming_node"), (
            "GoalieTree must have attribute 'is_ball_coming_node'"
        )

    def test_is_ball_coming_always_fails(self) -> None:
        """IsBallComing.update() must return FAILURE unconditionally."""
        tree = GoalieTree()
        snap = SNAPSHOT_BALL_RIGHT
        tree.set_snapshot(snap)
        bb = _make_goalie_blackboard()
        tree._blackboard_ref[0] = bb

        node = tree.is_ball_coming_node
        result = node.update()
        assert result == py_trees.common.Status.FAILURE, (
            f"IsBallComing stub must always return FAILURE, got {result}"
        )

    def test_is_ball_coming_stub_marked_in_source(self) -> None:
        """Source must contain a TODO comment about wiring DoBallTrajectory."""
        import inspect
        import TeamControl.bt.trees.goalie as mod
        source = inspect.getsource(mod)
        assert "TODO" in source and "DoBallTrajectory" in source, (
            "IsBallComing stub must be marked with "
            "'# TODO: wire DoBallTrajectory result' comment"
        )


# ---------------------------------------------------------------------------
# TestGetBallHistory — stores ball position from snapshot
# ---------------------------------------------------------------------------

class TestGetBallHistory:
    """GetBallHistory must store snap.ball_position as 'history' on the tree."""

    def test_get_ball_history_stores_ball_position(self) -> None:
        tree = GoalieTree()
        snap = SNAPSHOT_BALL_RIGHT
        tree.set_snapshot(snap)
        bb = _make_goalie_blackboard()
        tree._blackboard_ref[0] = bb

        get_history_node = tree.root.children[1].children[0]
        result = get_history_node.update()
        assert result == py_trees.common.Status.SUCCESS, (
            f"GetBallHistory.update() must return SUCCESS, got {result}"
        )
        assert tree.ball_history is not None, (
            "GetBallHistory must set tree.ball_history"
        )
        assert tree.ball_history == snap.ball_position, (
            f"ball_history must equal snap.ball_position={snap.ball_position}, "
            f"got {tree.ball_history}"
        )

    def test_ball_history_updates_each_tick(self) -> None:
        """ball_history must reflect the most recent snapshot after each tick."""
        tree = GoalieTree()
        bb = _make_goalie_blackboard()

        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree.tick(bb)
        assert tree.ball_history == SNAPSHOT_BALL_CENTRE.ball_position

        tree.set_snapshot(SNAPSHOT_BALL_RIGHT)
        tree.tick(bb)
        assert tree.ball_history == SNAPSHOT_BALL_RIGHT.ball_position


# ---------------------------------------------------------------------------
# TestDoBallTrajectory — v1 returns neutral goal position
# ---------------------------------------------------------------------------

class TestDoBallTrajectory:
    """DoBallTrajectory must set tree.predicted_intercept to NEUTRAL_GOAL_POSITION."""

    def test_do_ball_trajectory_sets_predicted_intercept(self) -> None:
        tree = GoalieTree()
        snap = SNAPSHOT_BALL_RIGHT
        tree.set_snapshot(snap)
        bb = _make_goalie_blackboard()
        tree.tick(bb)
        assert hasattr(tree, "predicted_intercept"), (
            "GoalieTree must have attribute 'predicted_intercept' after tick"
        )
        assert tree.predicted_intercept == NEUTRAL_GOAL_POSITION, (
            f"predicted_intercept must be {NEUTRAL_GOAL_POSITION}, "
            f"got {tree.predicted_intercept}"
        )

    def test_predicted_intercept_is_neutral_regardless_of_ball_pos(self) -> None:
        """v1 always returns neutral goal pos, ignoring actual ball trajectory."""
        for snap in (SNAPSHOT_BALL_CENTRE, SNAPSHOT_BALL_RIGHT, SNAPSHOT_BALL_LEFT):
            tree = GoalieTree()
            bb = _make_goalie_blackboard()
            tree.set_snapshot(snap)
            tree.tick(bb)
            assert tree.predicted_intercept == NEUTRAL_GOAL_POSITION


# ---------------------------------------------------------------------------
# TestNoRobotCommandWritten — R008 invariant
# ---------------------------------------------------------------------------

class TestNoRobotCommandWritten:
    """Tree must NEVER write a RobotCommand — only Intent objects."""

    _INTENT_TYPES = (IntentMove, IntentOrient, IntentKick, IntentPass, IntentDribble)

    def _assert_no_robot_command(self, bb: RobotBlackboard) -> None:
        intent = bb.current_intent
        if intent is None:
            return
        cls_name = type(intent).__name__
        assert not cls_name.endswith("Command"), (
            f"Tree wrote a RobotCommand ({cls_name}) to blackboard — "
            "only Intent objects are allowed"
        )

    def test_no_robot_command_ball_centre(self) -> None:
        bb = _tick(SNAPSHOT_BALL_CENTRE, _make_goalie_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_ball_right(self) -> None:
        bb = _tick(SNAPSHOT_BALL_RIGHT, _make_goalie_blackboard())
        self._assert_no_robot_command(bb)

    def test_no_robot_command_ball_left(self) -> None:
        bb = _tick(SNAPSHOT_BALL_LEFT, _make_goalie_blackboard())
        self._assert_no_robot_command(bb)

    def test_intent_is_known_type(self) -> None:
        for snap in (SNAPSHOT_BALL_CENTRE, SNAPSHOT_BALL_RIGHT, SNAPSHOT_BALL_LEFT):
            bb = _tick(snap, _make_goalie_blackboard())
            assert isinstance(bb.current_intent, self._INTENT_TYPES), (
                f"Unknown intent type: {type(bb.current_intent)}"
            )


# ---------------------------------------------------------------------------
# TestConditionsReadFromSnapshot — R008: conditions use Snapshot, not blackboard
# ---------------------------------------------------------------------------

class TestConditionsReadFromSnapshot:
    """Conditions must read from Snapshot; blackboard must not carry world state."""

    def test_blackboard_has_no_ball_position_field(self) -> None:
        import dataclasses
        bb = _make_goalie_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "ball_position" not in field_names

    def test_blackboard_has_no_own_robots_field(self) -> None:
        import dataclasses
        bb = _make_goalie_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "own_robots" not in field_names

    def test_blackboard_has_no_snapshot_field(self) -> None:
        import dataclasses
        bb = _make_goalie_blackboard()
        field_names = {f.name for f in dataclasses.fields(bb)}
        assert "snapshot" not in field_names


# ---------------------------------------------------------------------------
# TestIsolation — tree works without Coordinator (R008)
# ---------------------------------------------------------------------------

class TestIsolation:
    """Tree can be ticked in isolation with a mock snapshot; no Coordinator needed."""

    def test_fresh_tree_needs_no_coordinator(self) -> None:
        tree = GoalieTree()
        bb = _make_goalie_blackboard()
        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree.tick(bb)  # must not raise

    def test_two_tree_instances_are_independent(self) -> None:
        tree_a = GoalieTree()
        tree_b = GoalieTree()
        bb_a = _make_goalie_blackboard()
        bb_b = _make_goalie_blackboard()

        tree_a.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree_b.set_snapshot(SNAPSHOT_BALL_RIGHT)

        tree_a.tick(bb_a)
        tree_b.tick(bb_b)

        # Both should produce IntentMove to neutral position
        assert isinstance(bb_a.current_intent, IntentMove)
        assert isinstance(bb_b.current_intent, IntentMove)
        assert bb_a.current_intent.target_pos == NEUTRAL_GOAL_POSITION
        assert bb_b.current_intent.target_pos == NEUTRAL_GOAL_POSITION

    def test_tree_can_be_used_without_py_trees_runner(self) -> None:
        """GoalieTree.tick() must work without py_trees BehaviourTree/Runner."""
        tree = GoalieTree()
        bb = _make_goalie_blackboard()
        tree.set_snapshot(SNAPSHOT_BALL_CENTRE)
        tree.tick(bb)
        assert bb.current_intent is not None


# ---------------------------------------------------------------------------
# TestBlackboardUpdatedAfterTick — R008: current_intent updated after each tick
# ---------------------------------------------------------------------------

class TestBlackboardUpdatedAfterTick:
    """blackboard.current_intent must be set (not None) after every tick."""

    def test_intent_set_after_tick_ball_centre(self) -> None:
        bb = _make_goalie_blackboard()
        assert bb.current_intent is None
        _tick(SNAPSHOT_BALL_CENTRE, bb)
        assert bb.current_intent is not None

    def test_intent_set_after_tick_ball_right(self) -> None:
        bb = _make_goalie_blackboard()
        _tick(SNAPSHOT_BALL_RIGHT, bb)
        assert bb.current_intent is not None

    def test_intent_set_after_tick_ball_left(self) -> None:
        bb = _make_goalie_blackboard()
        _tick(SNAPSHOT_BALL_LEFT, bb)
        assert bb.current_intent is not None

    def test_three_ticks_all_produce_intents(self) -> None:
        tree = GoalieTree()
        bb = _make_goalie_blackboard()

        for snap in (
            SNAPSHOT_BALL_CENTRE,
            SNAPSHOT_BALL_RIGHT,
            SNAPSHOT_BALL_LEFT,
        ):
            tree.set_snapshot(snap)
            tree.tick(bb)
            assert bb.current_intent is not None, (
                f"current_intent must be set after tick with snapshot {snap}"
            )
