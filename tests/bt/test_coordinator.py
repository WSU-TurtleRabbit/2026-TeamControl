"""Tests for Coordinator role assignment and tree dispatch — R004.

TDD: tests are written before the implementation.
All tests should fail until src/bt/coordinator.py is implemented.

Convention: mock tree nodes expose ``_blackboard_ref: list`` — a one-element
list that the Coordinator sets to the per-robot RobotBlackboard before
ticking. This is the standard injection interface.
"""
from __future__ import annotations

import pytest
import py_trees

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import IntentMove, IntentOrient
from TeamControl.bt.contracts.snapshot import GamePhase, RefereeState, RobotState, Snapshot


# ---------------------------------------------------------------------------
# Helpers — minimal snapshot and mock behaviour tree nodes
# ---------------------------------------------------------------------------

def _make_snapshot(robot_ids: list[int]) -> Snapshot:
    """Create a Snapshot that contains RobotState for each given robot ID."""
    own_robots = [
        RobotState(robot_id=rid, position=(float(rid), 0.0), orientation=0.0)
        for rid in robot_ids
    ]
    return Snapshot(
        ball_position=(0.0, 0.0),
        ball_velocity=(0.0, 0.0),
        own_robots=own_robots,
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


class FixedIntentBehaviour(py_trees.behaviour.Behaviour):
    """Mock tree node: writes a fixed Intent to the blackboard each tick."""

    def __init__(self, name: str, fixed_intent: object) -> None:
        super().__init__(name)
        self._fixed_intent = fixed_intent
        self._blackboard_ref: list = [None]

    def update(self) -> py_trees.common.Status:
        if self._blackboard_ref[0] is not None:
            self._blackboard_ref[0].current_intent = self._fixed_intent
        return py_trees.common.Status.SUCCESS


def _make_mock_trees(
    intent_map: dict[RoleType, object],
) -> dict[RoleType, FixedIntentBehaviour]:
    """Build one FixedIntentBehaviour per role."""
    return {
        role: FixedIntentBehaviour(role.value, intent_map[role])
        for role in intent_map
    }


# ---------------------------------------------------------------------------
# Coordinator import check
# ---------------------------------------------------------------------------

class TestCoordinatorImport:
    def test_coordinator_is_importable(self):
        from TeamControl.bt.coordinator import Coordinator  # noqa: F401

    def test_coordinator_does_not_import_robot_command(self):
        import inspect
        from src.bt import coordinator as coord_mod
        source = inspect.getsource(coord_mod)
        assert "RobotCommand" not in source, (
            "Coordinator must not reference RobotCommand anywhere"
        )


# ---------------------------------------------------------------------------
# Role assignment
# ---------------------------------------------------------------------------

EXPECTED_ROLE_ASSIGNMENT = {
    0: RoleType.GOALIE,
    1: RoleType.DEFENDER,
    2: RoleType.DEFENDER,
    3: RoleType.SUPPORTER,
    4: RoleType.SUPPORTER,
    5: RoleType.ATTACKER,
}


class TestRoleAssignment:
    def _make_coordinator(self):
        from TeamControl.bt.coordinator import Coordinator

        intent_for = {
            RoleType.GOALIE: IntentOrient(target_orientation=0.0),
            RoleType.DEFENDER: IntentOrient(target_orientation=1.0),
            RoleType.SUPPORTER: IntentOrient(target_orientation=2.0),
            RoleType.ATTACKER: IntentOrient(target_orientation=3.0),
        }
        trees = _make_mock_trees(intent_for)
        return Coordinator(trees=trees)

    def test_index_0_gets_goalie(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([0]), [0])
        assert coord.blackboards[0].current_role == RoleType.GOALIE

    def test_index_1_gets_defender(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([1]), [1])
        assert coord.blackboards[1].current_role == RoleType.DEFENDER

    def test_index_2_gets_defender(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([2]), [2])
        assert coord.blackboards[2].current_role == RoleType.DEFENDER

    def test_index_3_gets_supporter(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([3]), [3])
        assert coord.blackboards[3].current_role == RoleType.SUPPORTER

    def test_index_4_gets_supporter(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([4]), [4])
        assert coord.blackboards[4].current_role == RoleType.SUPPORTER

    def test_index_5_gets_attacker(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([5]), [5])
        assert coord.blackboards[5].current_role == RoleType.ATTACKER

    def test_all_roles_assigned_for_full_team(self):
        coord = self._make_coordinator()
        robot_ids = list(range(6))
        coord.tick(_make_snapshot(robot_ids), robot_ids)
        for idx, expected_role in EXPECTED_ROLE_ASSIGNMENT.items():
            assert coord.blackboards[idx].current_role == expected_role


# ---------------------------------------------------------------------------
# Correct tree dispatched per role
# ---------------------------------------------------------------------------

class TestCorrectTreeDispatched:
    """The Coordinator must tick the tree registered for the robot's role."""

    def _make_coordinator_with_tick_counters(self):
        from TeamControl.bt.coordinator import Coordinator

        tick_counts: dict[RoleType, int] = {role: 0 for role in RoleType}

        class CountingBehaviour(py_trees.behaviour.Behaviour):
            def __init__(self, role: RoleType) -> None:
                super().__init__(role.value)
                self._role = role
                self._blackboard_ref: list = [None]

            def update(self) -> py_trees.common.Status:
                tick_counts[self._role] += 1
                if self._blackboard_ref[0] is not None:
                    self._blackboard_ref[0].current_intent = IntentOrient(0.0)
                return py_trees.common.Status.SUCCESS

        trees = {role: CountingBehaviour(role) for role in RoleType}
        coord = Coordinator(trees=trees)
        return coord, tick_counts

    def test_only_goalie_tree_ticked_for_index_0(self):
        coord, tick_counts = self._make_coordinator_with_tick_counters()
        coord.tick(_make_snapshot([0]), [0])
        assert tick_counts[RoleType.GOALIE] == 1
        assert tick_counts[RoleType.ATTACKER] == 0
        assert tick_counts[RoleType.DEFENDER] == 0
        assert tick_counts[RoleType.SUPPORTER] == 0

    def test_correct_trees_ticked_for_full_team(self):
        coord, tick_counts = self._make_coordinator_with_tick_counters()
        robot_ids = list(range(6))
        coord.tick(_make_snapshot(robot_ids), robot_ids)
        assert tick_counts[RoleType.GOALIE] == 1
        assert tick_counts[RoleType.ATTACKER] == 1
        assert tick_counts[RoleType.DEFENDER] == 2   # robots 1 and 2
        assert tick_counts[RoleType.SUPPORTER] == 2  # robots 3 and 4


# ---------------------------------------------------------------------------
# Returns list[Intent] of correct length
# ---------------------------------------------------------------------------

class TestReturnType:
    def _coord_with_fixed_intents(self):
        from TeamControl.bt.coordinator import Coordinator

        intent_for = {
            RoleType.GOALIE: IntentOrient(target_orientation=0.0),
            RoleType.DEFENDER: IntentOrient(target_orientation=1.0),
            RoleType.SUPPORTER: IntentOrient(target_orientation=2.0),
            RoleType.ATTACKER: IntentOrient(target_orientation=3.0),
        }
        return Coordinator(trees=_make_mock_trees(intent_for))

    def test_returns_list(self):
        coord = self._coord_with_fixed_intents()
        result = coord.tick(_make_snapshot([0]), [0])
        assert isinstance(result, list)

    def test_length_matches_robot_count(self):
        coord = self._coord_with_fixed_intents()
        result = coord.tick(_make_snapshot(list(range(6))), list(range(6)))
        assert len(result) == 6

    def test_single_robot_returns_one_intent(self):
        coord = self._coord_with_fixed_intents()
        result = coord.tick(_make_snapshot([0]), [0])
        assert len(result) == 1

    def test_each_element_is_intent(self):
        from TeamControl.bt.contracts.intent import (
            IntentDribble, IntentKick, IntentMove,
            IntentOrient, IntentPass, IntentReceive,
        )
        INTENT_TYPES = (IntentDribble, IntentKick, IntentMove,
                        IntentOrient, IntentPass, IntentReceive)
        coord = self._coord_with_fixed_intents()
        result = coord.tick(_make_snapshot(list(range(6))), list(range(6)))
        for item in result:
            assert isinstance(item, INTENT_TYPES), (
                f"Expected an Intent variant, got {type(item)}"
            )

    def test_no_robot_command_in_output(self):
        coord = self._coord_with_fixed_intents()
        result = coord.tick(_make_snapshot(list(range(6))), list(range(6)))
        for item in result:
            assert not type(item).__name__.endswith("Command"), (
                f"RobotCommand found in output: {type(item)}"
            )


# ---------------------------------------------------------------------------
# Skips missing robot IDs gracefully
# ---------------------------------------------------------------------------

class TestMissingRobotHandling:
    def _make_coordinator(self):
        from TeamControl.bt.coordinator import Coordinator
        intent_for = {
            RoleType.GOALIE: IntentOrient(target_orientation=0.0),
            RoleType.DEFENDER: IntentOrient(target_orientation=1.0),
            RoleType.SUPPORTER: IntentOrient(target_orientation=2.0),
            RoleType.ATTACKER: IntentOrient(target_orientation=3.0),
        }
        return Coordinator(trees=_make_mock_trees(intent_for))

    def test_missing_robot_does_not_crash(self):
        coord = self._make_coordinator()
        result = coord.tick(_make_snapshot([0]), [0, 99])
        assert len(result) == 1

    def test_all_missing_robots_returns_empty_list(self):
        coord = self._make_coordinator()
        result = coord.tick(_make_snapshot([]), [0, 1, 2])
        assert result == []

    def test_present_robots_still_produce_intents_when_some_missing(self):
        coord = self._make_coordinator()
        result = coord.tick(_make_snapshot([0, 1]), [0, 1, 2, 3])
        assert len(result) == 2

    def test_missing_robot_not_in_blackboards(self):
        coord = self._make_coordinator()
        coord.tick(_make_snapshot([0]), [0, 99])
        assert 99 not in coord.blackboards


# ---------------------------------------------------------------------------
# Tree instances reused across ticks
# ---------------------------------------------------------------------------

class TestTreeReuse:
    def test_same_tree_instance_used_across_ticks(self):
        from TeamControl.bt.coordinator import Coordinator

        tick_counts: dict[str, int] = {"goalie": 0}

        class GoalieBehaviour(py_trees.behaviour.Behaviour):
            def __init__(self) -> None:
                super().__init__("GoalieTree")
                self._blackboard_ref: list = [None]
                GoalieBehaviour.instance_count = getattr(
                    GoalieBehaviour, "instance_count", 0
                ) + 1

            def update(self) -> py_trees.common.Status:
                tick_counts["goalie"] += 1
                if self._blackboard_ref[0] is not None:
                    self._blackboard_ref[0].current_intent = IntentOrient(0.0)
                return py_trees.common.Status.SUCCESS

        GoalieBehaviour.instance_count = 0
        goalie_node = GoalieBehaviour()
        instance_count_at_construction = GoalieBehaviour.instance_count

        trees = {
            RoleType.GOALIE: goalie_node,
            RoleType.DEFENDER: FixedIntentBehaviour("def", IntentOrient(1.0)),
            RoleType.SUPPORTER: FixedIntentBehaviour("sup", IntentOrient(2.0)),
            RoleType.ATTACKER: FixedIntentBehaviour("att", IntentOrient(3.0)),
        }
        coord = Coordinator(trees=trees)
        snapshot = _make_snapshot([0])
        coord.tick(snapshot, [0])
        coord.tick(snapshot, [0])
        coord.tick(snapshot, [0])

        assert GoalieBehaviour.instance_count == instance_count_at_construction, (
            "Coordinator must not re-instantiate tree nodes"
        )
        assert tick_counts["goalie"] == 3

    def test_blackboard_persists_between_ticks(self):
        from TeamControl.bt.coordinator import Coordinator

        intent_a = IntentOrient(target_orientation=1.0)
        intent_b = IntentOrient(target_orientation=2.0)
        intents: list[object] = [intent_a, intent_b]
        call_count = [0]

        class SequenceBehaviour(py_trees.behaviour.Behaviour):
            def __init__(self) -> None:
                super().__init__("seq")
                self._blackboard_ref: list = [None]

            def update(self) -> py_trees.common.Status:
                idx = min(call_count[0], len(intents) - 1)
                if self._blackboard_ref[0] is not None:
                    self._blackboard_ref[0].current_intent = intents[idx]
                call_count[0] += 1
                return py_trees.common.Status.SUCCESS

        trees = {
            RoleType.GOALIE: SequenceBehaviour(),
            RoleType.DEFENDER: FixedIntentBehaviour("def", IntentOrient(0.0)),
            RoleType.SUPPORTER: FixedIntentBehaviour("sup", IntentOrient(0.0)),
            RoleType.ATTACKER: FixedIntentBehaviour("att", IntentOrient(0.0)),
        }
        coord = Coordinator(trees=trees)
        snapshot = _make_snapshot([0])

        coord.tick(snapshot, [0])
        assert coord.blackboards[0].current_intent == intent_a

        coord.tick(snapshot, [0])
        assert coord.blackboards[0].last_intent == intent_a
        assert coord.blackboards[0].current_intent == intent_b


# ---------------------------------------------------------------------------
# Blackboard injection — tree node receives the correct per-robot blackboard
# ---------------------------------------------------------------------------

class TestBlackboardInjection:
    def test_each_robot_gets_own_blackboard(self):
        from TeamControl.bt.coordinator import Coordinator

        seen_bbs: dict[str, list] = {"goalie": [], "defender": []}

        class CapturingBehaviour(py_trees.behaviour.Behaviour):
            def __init__(self, name: str, bucket: list) -> None:
                super().__init__(name)
                self._blackboard_ref: list = [None]
                self._bucket = bucket

            def update(self) -> py_trees.common.Status:
                if self._blackboard_ref[0] is not None:
                    self._blackboard_ref[0].current_intent = IntentOrient(0.0)
                    self._bucket.append(self._blackboard_ref[0])
                return py_trees.common.Status.SUCCESS

        trees = {
            RoleType.GOALIE: CapturingBehaviour("goalie", seen_bbs["goalie"]),
            RoleType.DEFENDER: CapturingBehaviour("defender", seen_bbs["defender"]),
            RoleType.SUPPORTER: FixedIntentBehaviour("sup", IntentOrient(0.0)),
            RoleType.ATTACKER: FixedIntentBehaviour("att", IntentOrient(0.0)),
        }
        coord = Coordinator(trees=trees)
        coord.tick(_make_snapshot([0, 1, 2]), [0, 1, 2])

        assert len(seen_bbs["goalie"]) == 1
        assert seen_bbs["goalie"][0].robot_id == 0

        assert len(seen_bbs["defender"]) >= 1
        defender_ids = {bb.robot_id for bb in seen_bbs["defender"]}
        assert 1 in defender_ids
