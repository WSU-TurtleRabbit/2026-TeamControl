"""Tests for RobotBlackboard dataclass and RoleType enum — R003.

TDD: these tests are written before the implementation.
All tests should fail until src/bt/contracts/blackboard.py is implemented.
"""
from __future__ import annotations

import dataclasses

import pytest

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType


# ---------------------------------------------------------------------------
# RoleType enum
# ---------------------------------------------------------------------------

class TestRoleType:
    def test_attacker_value(self):
        assert RoleType.ATTACKER.value == "ATTACKER"

    def test_defender_value(self):
        assert RoleType.DEFENDER.value == "DEFENDER"

    def test_supporter_value(self):
        assert RoleType.SUPPORTER.value == "SUPPORTER"

    def test_goalie_value(self):
        assert RoleType.GOALIE.value == "GOALIE"

    def test_all_members_present(self):
        names = {m.name for m in RoleType}
        assert names == {"ATTACKER", "DEFENDER", "SUPPORTER", "GOALIE"}


# ---------------------------------------------------------------------------
# RobotBlackboard construction
# ---------------------------------------------------------------------------

class TestRobotBlackboardConstruction:
    def test_construction_with_robot_id_and_role(self):
        bb = RobotBlackboard(robot_id=3, current_role=RoleType.ATTACKER)
        assert bb.robot_id == 3
        assert bb.current_role == RoleType.ATTACKER

    def test_current_intent_defaults_to_none(self):
        bb = RobotBlackboard(robot_id=0, current_role=RoleType.DEFENDER)
        assert bb.current_intent is None

    def test_last_intent_defaults_to_none(self):
        bb = RobotBlackboard(robot_id=0, current_role=RoleType.DEFENDER)
        assert bb.last_intent is None

    def test_is_dataclass(self):
        assert dataclasses.is_dataclass(RobotBlackboard)

    def test_is_not_frozen(self):
        """RobotBlackboard must be mutable — it is written to each tick."""
        bb = RobotBlackboard(robot_id=1, current_role=RoleType.GOALIE)
        # Should NOT raise — mutable dataclass
        bb.current_role = RoleType.SUPPORTER
        assert bb.current_role == RoleType.SUPPORTER


# ---------------------------------------------------------------------------
# Mutability — current_intent and last_intent can be updated
# ---------------------------------------------------------------------------

class TestRobotBlackboardMutability:
    def test_can_update_current_intent(self):
        bb = RobotBlackboard(robot_id=1, current_role=RoleType.ATTACKER)
        sentinel = object()
        bb.current_intent = sentinel  # type: ignore[assignment]
        assert bb.current_intent is sentinel

    def test_can_shift_last_intent(self):
        """Simulate the tick update: last_intent = current_intent, then set new current."""
        bb = RobotBlackboard(robot_id=2, current_role=RoleType.SUPPORTER)
        first_intent = object()
        second_intent = object()

        bb.current_intent = first_intent  # type: ignore[assignment]
        # Tick: shift
        bb.last_intent = bb.current_intent
        bb.current_intent = second_intent  # type: ignore[assignment]

        assert bb.current_intent is second_intent
        assert bb.last_intent is first_intent

    def test_can_clear_current_intent_to_none(self):
        bb = RobotBlackboard(robot_id=3, current_role=RoleType.GOALIE)
        bb.current_intent = object()  # type: ignore[assignment]
        bb.current_intent = None
        assert bb.current_intent is None


# ---------------------------------------------------------------------------
# No world state fields
# ---------------------------------------------------------------------------

class TestNoWorldStateFields:
    FORBIDDEN_FIELD_NAMES = {
        "ball_position",
        "ball_velocity",
        "own_robots",
        "opponent_robots",
        "referee_state",
        "snapshot",
        "game_phase",
        "score",
    }

    def test_no_world_state_fields(self):
        field_names = {f.name for f in dataclasses.fields(RobotBlackboard)}
        overlap = field_names & self.FORBIDDEN_FIELD_NAMES
        assert overlap == set(), f"World-state fields found on blackboard: {overlap}"

    def test_only_expected_fields_present(self):
        field_names = {f.name for f in dataclasses.fields(RobotBlackboard)}
        assert field_names == {"robot_id", "current_role", "current_intent", "last_intent"}


# ---------------------------------------------------------------------------
# Instance isolation — two blackboards don't share state
# ---------------------------------------------------------------------------

class TestInstanceIsolation:
    def test_two_instances_are_independent(self):
        bb1 = RobotBlackboard(robot_id=1, current_role=RoleType.ATTACKER)
        bb2 = RobotBlackboard(robot_id=2, current_role=RoleType.DEFENDER)

        sentinel = object()
        bb1.current_intent = sentinel  # type: ignore[assignment]

        assert bb2.current_intent is None

    def test_robot_ids_are_independent(self):
        bb1 = RobotBlackboard(robot_id=1, current_role=RoleType.ATTACKER)
        bb2 = RobotBlackboard(robot_id=2, current_role=RoleType.DEFENDER)
        assert bb1.robot_id != bb2.robot_id

    def test_role_changes_do_not_cross_instances(self):
        bb1 = RobotBlackboard(robot_id=0, current_role=RoleType.ATTACKER)
        bb2 = RobotBlackboard(robot_id=1, current_role=RoleType.ATTACKER)

        bb1.current_role = RoleType.GOALIE
        assert bb2.current_role == RoleType.ATTACKER
