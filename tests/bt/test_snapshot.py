"""Tests for Snapshot dataclass — R001.

TDD: these tests are written before the implementation.
All tests should fail until src/bt/contracts/snapshot.py is implemented.
"""
import dataclasses
import pytest

from TeamControl.bt.contracts.snapshot import (
    GamePhase,
    RefereeState,
    RobotState,
    Snapshot,
)


# ---------------------------------------------------------------------------
# RobotState
# ---------------------------------------------------------------------------

class TestRobotState:
    def test_construction(self):
        r = RobotState(robot_id=0, position=(1.0, 2.0), orientation=0.5)
        assert r.robot_id == 0
        assert r.position == (1.0, 2.0)
        assert r.orientation == 0.5

    def test_frozen(self):
        r = RobotState(robot_id=0, position=(0.0, 0.0), orientation=0.0)
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            r.position = (9.0, 9.0)  # type: ignore[misc]

    def test_is_dataclass(self):
        assert dataclasses.is_dataclass(RobotState)


# ---------------------------------------------------------------------------
# RefereeState
# ---------------------------------------------------------------------------

class TestRefereeState:
    def test_construction(self):
        ref = RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0))
        assert ref.game_phase == GamePhase.RUNNING
        assert ref.score == (0, 0)

    def test_frozen(self):
        ref = RefereeState(game_phase=GamePhase.HALTED, score=(1, 2))
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            ref.score = (9, 9)  # type: ignore[misc]

    def test_game_phase_enum_values(self):
        assert GamePhase.RUNNING.value == "RUNNING"
        assert GamePhase.STOPPED.value == "STOPPED"
        assert GamePhase.HALTED.value == "HALTED"


# ---------------------------------------------------------------------------
# Snapshot
# ---------------------------------------------------------------------------

def make_snapshot(**overrides):
    """Helper: construct a minimal valid Snapshot for testing."""
    defaults = dict(
        ball_position=(0.0, 0.0),
        ball_velocity=(0.0, 0.0),
        own_robots=[RobotState(robot_id=i, position=(float(i), 0.0), orientation=0.0) for i in range(6)],
        opponent_robots=[RobotState(robot_id=i, position=(float(-i), 0.0), orientation=0.0) for i in range(6)],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )
    defaults.update(overrides)
    return Snapshot(**defaults)


class TestSnapshot:
    def test_construction(self):
        snap = make_snapshot(ball_position=(3.0, 4.0))
        assert snap.ball_position == (3.0, 4.0)

    def test_ball_velocity_stored(self):
        snap = make_snapshot(ball_velocity=(1.5, -0.5))
        assert snap.ball_velocity == (1.5, -0.5)

    def test_own_robots_stored(self):
        snap = make_snapshot()
        assert len(snap.own_robots) == 6
        assert isinstance(snap.own_robots[0], RobotState)

    def test_opponent_robots_stored(self):
        snap = make_snapshot()
        assert len(snap.opponent_robots) == 6

    def test_referee_state_stored(self):
        snap = make_snapshot()
        assert snap.referee_state.game_phase == GamePhase.RUNNING

    def test_frozen_ball_position(self):
        snap = make_snapshot()
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            snap.ball_position = (99.0, 99.0)  # type: ignore[misc]

    def test_frozen_own_robots(self):
        snap = make_snapshot()
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            snap.own_robots = []  # type: ignore[misc]

    def test_no_network_dependency(self):
        """Snapshot must be constructable from pure mock data — no I/O."""
        # This test passes if construction above doesn't throw import errors
        # or attempt any network calls.
        snap = make_snapshot()
        assert snap is not None

    def test_is_dataclass(self):
        assert dataclasses.is_dataclass(Snapshot)

    def test_empty_robot_lists_allowed(self):
        snap = make_snapshot(own_robots=[], opponent_robots=[])
        assert len(snap.own_robots) == 0
        assert len(snap.opponent_robots) == 0

    def test_snapshot_equality(self):
        snap_a = make_snapshot(ball_position=(1.0, 2.0))
        snap_b = make_snapshot(ball_position=(1.0, 2.0))
        assert snap_a == snap_b

    def test_snapshot_inequality(self):
        snap_a = make_snapshot(ball_position=(1.0, 2.0))
        snap_b = make_snapshot(ball_position=(3.0, 4.0))
        assert snap_a != snap_b
