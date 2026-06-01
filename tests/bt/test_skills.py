"""Tests for skill functions: move_to, kick_at, receive_ball.

R009 (T006): Skill functions are pure stateless module-level functions.
TDD: tests written first; implementations replace the NotImplementedError stubs.
"""
from __future__ import annotations

import math

import pytest

from TeamControl.bt.contracts.motion_target import MotionTarget
from TeamControl.bt.contracts.snapshot import GamePhase, RefereeState, RobotState, Snapshot
from TeamControl.bt.skills.kick_at import kick_at
from TeamControl.bt.skills.move_to import move_to
from TeamControl.bt.skills.receive_ball import receive_ball


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

def _make_snapshot(
    robot_id: int = 0,
    robot_pos: tuple[float, float] = (0.0, 0.0),
    robot_orientation: float = 0.0,
    ball_pos: tuple[float, float] = (1.0, 0.0),
    ball_vel: tuple[float, float] = (0.0, 0.0),
) -> Snapshot:
    """Build a minimal Snapshot containing one own robot."""
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=ball_vel,
        own_robots=[
            RobotState(
                robot_id=robot_id,
                position=robot_pos,
                orientation=robot_orientation,
            )
        ],
        opponent_robots=[],
        referee_state=RefereeState(
            game_phase=GamePhase.RUNNING,
            score=(0, 0),
        ),
    )


# ---------------------------------------------------------------------------
# move_to
# ---------------------------------------------------------------------------

class TestMoveToSignatureAndReturnType:
    def test_returns_motion_target(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(1.0, 0.0))
        assert isinstance(result, MotionTarget)

    def test_accepts_optional_target_orientation(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(1.0, 0.0), target_orientation=math.pi / 2)
        assert isinstance(result, MotionTarget)

    def test_accepts_none_orientation(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(1.0, 0.0), target_orientation=None)
        assert isinstance(result, MotionTarget)


class TestMoveToArrivalMode:
    def test_arrival_mode_is_precision(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(2.0, 0.0))
        assert result.arrival_mode == "precision"


class TestMoveToVelocityDirection:
    def test_moves_right_when_target_is_right(self):
        """Robot at origin, target to the right → vx positive, vy near zero."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(5.0, 0.0))
        assert result.target_velocity[0] > 0.0, "vx should be positive (moving right)"
        assert abs(result.target_velocity[1]) < 1e-6, "vy should be ~0 when target is directly right"

    def test_moves_up_when_target_is_above(self):
        """Robot at origin, target above → vy positive, vx near zero."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(0.0, 5.0))
        assert result.target_velocity[1] > 0.0, "vy should be positive (moving up)"
        assert abs(result.target_velocity[0]) < 1e-6, "vx should be ~0 when target is directly above"

    def test_moves_left_when_target_is_left(self):
        snap = _make_snapshot(robot_pos=(3.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(0.0, 0.0))
        assert result.target_velocity[0] < 0.0, "vx should be negative (moving left)"

    def test_moves_diagonally(self):
        """Target at (3, 4) from origin → both vx, vy positive."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(3.0, 4.0))
        assert result.target_velocity[0] > 0.0
        assert result.target_velocity[1] > 0.0

    def test_zero_velocity_at_target(self):
        """Robot already at target → velocity should be zero (or near-zero)."""
        snap = _make_snapshot(robot_pos=(2.0, 3.0))
        result = move_to(snap, robot_id=0, target_pos=(2.0, 3.0))
        vx, vy = result.target_velocity
        assert abs(vx) < 1e-9
        assert abs(vy) < 1e-9


class TestMoveToOrientation:
    def test_orientation_used_when_provided(self):
        """Explicit target_orientation should appear in MotionTarget."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(1.0, 0.0), target_orientation=1.57)
        assert result.target_orientation == pytest.approx(1.57)

    def test_orientation_defaults_to_zero_when_none(self):
        """When target_orientation is None the result should still have a float."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = move_to(snap, robot_id=0, target_pos=(1.0, 0.0), target_orientation=None)
        assert isinstance(result.target_orientation, float)


class TestMoveToPurity:
    def test_same_inputs_same_outputs(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        r1 = move_to(snap, robot_id=0, target_pos=(3.0, 0.0))
        r2 = move_to(snap, robot_id=0, target_pos=(3.0, 0.0))
        assert r1 == r2

    def test_snapshot_not_mutated(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        original_pos = snap.own_robots[0].position
        move_to(snap, robot_id=0, target_pos=(5.0, 5.0))
        assert snap.own_robots[0].position == original_pos

    def test_unknown_robot_raises_value_error(self):
        snap = _make_snapshot(robot_id=0)
        with pytest.raises(ValueError):
            move_to(snap, robot_id=99, target_pos=(1.0, 0.0))


# ---------------------------------------------------------------------------
# kick_at
# ---------------------------------------------------------------------------

class TestKickAtSignatureAndReturnType:
    def test_returns_motion_target(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(5.0, 0.0))
        assert isinstance(result, MotionTarget)


class TestKickAtArrivalMode:
    def test_arrival_mode_is_fast(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(5.0, 0.0))
        assert result.arrival_mode == "fast"


class TestKickAtOrientation:
    def test_orientation_points_at_target_right(self):
        """Robot at origin, kick target at (5, 0) → orientation ≈ 0."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(5.0, 0.0))
        assert result.target_orientation == pytest.approx(0.0, abs=1e-6)

    def test_orientation_points_at_target_up(self):
        """Robot at origin, kick target at (0, 5) → orientation ≈ π/2."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(0.0, 5.0))
        assert result.target_orientation == pytest.approx(math.pi / 2, abs=1e-6)

    def test_orientation_points_at_target_diagonal(self):
        """Robot at origin, kick target at (3, 3) → orientation ≈ π/4."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(3.0, 3.0))
        assert result.target_orientation == pytest.approx(math.pi / 4, abs=1e-6)

    def test_orientation_handles_offset_robot(self):
        """Works correctly when robot is not at origin."""
        snap = _make_snapshot(robot_pos=(2.0, 2.0))
        result = kick_at(snap, robot_id=0, target_pos=(2.0, 7.0))
        assert result.target_orientation == pytest.approx(math.pi / 2, abs=1e-6)


class TestKickAtVelocity:
    def test_velocity_is_tuple_of_two_floats(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(1.0, 0.0))
        vx, vy = result.target_velocity
        assert isinstance(vx, float)
        assert isinstance(vy, float)

    def test_velocity_toward_target_right(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = kick_at(snap, robot_id=0, target_pos=(5.0, 0.0))
        assert result.target_velocity[0] > 0.0


class TestKickAtPurity:
    def test_same_inputs_same_outputs(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        r1 = kick_at(snap, robot_id=0, target_pos=(3.0, 4.0))
        r2 = kick_at(snap, robot_id=0, target_pos=(3.0, 4.0))
        assert r1 == r2

    def test_unknown_robot_raises_value_error(self):
        snap = _make_snapshot(robot_id=0)
        with pytest.raises(ValueError):
            kick_at(snap, robot_id=42, target_pos=(1.0, 0.0))


# ---------------------------------------------------------------------------
# receive_ball
# ---------------------------------------------------------------------------

class TestReceiveBallSignatureAndReturnType:
    def test_returns_motion_target(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = receive_ball(snap, robot_id=0)
        assert isinstance(result, MotionTarget)


class TestReceiveBallV1Behaviour:
    def test_arrival_mode_is_precision(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = receive_ball(snap, robot_id=0)
        assert result.arrival_mode == "precision"

    def test_velocity_is_zero(self):
        """v1 spec: stationary receive — zero velocity."""
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = receive_ball(snap, robot_id=0)
        assert result.target_velocity == (0.0, 0.0)

    def test_orientation_is_float(self):
        snap = _make_snapshot(robot_pos=(0.0, 0.0))
        result = receive_ball(snap, robot_id=0)
        assert isinstance(result.target_orientation, float)


class TestReceiveBallPurity:
    def test_same_inputs_same_outputs(self):
        snap = _make_snapshot(robot_pos=(1.0, 2.0))
        r1 = receive_ball(snap, robot_id=0)
        r2 = receive_ball(snap, robot_id=0)
        assert r1 == r2

    def test_snapshot_not_mutated(self):
        snap = _make_snapshot(robot_pos=(1.0, 2.0))
        original_ball = snap.ball_position
        receive_ball(snap, robot_id=0)
        assert snap.ball_position == original_ball

    def test_unknown_robot_raises_value_error(self):
        snap = _make_snapshot(robot_id=0)
        with pytest.raises(ValueError):
            receive_ball(snap, robot_id=7)
